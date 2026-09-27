/*=============================================================================
  The depth pass, and the per-mode tables it runs on.

  The closed depth engine is a D3D11 pipeline, and both halves of what it
  runs were read off it rather than guessed: the shaders it carries
  (shader_dump.c, in the SDK tree's tools/depthengine_open) and one recording
  of the numbers it binds while it runs (capture_engine.cmd, beside it). What
  comes out is

     projection   the nine taps of one pixel -> (real, imaginary) per
                  frequency, from the inverse of the sensor's phase matrix,
                  then rotated by the per-pixel phase correction the tables
                  hold for that frequency;
     de-alias     the three phases -> three integer wrap counts out of the
                  de-alias LUT, indexed by two rounded combinations of those
                  phases -> one radial distance, a weighted mean of the three
                  single-frequency estimates;
     radial -> Z  the radialToZ table, which is the cosine of the pixel's
                  angle off the axis;
     gates        the LUT's own invalid mark, a confidence term, the radius
                  range, how far apart the three amplitudes are, the two
                  amplitude floors, and the reflectivity.

  Everything above the tables is arithmetic this file does for itself. The
  tables are the device's own numbers, and there are three places to get them
  from, tried in this order:

    1. the device's own calibration block, read over the depth MCU's command
       set - the same 506,952-byte block the closed engine is created with.
       openk4a_block.c expands it;
    2. depth_tables.bin beside the executable, in the cache beside it, or at
       the path in OPENK4A_DEPTH_TABLES - the file form depth_model.py --write
       produces, and the only way to give another unit or another mode the
       engine's own numbers;
    3. the calibration compiled into openk4a_depth_builtin.h, which is one
       device's NFOV_UNBINNED set.

  Two things make the pass fast enough to matter, and neither changes what it
  computes. The phase correction is a constant per pixel per frequency - the
  temperature factor in it is a table scalar, not the live sensor
  temperature - so its cosine and sine are computed once, when the model is
  built, and the per-frame work is a rotation instead of two transcendental
  calls per tap. And the filter's weight is exp(cJblExpTerm * (1 - cos)),
  a function of the cosine alone: 27 million of those a frame is a table
  lookup, not an exp.
=============================================================================*/

#include "openk4a.h"

#include <math.h>

/* The filter runs eight pixels at a time where the vector unit is there, and
 * the scalar path below is what everything else - another compiler, another
 * architecture - gets. Both are held to the same reference. */
#if defined(__AVX2__)
#include <immintrin.h>
#define OPENK4A_HAVE_AVX2 1
#else
#define OPENK4A_HAVE_AVX2 0
#endif

#define OPENK4A_DEALIAS_ENTRIES 128
#define OPENK4A_WEIGHT_ENTRIES 1024
#define OPENK4A_DEPTH_TABLES_MAGIC "OK4ADPTH"
/* The name the v2 form was written under before this tree was OpenK4A. */
#define OPENK4A_DEPTH_TABLES_MAGIC_AK2 "AK2DPTH2"
/* The first form carried one device's NFOV_UNBINNED tables and no
 * dimensions; this one names the mode and the grid, so any mode's tables can
 * sit beside the executable. ak64 wrote the same layout as v1 under its own
 * magic. All four names are read, so files written before the rename load. */
#define OPENK4A_DEPTH_TABLES_MAGIC_V1 "AK2DPTH1"
#define OPENK4A_DEPTH_TABLES_MAGIC_AK64 "AK64DPTH"
/* The phase correction's temperature term, measured on this device. */
#define OPENK4A_TEMP_SLOPE 0.0099913086f
#define OPENK4A_TEMP_INTERCEPT (-0.2786280954f)

struct openk4a_depth_model
{
    k4a_depth_mode_t mode;
    uint32_t flags; /* bit 0: the engine's output is Z rather than radial */
    int width;
    int height;
    size_t pixels;

    float spatial_temp_factor;
    /* The phase correction's temperature term is linear in the sensor's own
     * temperature, and the factor is recomputed per frame from the footer:
     * the engine does the same, and a factor frozen at one capture's
     * temperature is a systematic phase error that grows as the sensor
     * warms. The two constants are measured - eight captures between 27.5 and
     * 31.7 degrees, an affine fit with a worst residual of 2.6e-5, which is
     * a fifth of a micron at the longest frequency. */
    float temperature_slope;
    float temperature_intercept;
    float built_factor;
    float smoothed_temperature;
    bool have_temperature;
    bool track_temperature;
    float p0_phase_conversion;
    float inv_2pi;
    float two_pi;
    float output_scale;
    float ab_multiplier;
    float ab_min_freq_threshold;
    float ab_sum_threshold;
    float ab_ratio_threshold;
    float max_conf_sq;
    float saturated_input;
    float saturated_ab;
    float reflectivity_min_threshold;
    float radial_min;
    float radial_max;
    float invalid_lut_entry;
    float jbl_exp_term;
    float jbl_edge_threshold;
    float center_low_ab_threshold;
    float neighbor_low_ab_threshold;
    float r_offset[2];
    float r_scale[2];
    float alpha[3];
    float c_m[3];
    float c_md[3];
    float t[2][3];
    float row_coeffs[3];
    float inverse_r[3][3];
    float inverse_i[3][3];

    float *col_coeffs;   /* height * 3 */
    float *lut;          /* OPENK4A_DEALIAS_ENTRIES * 3 */
    float *theta;        /* 3 * pixels, radians */
    float *rotate;       /* 3 * pixels * 2, cos then sin */
    uint16_t *radial_to_z; /* pixels */
    float *weight_table; /* OPENK4A_WEIGHT_ENTRIES */
    float weight_scale;

    /* The projection's output, where the filter reads it back: one plane per
     * frequency per component, so the three components of a neighbour are
     * three contiguous floats rather than three strides into one array. The
     * buffer belongs to the model and is written over every frame, which is
     * what keeps a frame from allocating thirteen megabytes and faulting in
     * thirteen megabytes of fresh pages on every pass. */
    float *proj;         /* 3 frequencies * 3 components * pixels */

    /* The frame unpacked into its tap planes: nine planes of companded,
     * signed magnitudes, which is what the projection reads and what makes
     * the projection a set of contiguous loads. A mode that bins 2x2 gets
     * its planes over the sub-image and a second set over the output grid;
     * the two are the same buffer when the mode does not bin. */
    float *tap;          /* OPENK4A_TAP_COUNT * pixels, the output grid */
    float *tap_sub;      /* OPENK4A_TAP_COUNT * sub_pixels, the sub-image */
    int binning;
    size_t sub_pixels;

    /* The three IR amplitudes of a pixel, summed before they are scaled and
     * rounded, because the reference rounds once at the end and the planes
     * are accumulated one frequency at a time. */
    float *ir_amp;       /* pixels */

    uint32_t source;
    char where[64];
};

enum
{
    OPENK4A_MODEL_FROM_BLOCK = 1,
    OPENK4A_MODEL_FROM_FILE,
    OPENK4A_MODEL_FROM_BUILTIN
};

#include "openk4a_depth_builtin.h"

/*=============================================================================
  Building a model
=============================================================================*/

static openk4a_depth_model_t *model_alloc(k4a_depth_mode_t mode, int width, int height)
{
    openk4a_depth_model_t *model = (openk4a_depth_model_t *)openk4a_alloc_zero(sizeof(openk4a_depth_model_t));
    if (model == NULL)
    {
        return NULL;
    }
    model->mode = mode;
    model->width = width;
    model->height = height;
    model->pixels = (size_t)width * (size_t)height;
    return model;
}

static bool model_arrays(openk4a_depth_model_t *model)
{
    const size_t pixels = model->pixels;
    const openk4a_mode_t *mode = openk4a_mode_of(model->mode);
    model->binning = mode != NULL ? mode->binning : 1;
    model->sub_pixels = pixels * (size_t)model->binning * (size_t)model->binning;
    model->col_coeffs = (float *)openk4a_alloc_zero((size_t)model->height * 3 * sizeof(float));
    model->lut = (float *)openk4a_alloc_zero(OPENK4A_DEALIAS_ENTRIES * 3 * sizeof(float));
    model->theta = (float *)openk4a_alloc_zero(pixels * 3 * sizeof(float));
    model->rotate = (float *)openk4a_alloc_zero(pixels * 3 * 2 * sizeof(float));
    model->radial_to_z = (uint16_t *)openk4a_alloc_zero(pixels * sizeof(uint16_t));
    model->weight_table = (float *)openk4a_alloc_zero(OPENK4A_WEIGHT_ENTRIES * sizeof(float));
    model->proj = (float *)openk4a_alloc_zero(pixels * 3 * 3 * sizeof(float));
    model->tap = (float *)openk4a_alloc_zero(pixels * OPENK4A_TAP_COUNT * sizeof(float));

    /* A mode that does not bin reads its planes straight out of the sub-
     * image, which is the output grid; there is no second copy to keep. */
    model->tap_sub = model->binning == 1
                         ? model->tap
                         : (float *)openk4a_alloc_zero(model->sub_pixels * OPENK4A_TAP_COUNT * sizeof(float));
    model->ir_amp = (float *)openk4a_alloc_zero(pixels * sizeof(float));
    return model->col_coeffs != NULL && model->lut != NULL && model->theta != NULL &&
           model->rotate != NULL && model->radial_to_z != NULL && model->weight_table != NULL &&
           model->proj != NULL && model->tap != NULL && model->tap_sub != NULL && model->ir_amp != NULL;
}

/* The filter's weight is a function of the cosine alone, so the table is
 * built from the model's own exponent and the pass indexes into it. */
static void model_build_weight_table(openk4a_depth_model_t *model)
{
    for (int i = 0; i < OPENK4A_WEIGHT_ENTRIES; i++)
    {
        const double cosine = -1.0 + 2.0 * (double)i / (double)(OPENK4A_WEIGHT_ENTRIES - 1);
        model->weight_table[i] = (float)exp((double)model->jbl_exp_term * (1.0 - cosine));
    }
    model->weight_scale = (float)((OPENK4A_WEIGHT_ENTRIES - 1) / 2);
}

/* The phase correction is theta = p0 * c + (col * rowCoeff[f] +
 * colCoeff[row][f]) * tempFactor, and the tables are the device's. It is a
 * constant, so the cosine and sine are taken here, once, rather than eight
 * hundred thousand times a frame. */
/* theta = p0 (fixed) + the spatial term scaled by the temperature factor, so
 * the rotation depends on the factor and is rebuilt when it moves far enough
 * to matter. Rebuilding costs one fixed-point measurement per pixel per
 * frequency, which is why it is done on a threshold rather than every
 * frame. */
static void model_build_rotation(openk4a_depth_model_t *model, float factor)
{
    for (int f = 0; f < 3; f++)
    {
        float *theta = model->theta + (size_t)f * model->pixels;
        float *rotate = model->rotate + (size_t)f * model->pixels * 2;
        for (int row = 0; row < model->height; row++)
        {
            const float spatial = model->col_coeffs[(size_t)row * 3 + f] * factor;
            const float row_step = model->row_coeffs[f] * factor;
            for (int column = 0; column < model->width; column++)
            {
                const size_t pixel = (size_t)row * model->width + (size_t)column;
                const float value = theta[pixel] + (float)column * row_step + spatial;
                rotate[pixel * 2] = cosf(value);
                rotate[pixel * 2 + 1] = sinf(value);
            }
        }
    }
    model->built_factor = factor;
}

/* The temperature factor for a frame, out of the frame's own footer. */
static float model_factor_for(const openk4a_depth_model_t *model, float sensor_temp_c)
{
    if (!model->track_temperature)
    {
        return model->spatial_temp_factor;
    }
    return model->temperature_slope * sensor_temp_c + model->temperature_intercept;
}

/* A quarter of a degree is what the SDK's own inertial rectifier uses for the
 * same purpose, and at that threshold the phase error it leaves is 0.06 mm at
 * the longest frequency. */
#define OPENK4A_TEMPERATURE_STEP 0.25f

void openk4a_depth_model_free(openk4a_depth_model_t *model)
{
    if (model == NULL)
    {
        return;
    }
    openk4a_free(model->col_coeffs);
    openk4a_free(model->lut);
    openk4a_free(model->theta);
    openk4a_free(model->rotate);
    openk4a_free(model->radial_to_z);
    openk4a_free(model->weight_table);
    openk4a_free(model->proj);
    if (model->tap_sub != model->tap)
    {
        openk4a_free(model->tap_sub);
    }
    openk4a_free(model->tap);
    openk4a_free(model->ir_amp);
    openk4a_free(model);
}

bool openk4a_depth_model_valid(const openk4a_depth_model_t *model)
{
    return model != NULL && model->rotate != NULL;
}

const char *openk4a_depth_model_where(const openk4a_depth_model_t *model)
{
    return model != NULL ? model->where : "";
}

/*-----------------------------------------------------------------------------
  The file form. It is the closed engine's own tables, packed once by
  depth_model.py --write; OpenK4A reads it for a unit or a mode whose block
  expansion has not been proved.
---------------------------------------------------------------------------*/

/* The file: a header, then the scalars, the small tables, the per-row column
 * coefficients, the de-alias LUT, the three phase tables in 1/65535 turns and
 * radialToZ - each the size its own dimensions imply. */
typedef struct
{
    uint32_t mode;
    uint32_t width;
    uint32_t height;
    uint32_t flags;
    float scalars[24];
    float alpha[3];
    float c_m[3];
    float c_md[3];
    float c_t[2][3];
    float row_coeffs[3];
    float inverse_r[3][3];
    float inverse_i[3][3];
} openk4a_table_header_t;

static bool table_read(void *destination, size_t size, FILE *file)
{
    return fread(destination, 1, size, file) == size;
}

static void model_from_scalars(openk4a_depth_model_t *model, const float *s)
{
    model->spatial_temp_factor = s[0];
    model->p0_phase_conversion = s[1];
    model->inv_2pi = s[2];
    model->two_pi = s[3];
    model->output_scale = s[4];
    model->ab_multiplier = s[5];
    model->ab_min_freq_threshold = s[6];
    model->ab_sum_threshold = s[7];
    model->ab_ratio_threshold = s[8];
    model->max_conf_sq = s[9];
    model->saturated_input = s[10];
    model->saturated_ab = s[11];
    model->reflectivity_min_threshold = s[12];
    model->radial_min = s[13];
    model->radial_max = s[14];
    model->invalid_lut_entry = s[15];
    model->jbl_exp_term = s[16];
    model->jbl_edge_threshold = s[17];
    model->center_low_ab_threshold = s[18];
    model->neighbor_low_ab_threshold = s[19];
    model->r_offset[0] = s[20];
    model->r_offset[1] = s[21];
    model->r_scale[0] = s[22];
    model->r_scale[1] = s[23];
}

/* Where a table file for a mode is looked for: the path the caller names in
 * OPENK4A_DEPTH_TABLES, then beside the executable, then the tree's own cache.
 * The name carries the depth mode, so the tables for several modes can sit
 * together. */
static bool model_file_find(k4a_depth_mode_t mode, char *out, size_t out_size)
{
    const char *given = getenv("OPENK4A_DEPTH_TABLES");
    if (given != NULL && given[0] != '\0')
    {
        FILE *named = fopen(given, "rb");
        if (named != NULL)
        {
            fclose(named);
            snprintf(out, out_size, "%s", given);
            return true;
        }
    }

    char exe[512];
    char exe_dir[512] = "";
    if (openk4a_exe_dir(exe, sizeof(exe)))
    {
        snprintf(exe_dir, sizeof(exe_dir), "%s", exe);
    }

    const char *candidates[] = {
        "%s\\depth_tables_%d.bin",
        "%s\\cache\\depth_tables_%d.bin",
        "%s\\..\\cache\\depth_tables_%d.bin",
        "bin\\cache\\depth_tables_%d.bin",
    };
    for (size_t i = 0; i < sizeof(candidates) / sizeof(candidates[0]); i++)
    {
        if (exe_dir[0] == '\0' && candidates[i][0] == '%')
        {
            continue;
        }
        if (strstr(candidates[i], "%d") != NULL)
        {
            snprintf(out, out_size, candidates[i], exe_dir, (int)mode);
        }
        else
        {
            snprintf(out, out_size, candidates[i], exe_dir);
        }
        FILE *probe = fopen(out, "rb");
        if (probe != NULL)
        {
            fclose(probe);
            return true;
        }
    }
    return false;
}

static openk4a_depth_model_t *model_from_file(k4a_depth_mode_t mode, const char *path)
{
    FILE *file = fopen(path, "rb");
    if (file == NULL)
    {
        return NULL;
    }
    char magic[8];
    if (fread(magic, 1, sizeof(magic), file) != sizeof(magic) ||
        (memcmp(magic, OPENK4A_DEPTH_TABLES_MAGIC, 8) != 0 && memcmp(magic, OPENK4A_DEPTH_TABLES_MAGIC_AK2, 8) != 0 &&
         memcmp(magic, OPENK4A_DEPTH_TABLES_MAGIC_AK64, 8) != 0 && memcmp(magic, OPENK4A_DEPTH_TABLES_MAGIC_V1, 8) != 0))
    {
        fclose(file);
        return NULL;
    }

    openk4a_table_header_t header;
    memset(&header, 0, sizeof(header));
    if (!table_read(&header, sizeof(header), file))
    {
        fclose(file);
        return NULL;
    }
    if (header.mode != (uint32_t)mode || header.width == 0 || header.height == 0)
    {
        openk4a_log(OPENK4A_LOG_WARNING, "%s holds %ux%u tables for mode %u, not this mode's",
                path, (unsigned)header.width, (unsigned)header.height, (unsigned)header.mode);
        fclose(file);
        return NULL;
    }

    openk4a_depth_model_t *model = model_alloc(mode, (int)header.width, (int)header.height);
    if (model == NULL || !model_arrays(model))
    {
        openk4a_depth_model_free(model);
        fclose(file);
        return NULL;
    }

    model->flags = header.flags;
    model_from_scalars(model, header.scalars);
    model->temperature_slope = OPENK4A_TEMP_SLOPE;
    model->temperature_intercept = OPENK4A_TEMP_INTERCEPT;
    model->track_temperature = true;
    memcpy(model->alpha, header.alpha, sizeof(model->alpha));
    memcpy(model->c_m, header.c_m, sizeof(model->c_m));
    memcpy(model->c_md, header.c_md, sizeof(model->c_md));
    memcpy(model->t, header.c_t, sizeof(model->t));
    memcpy(model->row_coeffs, header.row_coeffs, sizeof(model->row_coeffs));
    memcpy(model->inverse_r, header.inverse_r, sizeof(model->inverse_r));
    memcpy(model->inverse_i, header.inverse_i, sizeof(model->inverse_i));

    const size_t pixels = model->pixels;
    bool complete = table_read(model->col_coeffs, (size_t)model->height * 3 * sizeof(float), file) &&
                    table_read(model->lut, OPENK4A_DEALIAS_ENTRIES * 3 * sizeof(float), file);

    /* The file carries p0 as the engine's own R16 table, in 1/65535 turns;
     * the model wants the phase in radians, so it is converted as it is
     * read. */
    uint16_t *phase = (uint16_t *)openk4a_alloc(pixels * sizeof(uint16_t));
    if (phase == NULL)
    {
        complete = false;
    }
    for (int f = 0; f < 3; f++)
    {
        if (!complete || !table_read(phase, pixels * sizeof(uint16_t), file))
        {
            complete = false;
            break;
        }
        float *theta = model->theta + (size_t)f * model->pixels;
        for (size_t pixel = 0; pixel < pixels; pixel++)
        {
            theta[pixel] = (float)phase[pixel] * (1.0f / 65535.0f) * model->p0_phase_conversion;
        }
    }
    if (complete && !table_read(model->radial_to_z, pixels * sizeof(uint16_t), file))
    {
        complete = false;
    }
    openk4a_free(phase);
    fclose(file);

    if (!complete)
    {
        openk4a_log(OPENK4A_LOG_WARNING, "%s is short: it stops before the tables do", path);
        openk4a_depth_model_free(model);
        return NULL;
    }

    model_build_weight_table(model);
    model->source = OPENK4A_MODEL_FROM_FILE;
    snprintf(model->where, sizeof(model->where), "%s", path);
    return model;
}

/*-----------------------------------------------------------------------------
  The compiled-in form: one device's NFOV_UNBINNED calibration, with p0 as a
  row profile plus a column profile per frequency and radialToZ as a
  polynomial.
---------------------------------------------------------------------------*/

static openk4a_depth_model_t *model_from_builtin(k4a_depth_mode_t mode)
{
    if (mode != K4A_DEPTH_MODE_NFOV_UNBINNED)
    {
        return NULL;
    }

    openk4a_depth_model_t *model = model_alloc(mode, OPENK4A_BUILTIN_WIDTH, OPENK4A_BUILTIN_HEIGHT);
    if (model == NULL || !model_arrays(model))
    {
        openk4a_depth_model_free(model);
        return NULL;
    }

    model->flags = 1;
    model_from_scalars(model, openk4a_builtin_scalars);
    model->temperature_slope = OPENK4A_TEMP_SLOPE;
    model->temperature_intercept = OPENK4A_TEMP_INTERCEPT;
    model->track_temperature = true;
    memcpy(model->alpha, openk4a_builtin_alpha, sizeof(model->alpha));
    memcpy(model->c_m, openk4a_builtin_c_m, sizeof(model->c_m));
    memcpy(model->c_md, openk4a_builtin_c_md, sizeof(model->c_md));
    for (int k = 0; k < 2; k++)
    {
        for (int j = 0; j < 3; j++)
        {
            model->t[k][j] = openk4a_builtin_c_t[k * 3 + j];
        }
    }
    memcpy(model->row_coeffs, openk4a_builtin_row_coeffs, sizeof(model->row_coeffs));
    for (int k = 0; k < 3; k++)
    {
        for (int j = 0; j < 3; j++)
        {
            model->inverse_r[k][j] = openk4a_builtin_inverse_r[k * 3 + j];
            model->inverse_i[k][j] = openk4a_builtin_inverse_i[k * 3 + j];
        }
    }
    memcpy(model->col_coeffs, openk4a_builtin_col_coeffs, (size_t)model->height * 3 * sizeof(float));
    memcpy(model->lut, openk4a_builtin_lut, OPENK4A_DEALIAS_ENTRIES * 3 * sizeof(float));

    const float *const row_profile[3] = {
        openk4a_builtin_p0_row_0, openk4a_builtin_p0_row_1, openk4a_builtin_p0_row_2
    };
    const float *const col_profile[3] = {
        openk4a_builtin_p0_col_0, openk4a_builtin_p0_col_1, openk4a_builtin_p0_col_2
    };
    for (int f = 0; f < 3; f++)
    {
        float *theta = model->theta + (size_t)f * model->pixels;
        for (int row = 0; row < model->height; row++)
        {
            for (int column = 0; column < model->width; column++)
            {
                /* The profile is in turns and the composed table is its
                 * fractional part; the phase conversion takes it to radians
                 * with the rest of the model. */
                double turns = (double)row_profile[f][row] + (double)col_profile[f][column];
                turns -= floor(turns);
                theta[(size_t)row * model->width + column] = (float)(turns * model->p0_phase_conversion);
            }
        }
    }

    const float *origin = openk4a_builtin_radial_origin;
    for (int row = 0; row < model->height; row++)
    {
        const double v = ((double)row - (double)origin[1]) / (double)origin[3];
        for (int column = 0; column < model->width; column++)
        {
            const double u = ((double)column - (double)origin[0]) / (double)origin[2];
            double sum = 0.0;
            double u_power = 1.0;
            size_t k = 0;
            for (int i = 0; i <= openk4a_builtin_radial_degree; i++)
            {
                double v_power = 1.0;
                for (int j = 0; j <= openk4a_builtin_radial_degree - i; j++)
                {
                    sum += (double)openk4a_builtin_radial_coeffs[k] * u_power * v_power;
                    v_power *= v;
                    k++;
                }
                u_power *= u;
            }
            double scaled = sum * 65535.0 + 0.5;
            if (scaled < 0.0)
            {
                scaled = 0.0;
            }
            else if (scaled > 65535.0)
            {
                scaled = 65535.0;
            }
            model->radial_to_z[(size_t)row * model->width + column] = (uint16_t)scaled;
        }
    }

    model_build_weight_table(model);
    model->source = OPENK4A_MODEL_FROM_BUILTIN;
    snprintf(model->where, sizeof(model->where), "compiled in");
    return model;
}

/*-----------------------------------------------------------------------------
  The device's own calibration block, expanded. openk4a_block.c knows the
  container; this asks it for one mode's tables.
---------------------------------------------------------------------------*/

/* The device's own calibration block, expanded into one mode's tables. This
 * is the piece that makes OpenK4A work on a camera it has never seen: the block
 * is the camera's, and it holds a record set per mode. */
static bool block_expand(openk4a_depth_model_t *model,
                         const uint8_t *block,
                         size_t size,
                         const openk4a_calibration_t *calibration,
                         char *why,
                         size_t why_size);

/* The container is the camera's own record format: a run of records, each
 * with the engine's own magic at the front, a size at +4, the mode and
 * frequency in the id, and whatever the record holds after that. Reading it
 * is what makes this tree's tables the device's rather than a transcription
 * of one device's. */
static bool block_expand(openk4a_depth_model_t *model,
                         const uint8_t *block,
                         size_t size,
                         const openk4a_calibration_t *calibration,
                         char *why,
                         size_t why_size)
{
    (void)model;
    (void)calibration;

    /* What the block's own header says: the total size at +8, the firmware
     * version as text at +48. Both are worth checking even before the records
     * can be read, because a block that does not say the right size is a
     * transfer that came back short.
     *
     * Then the records: a run of them after the 256-byte header, one set per
     * mode and frequency, holding the phase ramps in an encoding this tree has
     * not read. What has been established is written down in README.md: the
     * data is not a run of plain float32 ramps, so the marker the notes name
     * (0x32CF) cannot be searched for as it occurs inside ordinary float data.
     * Until it is read, the tables come from the engine's own recording. */
    char version[16] = "";
    if (size > 56)
    {
        memcpy(version, block + 48, 12);
        version[12] = '\0';
    }
    const uint32_t claimed = size >= 12 ? openk4a_le32(block + 8) : 0;
    if (claimed != size)
    {
        snprintf(why, why_size, "the block says it is %u bytes and %zu arrived", (unsigned)claimed, size);
        return false;
    }
    snprintf(why, why_size, "the block is whole (%u bytes, firmware %s) but its records are not read yet",
             (unsigned)claimed, version);
    return false;
}

openk4a_depth_model_t *openk4a_depth_model_open(k4a_depth_mode_t mode,
                                        const uint8_t *calibration_block,
                                        size_t calibration_block_size,
                                        const openk4a_calibration_t *calibration,
                                        char *where,
                                        size_t where_size)
{
    openk4a_depth_model_t *model = NULL;

    if (calibration_block != NULL && calibration_block_size > 0)
    {
        const openk4a_mode_t *mode_info = openk4a_mode_of(mode);
        model = mode_info != NULL ? model_alloc(mode, mode_info->width, mode_info->height) : NULL;
        if (model != NULL)
        {
            if (!model_arrays(model))
            {
                openk4a_depth_model_free(model);
                model = NULL;
            }
            else
            {
                char why[128] = "";
                if (!block_expand(model, calibration_block, calibration_block_size, calibration, why, sizeof(why)))
                {
                    openk4a_log(OPENK4A_LOG_INFO, "the calibration block has no usable tables for %s: %s",
                            openk4a_mode_name(mode), why);
                    openk4a_depth_model_free(model);
                    model = NULL;
                }
                else
                {
                    model->source = OPENK4A_MODEL_FROM_BLOCK;
                    snprintf(model->where, sizeof(model->where), "the device's own calibration");
                }
            }
        }
    }

    if (model == NULL)
    {
        char path[512];
        if (model_file_find(mode, path, sizeof(path)))
        {
            model = model_from_file(mode, path);
        }
    }

    if (model == NULL)
    {
        model = model_from_builtin(mode);
    }

    if (model == NULL)
    {
        return NULL;
    }

    model_build_rotation(model, model_factor_for(model, 0.0f));
    if (where != NULL)
    {
        snprintf(where, where_size, "%s", model->where);
    }
    return model;
}

/*=============================================================================
  The pass
=============================================================================*/

/* atan2, near enough: the angle is needed as a fraction of a turn, and a
 * degree-9 minimax polynomial for atan on [-1, 1] with the quadrant handled
 * by hand is good to about 1e-7 of a turn - a micron at the longest
 * frequency. The library call is 1.1 million calls a frame and was the
 * second-largest cost in the pass. */
static float atan2_turns(float y, float x)
{
    if (x == 0.0f && y == 0.0f)
    {
        return 0.0f;
    }
    const float ax = fabsf(x);
    const float ay = fabsf(y);
    const float larger = ax > ay ? ax : ay;
    const float smaller = ax > ay ? ay : ax;
    const float ratio = smaller / larger; /* 0..1 */
    const float ratio2 = ratio * ratio;

    /* atan(r), |r| <= 1, a six-term odd minimax polynomial: Horner in r^2
     * from the highest term down, then one multiply by r. Good to about a
     * millionth of a radian, which is a quarter of a micron at the longest
     * frequency - and the engine itself uses five terms rather than the
     * library call. */
    float angle = -0.01172120f;
    angle = angle * ratio2 + 0.05265332f;
    angle = angle * ratio2 + (-0.11643287f);
    angle = angle * ratio2 + 0.19354346f;
    angle = angle * ratio2 + (-0.33262347f);
    angle = angle * ratio2 + 0.99997726f;
    angle *= ratio;
    angle *= 0.15915494309189533577f; /* radians to turns: 1 / 2 pi */

    if (ay > ax)
    {
        angle = 0.25f - angle;
    }
    if (x < 0.0f)
    {
        angle = 0.5f - angle;
    }
    if (y < 0.0f)
    {
        angle = -angle;
    }
    float turns = angle;
    if (turns < 0.0f)
    {
        turns += 1.0f;
    }
    return turns;
}

/* Round to nearest, ties to even, which is what the engine's own round does. */
static float round_half_even(float value)
{
    const float floor_value = floorf(value);
    const float fraction = value - floor_value;
    if (fraction > 0.5f)
    {
        return floor_value + 1.0f;
    }
    if (fraction < 0.5f)
    {
        return floor_value;
    }
    return fmodf(floor_value, 2.0f) == 0.0f ? floor_value : floor_value + 1.0f;
}

/*=============================================================================
  The reference pass.

  This is the pass exactly as it was first written: one pixel at a time, the
  companding through openk4a_frame_value, the neighbourhood filter walking the
  interleaved projection, and the IR decoded by a separate sweep over the
  frame's taps. It is kept, rather than deleted, because a pass rewritten for
  speed needs something to be held to that is not itself rewritten: the fast
  pass below is compared against this one pixel by pixel, and the two agree
  exactly on the fixtures.
=============================================================================*/

/* The frame carries the temperature its own phase correction depends on, so
 * the correction is rebuilt when that has moved far enough to show. */
static void depth_track_temperature(openk4a_depth_model_t *model, const uint8_t *raw, size_t size)
{
    openk4a_frame_info_t frame;
    if (!openk4a_frame_info(raw, size, &frame))
    {
        return;
    }
    /* The footer's reading moves by half a degree between neighbouring
     * frames, which is the sensor's own reporting rather than the sensor
     * warming: following it frame by frame would rebuild the correction
     * almost every frame and cost more than the correction is worth. The
     * temperature that matters is the slow one, so it is smoothed and the
     * rebuild follows that. */
    model->smoothed_temperature = model->have_temperature
                                      ? model->smoothed_temperature +
                                            (frame.sensor_temp_c - model->smoothed_temperature) * 0.0625f
                                      : frame.sensor_temp_c;
    model->have_temperature = true;
    const float factor = model_factor_for(model, model->smoothed_temperature);
    const float moved = factor > model->built_factor ? factor - model->built_factor : model->built_factor - factor;
    if (moved > OPENK4A_TEMPERATURE_STEP * model->temperature_slope)
    {
        model_build_rotation(model, factor);
    }
}

void openk4a_depth_decode_reference(openk4a_depth_model_t *model,
                                const uint8_t *raw,
                                size_t size,
                                uint16_t *depth,
                                openk4a_depth_times_t *times)
{
    if (!openk4a_depth_model_valid(model) || raw == NULL || depth == NULL)
    {
        return;
    }
    const uint64_t decode_start = openk4a_now_fine_nsec();

    depth_track_temperature(model, raw, size);

    const int width = model->width;
    const int height = model->height;
    const size_t pixels = model->pixels;
    const float inv_2pi = model->inv_2pi;
    /* OPENK4A_DEPTH_TIMING=1 says how the frame's time is split, which is the
     * only way to know whether a change helped the part it was meant to. */
    static int timing = -1;
    if (timing < 0)
    {
        const char *setting = getenv("OPENK4A_DEPTH_TIMING");
        timing = setting != NULL && setting[0] == '1' ? 1 : 0;
    }
    const uint64_t projection_start = openk4a_now_fine_nsec();

    /* One vector per frequency per pixel: the unit direction the filter
     * averages, and the amplitude the gates run on. */
    float *vectors = (float *)openk4a_alloc(pixels * 3 * 3 * sizeof(float));
    if (vectors == NULL)
    {
        return;
    }

    /* A group of eight bytes carries five pixels, and all five pixels' taps
     * come from the same set of groups: unpacking per pixel would read and
     * shift the same eight bytes five times over. A mode that bins 2x2 reads
     * four sub-pixels for every output pixel, so that one walks its pixels
     * one at a time. */
    const openk4a_mode_t *mode_info = openk4a_mode_of(model->mode);
    /* A layout this tree has not proved can be swept from outside rather than
     * recompiled: OPENK4A_TAP_STRIDE is the sub-image pixels between taps. */
    openk4a_mode_t override;
    if (mode_info != NULL)
    {
        const char *given = getenv("OPENK4A_TAP_STRIDE");
        if (given != NULL && given[0] != '\0')
        {
            override = *mode_info;
            override.tap_stride = (size_t)atoi(given);
            mode_info = &override;
        }
    }
    const size_t tap_groups = mode_info != NULL && mode_info->tap_stride != 0
                                  ? mode_info->tap_stride / OPENK4A_PIXELS_PER_GROUP
                                  : openk4a_frame_tap_groups(pixels);
    uint32_t taps[OPENK4A_TAP_COUNT][OPENK4A_PIXELS_PER_GROUP];
    const bool groupwise = mode_info != NULL && mode_info->binning == 1;
    const size_t groups = (pixels + OPENK4A_PIXELS_PER_GROUP - 1) / OPENK4A_PIXELS_PER_GROUP;
    for (size_t group = 0; group < (groupwise ? groups : 0); group++)
    {
        for (size_t tap = 0; tap < OPENK4A_TAP_COUNT; tap++)
        {
            openk4a_frame_group(raw, size, group + tap * tap_groups, taps[tap]);
        }
        for (size_t position = 0; position < OPENK4A_PIXELS_PER_GROUP; position++)
        {
            const size_t pixel = group * OPENK4A_PIXELS_PER_GROUP + position;
            if (pixel >= pixels)
            {
                /* A grid that is not a whole number of groups: 1024x1024 is
                 * not, and its last group is short. */
                continue;
            }
            for (int f = 0; f < 3; f++)
            {
                const uint32_t field_a = taps[f * 3][position];
                const uint32_t field_b = taps[f * 3 + 1][position];
                const uint32_t field_c = taps[f * 3 + 2][position];
                const float a = openk4a_frame_value(field_a);
                const float b = openk4a_frame_value(field_b);
                const float c = openk4a_frame_value(field_c);

                const float real = model->inverse_r[f][0] * a + model->inverse_r[f][1] * b +
                                   model->inverse_r[f][2] * c;
                const float imaginary = model->inverse_i[f][0] * a + model->inverse_i[f][1] * b +
                                        model->inverse_i[f][2] * c;

                const float *rotate = model->rotate + ((size_t)f * pixels + pixel) * 2;
                const float x = real * rotate[0] - imaginary * rotate[1];
                const float y = real * rotate[1] + imaginary * rotate[0];
                const float squared = x * x + y * y;
                const float magnitude = sqrtf(squared);

                float *out = vectors + ((size_t)f * pixels + pixel) * 3;
                /* One reciprocal instead of two divisions. */
                const float inverse = magnitude > 0.0f ? 1.0f / magnitude : 0.0f;
                out[0] = x * inverse;
                out[1] = y * inverse;
                out[2] = magnitude;
            }
        }
    }

    /* A mode that bins 2x2 reads four sub-pixels for every output pixel, so
     * its taps come from four places at once and are taken one pixel at a
     * time rather than a group at a time. */
    for (size_t pixel = 0; !groupwise && pixel < pixels; pixel++)
    {
        float values[OPENK4A_TAP_COUNT];
        openk4a_frame_taps(raw, size, mode_info, pixel, values);
        for (int f = 0; f < 3; f++)
        {
            const float a = values[f * 3];
            const float b = values[f * 3 + 1];
            const float c = values[f * 3 + 2];
            const float real = model->inverse_r[f][0] * a + model->inverse_r[f][1] * b + model->inverse_r[f][2] * c;
            const float imaginary =
                model->inverse_i[f][0] * a + model->inverse_i[f][1] * b + model->inverse_i[f][2] * c;
            const float *rotate = model->rotate + ((size_t)f * pixels + pixel) * 2;
            const float x = real * rotate[0] - imaginary * rotate[1];
            const float y = real * rotate[1] + imaginary * rotate[0];
            const float magnitude = sqrtf(x * x + y * y);
            float *out = vectors + ((size_t)f * pixels + pixel) * 3;
            const float inverse = magnitude > 0.0f ? 1.0f / magnitude : 0.0f;
            out[0] = x * inverse;
            out[1] = y * inverse;
            out[2] = magnitude;
        }
    }

    /* The neighbourhood filter. A neighbour counts by how nearly its phase
     * agrees, which is what keeps a depth discontinuity from being averaged
     * across; a pixel too dark to have a phase of its own is averaged without
     * the weighting, which is the engine's own cCenterLowABThreshold. */
    const float centre_dim_threshold = model->center_low_ab_threshold;
    const uint64_t filter_start = openk4a_now_fine_nsec();
    for (int row = 0; row < height; row++)
    {
        for (int column = 0; column < width; column++)
        {
            const size_t pixel = (size_t)row * width + (size_t)column;
            float phase[3];
            float amplitude[3];

            for (int f = 0; f < 3; f++)
            {
                float *const base = vectors + (size_t)f * pixels * 3;
                const float *centre = base + pixel * 3;
                const float magnitude = centre[2];
                float sum_x = 0.0f;
                float sum_y = 0.0f;
                float sum_magnitude = 0.0f;
                float sum_weight = 0.0f;
                if (magnitude > 0.0f)
                {
                    const float cx = centre[0];
                    const float cy = centre[1];
                    const bool dim = magnitude <= centre_dim_threshold;
                    const int first_row = row > 2 ? row - 2 : 0;
                    const int last_row = row + 2 < height ? row + 2 : height - 1;
                    const int first_column = column > 2 ? column - 2 : 0;
                    const int last_column = column + 2 < width ? column + 2 : width - 1;
                    for (int nb_row = first_row; nb_row <= last_row; nb_row++)
                    {
                        const float *other = base + ((size_t)nb_row * width + (size_t)first_column) * 3;
                        for (int nb_column = first_column; nb_column <= last_column; nb_column++, other += 3)
                        {
                            if (other[2] <= 0.0f)
                            {
                                continue;
                            }
                            const float nx = other[0];
                            const float ny = other[1];
                            float weight;
                            if (dim)
                            {
                                weight = 1.0f;
                            }
                            else
                            {
                                const float cosine = cx * nx + cy * ny;
                                int lookup = (int)((cosine + 1.0f) * model->weight_scale);
                                if (lookup < 0)
                                {
                                    lookup = 0;
                                }
                                else if (lookup >= OPENK4A_WEIGHT_ENTRIES)
                                {
                                    lookup = OPENK4A_WEIGHT_ENTRIES - 1;
                                }
                                weight = model->weight_table[lookup];
                            }
                            sum_x += weight * nx;
                            sum_y += weight * ny;
                            sum_magnitude += weight * other[2];
                            sum_weight += weight;
                        }
                    }
                }

                phase[f] = sum_weight > 0.0f ? atan2_turns(sum_y, sum_x) : 0.0f;
                amplitude[f] = sum_weight > 0.0f ? sum_magnitude / sum_weight : magnitude;
            }

            float coarse[2];
            for (int k = 0; k < 2; k++)
            {
                coarse[k] = model->t[k][0] * phase[0] + model->t[k][1] * phase[1] + model->t[k][2] * phase[2];
            }
            const int index = (int)((round_half_even(coarse[1]) - model->r_offset[1]) * model->r_scale[0] +
                                    (round_half_even(coarse[0]) - model->r_offset[0]));

            bool valid = index >= 0 && index < OPENK4A_DEALIAS_ENTRIES;
            float radial = 0.0f;
            if (valid)
            {
                const float *wraps = model->lut + (size_t)index * 3;
                valid = wraps[0] != model->invalid_lut_entry;

                float de_aliased[3];
                for (int k = 0; k < 3; k++)
                {
                    de_aliased[k] = phase[k] + wraps[k];
                    radial += model->alpha[k] * de_aliased[k];
                }

                float along = 0.0f;
                for (int k = 0; k < 3; k++)
                {
                    along += model->c_m[k] * de_aliased[k];
                }
                float confidence = 0.0f;
                for (int k = 0; k < 3; k++)
                {
                    const float residual = de_aliased[k] - along * model->c_md[k];
                    confidence += residual * residual;
                }

                float lowest = amplitude[0];
                float highest = amplitude[0];
                float amplitude_sum = 0.0f;
                for (int k = 0; k < 3; k++)
                {
                    if (amplitude[k] < lowest)
                    {
                        lowest = amplitude[k];
                    }
                    if (amplitude[k] > highest)
                    {
                        highest = amplitude[k];
                    }
                    amplitude_sum += amplitude[k];
                }
                const float spread = highest > 0.0f ? (highest - lowest) / highest : 1.0f;
                amplitude_sum *= model->ab_multiplier;

                valid = confidence <= model->max_conf_sq && radial >= model->radial_min &&
                        radial <= model->radial_max && spread <= model->ab_ratio_threshold &&
                        lowest > model->ab_min_freq_threshold && amplitude_sum > model->ab_sum_threshold &&
                        amplitude_sum * radial * radial >= model->reflectivity_min_threshold;
            }

            if (!valid)
            {
                depth[pixel] = 0;
                continue;
            }

            float z = radial;
            if ((model->flags & 1u) != 0)
            {
                z *= (float)model->radial_to_z[pixel] / 65535.0f;
            }
            const float value = z * 65535.0f * model->output_scale;
            depth[pixel] = (uint16_t)(value + 0.5f);
        }
    }

    openk4a_free(vectors);
    const uint64_t decode_end = openk4a_now_fine_nsec();
    if (times != NULL)
    {
        times->projection_nsec = filter_start - projection_start;
        times->filter_nsec = decode_end - filter_start;
        times->total_nsec = decode_end - decode_start;
    }
    if (timing)
    {
        openk4a_log(OPENK4A_LOG_INFO,
                "depth timing: projection %.2f ms, filter %.2f ms, total %.2f ms",
                (double)(filter_start - projection_start) / 1.0e6,
                (double)(decode_end - filter_start) / 1.0e6,
                (double)(decode_end - decode_start) / 1.0e6);
    }
    (void)inv_2pi;
}

/*=============================================================================
  The fast pass.

  The arithmetic is the reference pass's, to the last rounding; what changes
  is what it costs to arrange.

    - the companding is done in the integer domain and never leaves the
      vector unit: the twelve bits are a mantissa shift, which is a variable
      shift, and the reference's openk4a_frame_value call per tap is a call per
      tap;
    - the frame is unpacked once. Every tap plane of the frame is the same
      stream of groups, offset by a whole number of groups, so a group is
      read and unpacked once and its five fields land in all nine planes;
    - what the filter reads is three planes per frequency - the unit vector's
      two components and the amplitude - so a neighbour is three contiguous
      floats and not three strides into an interleaved array. That is what
      lets the filter run several pixels at a time;
    - the nine-tap IR is the sum of the three amplitudes the projection has
      already worked out, so the plane costs an add and a square root rather
      than a second sweep over the frame's taps;
    - the projection and the taps both live in buffers the model owns. A
      frame allocates nothing and faults in no fresh pages.
=============================================================================*/

/* The companded value of a twelve-bit field, signed, as a float: the same
 * numbers openk4a_frame_value returns, in a table of four-byte floats rather
 * than a call that widens to double and back. Two thousand and forty-eight
 * entries is 8 KB, which is small enough to sit in L1 while the pass asks
 * for ten million of them. */
static float openk4a_signed_value[2048];
static bool openk4a_signed_value_ready;

static void signed_value_init(void)
{
    if (openk4a_signed_value_ready)
    {
        return;
    }
    openk4a_code_table_init();
    for (uint32_t field = 0; field < 2048; field++)
    {
        /* openk4a_code_value is the tree's own companding table: the same
         * integer the reference's openk4a_frame_value widens to double. */
        const float magnitude = (float)openk4a_code_value(field & 0x3FF);
        openk4a_signed_value[field] = (field & 0x400) != 0 ? -magnitude : magnitude;
    }
    openk4a_signed_value_ready = true;
}

static inline float signed_value(uint32_t field)
{
    return openk4a_signed_value[field & 0x7FF];
}

/* One tap plane, read on its own rather than as part of the whole unpack.
 * Plane t of the frame's group stream starts at group t * tap_groups and runs
 * for the sub-image's worth of groups, so this is a walk over that span and
 * nothing else. */
bool openk4a_frame_tap_plane(const uint8_t *raw,
                         size_t size,
                         const openk4a_mode_t *mode,
                         size_t tap,
                         float *values,
                         size_t count)
{
    if (raw == NULL || mode == NULL || values == NULL || tap >= OPENK4A_TAP_COUNT || !mode->nine_taps)
    {
        return false;
    }
    signed_value_init();

    const size_t sub_pixels = openk4a_mode_sub_pixels(mode);
    const size_t tap_groups = mode->tap_stride != 0 ? mode->tap_stride / OPENK4A_PIXELS_PER_GROUP
                                                    : openk4a_frame_tap_groups(sub_pixels);
    const size_t groups = (sub_pixels + OPENK4A_PIXELS_PER_GROUP - 1) / OPENK4A_PIXELS_PER_GROUP;
    const size_t first = tap * tap_groups;
    size_t written = 0;
    uint32_t fields[OPENK4A_PIXELS_PER_GROUP];

    for (size_t group = 0; group < groups && written < count; group++)
    {
        const size_t byte = OPENK4A_FRAME_HEADER_BYTES + (first + group) * OPENK4A_GROUP_BYTES;
        if (byte + OPENK4A_GROUP_BYTES > size)
        {
            break;
        }
        const uint8_t *const bytes = raw + byte;
        const uint32_t lows = (uint32_t)(bytes[5] & 0x0F) | ((uint32_t)(bytes[5] >> 4) << 4) |
                              ((uint32_t)(bytes[6] & 0x0F) << 8) | ((uint32_t)(bytes[6] >> 4) << 12) |
                              ((uint32_t)(bytes[7] & 0x0F) << 16);
        for (size_t i = 0; i < OPENK4A_PIXELS_PER_GROUP; i++)
        {
            fields[i] = ((uint32_t)bytes[i] << 4) | ((lows >> (4 * i)) & 0x0F);
        }
        for (size_t i = 0; i < OPENK4A_PIXELS_PER_GROUP && written < count; i++)
        {
            values[written++] = signed_value(fields[i]);
        }
    }
    while (written < count)
    {
        values[written++] = 0.0f;
    }
    return true;
}

/* The frame's layout, resolved once for the whole pass: which taps sit how
 * far apart, where the frame's own groups end, and whether the taps are read
 * a group at a time or four sub-pixels to a pixel. */
typedef struct
{
    const openk4a_mode_t *mode;
    size_t tap_groups;
    size_t groups;
    bool groupwise;
} openk4a_layout_t;

static openk4a_layout_t depth_layout(const openk4a_depth_model_t *model)
{
    openk4a_layout_t layout;
    layout.mode = openk4a_mode_of(model->mode);
    layout.groupwise = layout.mode != NULL && layout.mode->binning == 1;
    const size_t pixels = model->pixels;
    size_t tap_stride = 0;
    if (layout.mode != NULL)
    {
        tap_stride = layout.mode->tap_stride;
    }
    /* A layout this tree has not proved can be swept from outside rather than
     * recompiled: OPENK4A_TAP_STRIDE is the sub-image pixels between taps. */
    static size_t override_stride = 0;
    static bool override_read;
    if (!override_read)
    {
        override_read = true;
        const char *given = getenv("OPENK4A_TAP_STRIDE");
        override_stride = given != NULL && given[0] != '\0' ? (size_t)atoi(given) : 0;
    }
    if (override_stride != 0)
    {
        tap_stride = override_stride;
    }
    layout.tap_groups = tap_stride != 0 ? tap_stride / OPENK4A_PIXELS_PER_GROUP
                                        : openk4a_frame_tap_groups(model->sub_pixels);
    layout.groups = (pixels + OPENK4A_PIXELS_PER_GROUP - 1) / OPENK4A_PIXELS_PER_GROUP;
    return layout;
}

/*-----------------------------------------------------------------------------
  Unpacking the frame into its tap planes.

  Plane t of the tap set is the frame's own group stream, offset by t whole
  group strides; so one walk over the frame fills every plane. A group of
  eight bytes carries five fields and lands in all nine planes at once, which
  is why the frame is read once and not nine times.

  A mode that bins 2x2 reads four sub-pixels to a pixel, so its planes are
  built over the sub-image and summed down into the output grid as they are
  written. The four are quartered and added as floats: every companded value
  is a whole number below 2^17, so a quarter of it is exact in single
  precision, and the four quarters of a 2x2 block add exactly.
---------------------------------------------------------------------------*/

static void depth_unpack(openk4a_depth_model_t *model, const uint8_t *raw, size_t size, const openk4a_layout_t *layout)
{
    const size_t sub_pixels = model->sub_pixels;
    const size_t tap_groups = layout->tap_groups;
    const size_t groups = (sub_pixels + OPENK4A_PIXELS_PER_GROUP - 1) / OPENK4A_PIXELS_PER_GROUP;
    /* The frame holds every tap plane one after another, so its last group
     * is the last group of the last plane, not of the first. */
    const size_t frame_groups = groups + (size_t)(OPENK4A_TAP_COUNT - 1) * tap_groups;
    float *const tap = model->tap_sub;
    uint32_t fields[OPENK4A_PIXELS_PER_GROUP];
    signed_value_init();

    /* Every plane is the same group stream, one tap stride apart. Plane t
     * holds the frame's groups [t * stride, t * stride + groups), so a frame
     * group belongs to at most a couple of planes and usually to exactly one:
     * the planes whose window it has reached and not yet left. Tracking those
     * two edges as counters keeps the inner loop to the groups that are
     * actually written, which over a whole frame is one write of five values
     * per frame group and not nine tests of which to skip. */
    size_t lowest = 0;
    size_t highest = 0;
    size_t opens_at = tap_groups;              /* when the next plane opens */
    size_t expires_at = groups;                /* when the first plane is full */

    for (size_t group = 0; group < frame_groups; group++)
    {
        if (group == opens_at && highest + 1 < OPENK4A_TAP_COUNT)
        {
            highest++;
            opens_at += tap_groups;
        }
        while (lowest <= highest && group == expires_at)
        {
            lowest++;
            expires_at += tap_groups;
        }
        /* The same five-to-eight unpacking openk4a_frame_group does, here rather
         * than there: it is six hundred and sixty thousand calls a frame and
         * it is eight loads and five shifts long, which is a call this pass
         * cannot afford. */
        const size_t byte = OPENK4A_FRAME_HEADER_BYTES + group * OPENK4A_GROUP_BYTES;
        if (byte + OPENK4A_GROUP_BYTES <= size)
        {
            const uint8_t *const bytes = raw + byte;
            const uint32_t lows = (uint32_t)(bytes[5] & 0x0F) | ((uint32_t)(bytes[5] >> 4) << 4) |
                                  ((uint32_t)(bytes[6] & 0x0F) << 8) | ((uint32_t)(bytes[6] >> 4) << 12) |
                                  ((uint32_t)(bytes[7] & 0x0F) << 16);
            for (size_t i = 0; i < OPENK4A_PIXELS_PER_GROUP; i++)
            {
                fields[i] = ((uint32_t)bytes[i] << 4) | ((lows >> (4 * i)) & 0x0F);
            }
        }
        else
        {
            for (size_t i = 0; i < OPENK4A_PIXELS_PER_GROUP; i++)
            {
                fields[i] = 0;
            }
        }
        for (size_t tap_index = lowest; tap_index <= highest; tap_index++)
        {
            const size_t within = group - tap_index * tap_groups;
            const size_t base = within * OPENK4A_PIXELS_PER_GROUP;
            if (base >= sub_pixels)
            {
                continue;
            }
            float *const destination = tap + tap_index * sub_pixels + base;
            size_t write = sub_pixels - base;
            if (write > OPENK4A_PIXELS_PER_GROUP)
            {
                write = OPENK4A_PIXELS_PER_GROUP;
            }
            for (size_t i = 0; i < write; i++)
            {
                destination[i] = signed_value(fields[i]);
            }
        }
    }
}

/* A mode that bins 2x2 reads four sub-pixels to a pixel: the planes above
 * are over the sub-image, and this sums each 2x2 block of them down. The
 * four quarters are exact in single precision - a companded value is a whole
 * number below 2^17, so a quarter of it is exact, and so is the sum of four
 * of them - which is why this agrees with the reference's double division to
 * the last count. */
static void depth_bin(const openk4a_depth_model_t *model)
{
    const size_t pixels = model->pixels;
    const size_t sub_pixels = model->sub_pixels;
    const size_t sub_width = (size_t)model->width * (size_t)model->binning;
    const int binning = model->binning;
    const float quarter = 1.0f / (float)(binning * binning);

    for (int tap_index = 0; tap_index < OPENK4A_TAP_COUNT; tap_index++)
    {
        const float *const source = model->tap_sub + (size_t)tap_index * sub_pixels;
        float *const destination = model->tap + (size_t)tap_index * pixels;
        for (int row = 0; row < model->height; row++)
        {
            const float *line = source + (size_t)row * (size_t)binning * sub_width;
            float *out = destination + (size_t)row * (size_t)model->width;
            for (int column = 0; column < model->width; column++)
            {
                float total = 0.0f;
                for (int dy = 0; dy < binning; dy++)
                {
                    for (int dx = 0; dx < binning; dx++)
                    {
                        total += line[(size_t)dy * sub_width + (size_t)column * (size_t)binning + (size_t)dx];
                    }
                }
                out[column] = total * quarter;
            }
        }
    }
}

/*-----------------------------------------------------------------------------
  The projection, and the IR with it.

  Per frequency: the three taps through the inverse of the sensor's phase
  matrix, the phase correction as a rotation, and the amplitude. The
  amplitude is both the weight the filter carries and the number the IR is
  the sum of, which is why the plane comes out of this pass and not a second
  one.
---------------------------------------------------------------------------*/

static void depth_project(openk4a_depth_model_t *model, uint16_t *ir)
{
    const size_t pixels = model->pixels;
    const float *const tap = model->tap;
    float *const proj = model->proj;

    for (int f = 0; f < 3; f++)
    {
        const float *const ta = tap + (size_t)(f * 3 + 0) * pixels;
        const float *const tb = tap + (size_t)(f * 3 + 1) * pixels;
        const float *const tc = tap + (size_t)(f * 3 + 2) * pixels;
        float *const qx = proj + (size_t)(f * 3 + 0) * pixels;
        float *const qy = proj + (size_t)(f * 3 + 1) * pixels;
        float *const qm = proj + (size_t)(f * 3 + 2) * pixels;
        const float r0 = model->inverse_r[f][0];
        const float r1 = model->inverse_r[f][1];
        const float r2 = model->inverse_r[f][2];
        const float i0 = model->inverse_i[f][0];
        const float i1 = model->inverse_i[f][1];
        const float i2 = model->inverse_i[f][2];
        const float *const rotate = model->rotate + (size_t)f * pixels * 2;
        size_t pixel = 0;

#if OPENK4A_HAVE_AVX2
        /* The three planes are contiguous, so this is three loads and the
         * same arithmetic the scalar loop below does. */
        {
            const __m256 vr0 = _mm256_set1_ps(r0);
            const __m256 vr1 = _mm256_set1_ps(r1);
            const __m256 vr2 = _mm256_set1_ps(r2);
            const __m256 vi0 = _mm256_set1_ps(i0);
            const __m256 vi1 = _mm256_set1_ps(i1);
            const __m256 vi2 = _mm256_set1_ps(i2);
            const __m256 zero = _mm256_setzero_ps();
            const __m256 two_thirds = _mm256_set1_ps(2.0f / 3.0f);
            /* The rotation is stored cos then sin per pixel, so eight pixels
             * are sixteen floats with the two halves interleaved; the two
             * shuffles take them apart and the permute puts them back in
             * pixel order, which is what the stores below want. */
            const __m256i deinterleave = _mm256_setr_epi32(0, 1, 4, 5, 2, 3, 6, 7);
            for (; pixel + 8 <= pixels; pixel += 8)
            {
                const __m256 a = _mm256_loadu_ps(ta + pixel);
                const __m256 b = _mm256_loadu_ps(tb + pixel);
                const __m256 c = _mm256_loadu_ps(tc + pixel);
                /* The three terms of each row of the inverse matrix, fused
                 * the way the scalar expression contracts: the first product
                 * plain, the other two accumulated into it. */
                const __m256 real = _mm256_fmadd_ps(vr2, c, _mm256_fmadd_ps(vr0, a, _mm256_mul_ps(vr1, b)));
                const __m256 imaginary = _mm256_fmadd_ps(vi2, c, _mm256_fmadd_ps(vi0, a, _mm256_mul_ps(vi1, b)));
                const __m256 rotation_low = _mm256_loadu_ps(rotate + pixel * 2);
                const __m256 rotation_high = _mm256_loadu_ps(rotate + pixel * 2 + 8);
                const __m256 cosine =
                    _mm256_permutevar8x32_ps(_mm256_shuffle_ps(rotation_low, rotation_high, 0x88), deinterleave);
                const __m256 sine =
                    _mm256_permutevar8x32_ps(_mm256_shuffle_ps(rotation_low, rotation_high, 0xDD), deinterleave);
                const __m256 x = _mm256_fmsub_ps(real, cosine, _mm256_mul_ps(imaginary, sine));
                const __m256 y = _mm256_fmadd_ps(real, sine, _mm256_mul_ps(cosine, imaginary));
                const __m256 magnitude =
                    _mm256_sqrt_ps(_mm256_fmadd_ps(x, x, _mm256_mul_ps(y, y)));
                const __m256 positive = _mm256_cmp_ps(magnitude, zero, _CMP_GT_OQ);
                const __m256 inverse = _mm256_and_ps(_mm256_div_ps(_mm256_set1_ps(1.0f), magnitude), positive);
                _mm256_storeu_ps(qx + pixel, _mm256_mul_ps(x, inverse));
                _mm256_storeu_ps(qy + pixel, _mm256_mul_ps(y, inverse));
                _mm256_storeu_ps(qm + pixel, magnitude);

                if (ir != NULL)
                {
                    __m256 squared = _mm256_mul_ps(a, a);
                    squared = _mm256_fmadd_ps(b, b, squared);
                    squared = _mm256_fmadd_ps(c, c, squared);
                    squared = _mm256_fnmadd_ps(a, b, squared);
                    squared = _mm256_fnmadd_ps(a, c, squared);
                    squared = _mm256_fnmadd_ps(b, c, squared);
                    const __m256 amplitude =
                        _mm256_mul_ps(two_thirds, _mm256_sqrt_ps(_mm256_max_ps(squared, zero)));
                    _mm256_storeu_ps(model->ir_amp + pixel,
                                     _mm256_add_ps(_mm256_loadu_ps(model->ir_amp + pixel), amplitude));
                }
            }
        }
#endif

        for (; pixel < pixels; pixel++)
        {
            const float a = ta[pixel];
            const float b = tb[pixel];
            const float c = tc[pixel];

            const float real = r0 * a + r1 * b + r2 * c;
            const float imaginary = i0 * a + i1 * b + i2 * c;
            const float cosine = rotate[pixel * 2];
            const float sine = rotate[pixel * 2 + 1];
            /* The rotation is what the phase correction is, and the
             * amplitude is the length of the rotated vector: the reference
             * takes the square root after the rotation, and the pass is held
             * to it to the last bit, so this does too rather than using the
             * rotation's invariance to save two multiplies. */
            const float x = real * cosine - imaginary * sine;
            const float y = real * sine + imaginary * cosine;
            const float magnitude = sqrtf(x * x + y * y);
            const float inverse = magnitude > 0.0f ? 1.0f / magnitude : 0.0f;
            qx[pixel] = x * inverse;
            qy[pixel] = y * inverse;
            qm[pixel] = magnitude;

            if (ir != NULL)
            {
                /* The engine's three-phase demodulation of one group, as a
                 * quadratic form: (4/9)(a^2 + b^2 + c^2 - ab - ac - bc). */
                float squared = a * a + b * b + c * c - a * b - a * c - b * c;
                if (squared < 0.0f)
                {
                    squared = 0.0f;
                }
                model->ir_amp[pixel] += (2.0f / 3.0f) * sqrtf(squared);
            }
        }
    }

    if (ir != NULL)
    {
        for (size_t pixel = 0; pixel < pixels; pixel++)
        {
            const float value = model->ir_amp[pixel] * OPENK4A_IR_TAP_SCALE;
            if (value >= 32767.0f)
            {
                ir[pixel] = 32767;
            }
            else
            {
                ir[pixel] = (uint16_t)(value + 0.5f);
            }
        }
    }
}

/*-----------------------------------------------------------------------------
  The neighbourhood filter.

  A neighbour counts by how nearly its phase agrees with the centre's, which
  is what keeps a depth discontinuity from being averaged across; a pixel too
  dark to have a phase of its own is averaged without the weighting, which is
  the engine's own cCenterLowABThreshold. The arithmetic below is the
  reference's, read off the three planes the projection wrote.
---------------------------------------------------------------------------*/

/* From the three phases and the three amplitudes to the millimetres: the
 * de-alias table, the radial distance, the confidence and the gates. Shared
 * by both paths, so there is one copy of it and not two that can drift. */
static inline void depth_conclude(const openk4a_depth_model_t *model,
                                  size_t pixel,
                                  const float phase[3],
                                  const float amplitude[3],
                                  uint16_t *depth)
{
    float coarse[2];
    for (int k = 0; k < 2; k++)
    {
        coarse[k] = model->t[k][0] * phase[0] + model->t[k][1] * phase[1] + model->t[k][2] * phase[2];
    }
    const int index = (int)((round_half_even(coarse[1]) - model->r_offset[1]) * model->r_scale[0] +
                            (round_half_even(coarse[0]) - model->r_offset[0]));

    bool valid = index >= 0 && index < OPENK4A_DEALIAS_ENTRIES;
    float radial = 0.0f;
    if (valid)
    {
        const float *wraps = model->lut + (size_t)index * 3;
        valid = wraps[0] != model->invalid_lut_entry;

        float de_aliased[3];
        for (int k = 0; k < 3; k++)
        {
            de_aliased[k] = phase[k] + wraps[k];
            radial += model->alpha[k] * de_aliased[k];
        }

        float along = 0.0f;
        for (int k = 0; k < 3; k++)
        {
            along += model->c_m[k] * de_aliased[k];
        }
        float confidence = 0.0f;
        for (int k = 0; k < 3; k++)
        {
            const float residual = de_aliased[k] - along * model->c_md[k];
            confidence += residual * residual;
        }

        float lowest = amplitude[0];
        float highest = amplitude[0];
        float amplitude_sum = 0.0f;
        for (int k = 0; k < 3; k++)
        {
            if (amplitude[k] < lowest)
            {
                lowest = amplitude[k];
            }
            if (amplitude[k] > highest)
            {
                highest = amplitude[k];
            }
            amplitude_sum += amplitude[k];
        }
        const float spread = highest > 0.0f ? (highest - lowest) / highest : 1.0f;
        amplitude_sum *= model->ab_multiplier;

        valid = confidence <= model->max_conf_sq && radial >= model->radial_min &&
                radial <= model->radial_max && spread <= model->ab_ratio_threshold &&
                lowest > model->ab_min_freq_threshold && amplitude_sum > model->ab_sum_threshold &&
                amplitude_sum * radial * radial >= model->reflectivity_min_threshold;
    }

    if (!valid)
    {
        depth[pixel] = 0;
        return;
    }

    float z = radial;
    if ((model->flags & 1u) != 0)
    {
        z *= (float)model->radial_to_z[pixel] / 65535.0f;
    }
    depth[pixel] = (uint16_t)(z * 65535.0f * model->output_scale + 0.5f);
}

/* One pixel, one neighbour at a time. */
static void depth_pixel(const openk4a_depth_model_t *model, int row, int column, uint16_t *depth)
{
    const int width = model->width;
    const int height = model->height;
    const size_t pixels = model->pixels;
    const float *const proj = model->proj;
    const size_t pixel = (size_t)row * (size_t)width + (size_t)column;
    const int first_row = row > 2 ? row - 2 : 0;
    const int last_row = row + 2 < height ? row + 2 : height - 1;
    const int first_column = column > 2 ? column - 2 : 0;
    const int last_column = column + 2 < width ? column + 2 : width - 1;
    float phase[3];
    float amplitude[3];

    for (int f = 0; f < 3; f++)
    {
        const float *const plane_x = proj + (size_t)(f * 3 + 0) * pixels;
        const float *const plane_y = proj + (size_t)(f * 3 + 1) * pixels;
        const float *const plane_m = proj + (size_t)(f * 3 + 2) * pixels;
        const float magnitude = plane_m[pixel];
        float sum_x = 0.0f;
        float sum_y = 0.0f;
        float sum_magnitude = 0.0f;
        float sum_weight = 0.0f;
        if (magnitude > 0.0f)
        {
            const float cx = plane_x[pixel];
            const float cy = plane_y[pixel];
            const bool dim = magnitude <= model->center_low_ab_threshold;
            for (int nb_row = first_row; nb_row <= last_row; nb_row++)
            {
                const size_t row_base = (size_t)nb_row * (size_t)width + (size_t)first_column;
                for (int nb_column = first_column; nb_column <= last_column; nb_column++)
                {
                    const size_t other = row_base + (size_t)(nb_column - first_column);
                    const float other_magnitude = plane_m[other];
                    if (other_magnitude <= 0.0f)
                    {
                        continue;
                    }
                    float weight;
                    if (dim)
                    {
                        weight = 1.0f;
                    }
                    else
                    {
                        const float cosine = cx * plane_x[other] + cy * plane_y[other];
                        int lookup = (int)((cosine + 1.0f) * model->weight_scale);
                        if (lookup < 0)
                        {
                            lookup = 0;
                        }
                        else if (lookup >= OPENK4A_WEIGHT_ENTRIES)
                        {
                            lookup = OPENK4A_WEIGHT_ENTRIES - 1;
                        }
                        weight = model->weight_table[lookup];
                    }
                    sum_x += weight * plane_x[other];
                    sum_y += weight * plane_y[other];
                    sum_magnitude += weight * other_magnitude;
                    sum_weight += weight;
                }
            }
        }

        phase[f] = sum_weight > 0.0f ? atan2_turns(sum_y, sum_x) : 0.0f;
        amplitude[f] = sum_weight > 0.0f ? sum_magnitude / sum_weight : magnitude;
    }
    depth_conclude(model, pixel, phase, amplitude, depth);
}

#if OPENK4A_HAVE_AVX2

/* Eight pixels of one row, whose whole 5x5 window is inside the row. Each of
 * the three components of a neighbour is eight consecutive floats in its own
 * plane, so a neighbour is three plain loads; the weight is a gather from the
 * model's 4 KB table, which is the one place the vector unit has to reach for
 * an index of its own. */
static void depth_block(const openk4a_depth_model_t *model, int row, int column, uint16_t *depth)
{
    const int width = model->width;
    const int height = model->height;
    const size_t pixels = model->pixels;
    const float *const proj = model->proj;
    const __m256 zero = _mm256_setzero_ps();
    const __m256 one = _mm256_set1_ps(1.0f);
    const __m256 scale = _mm256_set1_ps(model->weight_scale);
    const __m256 highest_entry = _mm256_set1_ps((float)(OPENK4A_WEIGHT_ENTRIES - 1));
    const __m256 dim_threshold = _mm256_set1_ps(model->center_low_ab_threshold);

    float sum_x[3][8];
    float sum_y[3][8];
    float sum_magnitude[3][8];
    float sum_weight[3][8];
    float centre_magnitude[3][8];

    for (int f = 0; f < 3; f++)
    {
        const float *const plane_x = proj + (size_t)(f * 3 + 0) * pixels;
        const float *const plane_y = proj + (size_t)(f * 3 + 1) * pixels;
        const float *const plane_m = proj + (size_t)(f * 3 + 2) * pixels;
        const size_t base = (size_t)row * (size_t)width + (size_t)column;
        const __m256 cx = _mm256_loadu_ps(plane_x + base);
        const __m256 cy = _mm256_loadu_ps(plane_y + base);
        const __m256 cm = _mm256_loadu_ps(plane_m + base);
        const __m256 active = _mm256_cmp_ps(cm, zero, _CMP_GT_OQ);
        const __m256 dim = _mm256_cmp_ps(cm, dim_threshold, _CMP_LE_OQ);

        __m256 acc_x = zero;
        __m256 acc_y = zero;
        __m256 acc_m = zero;
        __m256 acc_w = zero;
        for (int dr = -2; dr <= 2; dr++)
        {
            const int other_row = row + dr;
            if (other_row < 0 || other_row >= height)
            {
                continue;
            }
            const float *const nx_row = plane_x + (size_t)other_row * (size_t)width;
            const float *const ny_row = plane_y + (size_t)other_row * (size_t)width;
            const float *const nm_row = plane_m + (size_t)other_row * (size_t)width;
            for (int dc = -2; dc <= 2; dc++)
            {
                const int at = column + dc;
                const __m256 nx = _mm256_loadu_ps(nx_row + at);
                const __m256 ny = _mm256_loadu_ps(ny_row + at);
                const __m256 nm = _mm256_loadu_ps(nm_row + at);
                const __m256 present = _mm256_cmp_ps(nm, zero, _CMP_GT_OQ);
                const __m256 cosine = _mm256_fmadd_ps(cx, nx, _mm256_mul_ps(cy, ny));
                __m256 where = _mm256_mul_ps(_mm256_add_ps(cosine, one), scale);
                where = _mm256_max_ps(where, zero);
                where = _mm256_min_ps(where, highest_entry);
                __m256 weight = _mm256_i32gather_ps(model->weight_table, _mm256_cvttps_epi32(where), 4);
                weight = _mm256_blendv_ps(weight, one, dim);
                weight = _mm256_and_ps(weight, present);
                acc_x = _mm256_fmadd_ps(weight, nx, acc_x);
                acc_y = _mm256_fmadd_ps(weight, ny, acc_y);
                acc_m = _mm256_fmadd_ps(weight, nm, acc_m);
                acc_w = _mm256_add_ps(acc_w, weight);
            }
        }

        /* A lane whose centre has no phase of its own is the reference's
         * "the neighbourhood is not walked": its sums stay at zero. */
        _mm256_storeu_ps(sum_x[f], _mm256_and_ps(acc_x, active));
        _mm256_storeu_ps(sum_y[f], _mm256_and_ps(acc_y, active));
        _mm256_storeu_ps(sum_magnitude[f], _mm256_and_ps(acc_m, active));
        _mm256_storeu_ps(sum_weight[f], _mm256_and_ps(acc_w, active));
        _mm256_storeu_ps(centre_magnitude[f], cm);
    }

    for (int lane = 0; lane < 8; lane++)
    {
        float phase[3];
        float amplitude[3];
        for (int f = 0; f < 3; f++)
        {
            if (sum_weight[f][lane] > 0.0f)
            {
                phase[f] = atan2_turns(sum_y[f][lane], sum_x[f][lane]);
                amplitude[f] = sum_magnitude[f][lane] / sum_weight[f][lane];
            }
            else
            {
                phase[f] = 0.0f;
                amplitude[f] = centre_magnitude[f][lane];
            }
        }
        depth_conclude(model,
                       (size_t)row * (size_t)width + (size_t)column + (size_t)lane,
                       phase,
                       amplitude,
                       depth);
    }
}

#endif /* OPENK4A_HAVE_AVX2 */

static void depth_filter(const openk4a_depth_model_t *model, uint16_t *depth)
{
    const int width = model->width;
    const int height = model->height;
    /* The block path needs the whole window inside the row; the columns it
     * cannot cover are the two at each edge and whatever is left over. */
    const int block_first = 2;
    const int block_last = width - 2;

    for (int row = 0; row < height; row++)
    {
        int column = 0;
        for (; column < block_first && column < width; column++)
        {
            depth_pixel(model, row, column, depth);
        }
#if OPENK4A_HAVE_AVX2
        for (; column + 8 <= block_last; column += 8)
        {
            depth_block(model, row, column, depth);
        }
#endif
        for (; column < width; column++)
        {
            depth_pixel(model, row, column, depth);
        }
    }
}

/*-----------------------------------------------------------------------------
  The entry points.
---------------------------------------------------------------------------*/

void openk4a_depth_decode_full(openk4a_depth_model_t *model,
                           const uint8_t *raw,
                           size_t size,
                           uint16_t *depth,
                           uint16_t *ir,
                           openk4a_depth_times_t *times)
{
    if (!openk4a_depth_model_valid(model) || raw == NULL || (depth == NULL && ir == NULL))
    {
        return;
    }
    const uint64_t start = openk4a_now_fine_nsec();
    depth_track_temperature(model, raw, size);

    const openk4a_layout_t layout = depth_layout(model);
    if (layout.mode == NULL)
    {
        /* A mode the table does not carry: the reference is the only thing
         * that knows how to read it. */
        openk4a_depth_decode_reference(model, raw, size, depth, times);
        return;
    }

    if (ir != NULL)
    {
        memset(model->ir_amp, 0, model->pixels * sizeof(float));
    }

    const uint64_t prepare_start = openk4a_now_fine_nsec();
    depth_unpack(model, raw, size, &layout);
    if (model->binning != 1)
    {
        depth_bin(model);
    }
    const uint64_t project_start = openk4a_now_fine_nsec();
    depth_project(model, ir);
    const uint64_t filter_start = openk4a_now_fine_nsec();
    if (depth != NULL)
    {
        depth_filter(model, depth);
    }
    const uint64_t end = openk4a_now_fine_nsec();

    if (times != NULL)
    {
        times->projection_nsec = filter_start - prepare_start;
        times->filter_nsec = end - filter_start;
        times->total_nsec = end - start;
    }
    (void)project_start;
}

void openk4a_depth_decode(openk4a_depth_model_t *model, const uint8_t *raw, size_t size, uint16_t *depth)
{
    openk4a_depth_decode_full(model, raw, size, depth, NULL, NULL);
}

void openk4a_depth_decode_timed(openk4a_depth_model_t *model,
                            const uint8_t *raw,
                            size_t size,
                            uint16_t *depth,
                            openk4a_depth_times_t *times)
{
    openk4a_depth_decode_full(model, raw, size, depth, NULL, times);
}
