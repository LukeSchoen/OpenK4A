/*=============================================================================
  Calibration: the device's document, and the maths built on it.

  The camera hands over a JSON document with one entry per camera and one per
  inertial sensor. It is unitized and corner-centred - the principal point
  and focal length are fractions of the sensor, and (0,0) is the top left
  corner of the top left pixel - and a mode's calibration is that, multiplied
  by the mode's own grid, cropped, and shifted by half a pixel into the
  pixel-centred convention the API uses. The arithmetic is the SDK's own
  (mode_specific_calibration.c), because a program that has a k4a calibration
  in its hands expects the same numbers out of it.

  The projection model is Brown-Conrady, and the numbers in it are the
  device's: the same fifteen parameters the closed SDK reads. The
  project/unproject pair below is the SDK's own iterative one, because a
  closed-form inverse of a rational distortion model does not exist and
  approximating it moves points.
=============================================================================*/

#include "openk4a.h"

#include <math.h>

/*=============================================================================
  The document
=============================================================================*/

static k4a_calibration_model_type_t distortion_from_name(const char *name)
{
    if (name == NULL)
    {
        return K4A_CALIBRATION_LENS_DISTORTION_MODEL_UNKNOWN;
    }
    /* The document names the model; the enum numbers it, and the two orders
     * are not the same - Brown-Conrady is the last value, not the first. */
    static const struct
    {
        const char *name;
        k4a_calibration_model_type_t type;
    } names[] = {
        { "CALIBRATION_LensDistortionModelBrownConrady", K4A_CALIBRATION_LENS_DISTORTION_MODEL_BROWN_CONRADY },
        { "CALIBRATION_LensDistortionModelTheta", K4A_CALIBRATION_LENS_DISTORTION_MODEL_THETA },
        { "CALIBRATION_LensDistortionModelPolynomial3K", K4A_CALIBRATION_LENS_DISTORTION_MODEL_POLYNOMIAL_3K },
        { "CALIBRATION_LensDistortionModelRational6KT", K4A_CALIBRATION_LENS_DISTORTION_MODEL_RATIONAL_6KT },
    };
    for (size_t i = 0; i < sizeof(names) / sizeof(names[0]); i++)
    {
        if (strcmp(name, names[i].name) == 0)
        {
            return names[i].type;
        }
    }
    return K4A_CALIBRATION_LENS_DISTORTION_MODEL_UNKNOWN;
}

/* The document stores a pose as a rotation and a translation in metres; the
 * API is in millimetres, which is the depth image's own unit. */
static bool fill_extrinsics(openk4a_json_t *rt, k4a_calibration_extrinsics_t *extrinsics)
{
    openk4a_json_t *rotation = openk4a_json_get(rt, "Rotation");
    openk4a_json_t *translation = openk4a_json_get(rt, "Translation");
    if (rotation == NULL || translation == NULL)
    {
        return false;
    }
    if (openk4a_json_floats(rotation, extrinsics->rotation, 9) != 9)
    {
        return false;
    }
    if (openk4a_json_floats(translation, extrinsics->translation, 3) != 3)
    {
        return false;
    }
    for (int i = 0; i < 3; i++)
    {
        extrinsics->translation[i] *= 1000.0f;
    }
    return true;
}

static bool fill_intrinsics(openk4a_json_t *intrinsics, k4a_calibration_intrinsics_t *out)
{
    openk4a_json_t *count = openk4a_json_get(intrinsics, "ModelParameterCount");
    openk4a_json_t *type = openk4a_json_get(intrinsics, "ModelType");
    openk4a_json_t *parameters = openk4a_json_get(intrinsics, "ModelParameters");
    if (!openk4a_json_is_number(count) || !openk4a_json_is_string(type) || parameters == NULL)
    {
        return false;
    }

    unsigned int parameter_count = (unsigned int)openk4a_json_number(count);
    if (parameter_count > 15)
    {
        parameter_count = 15;
    }
    out->parameter_count = parameter_count;
    out->type = distortion_from_name(openk4a_json_string(type));
    if (out->type == K4A_CALIBRATION_LENS_DISTORTION_MODEL_UNKNOWN)
    {
        return false;
    }
    return openk4a_json_floats(parameters, out->parameters.v, parameter_count) == parameter_count;
}

static bool fill_camera(openk4a_json_t *camera, k4a_calibration_camera_t *out)
{
    openk4a_json_t *intrinsics = openk4a_json_get(camera, "Intrinsics");
    openk4a_json_t *rt = openk4a_json_get(camera, "Rt");
    openk4a_json_t *width = openk4a_json_get(camera, "SensorWidth");
    openk4a_json_t *height = openk4a_json_get(camera, "SensorHeight");
    openk4a_json_t *radius = openk4a_json_get(camera, "MetricRadius");
    if (intrinsics == NULL || rt == NULL || !openk4a_json_is_number(width) || !openk4a_json_is_number(height) ||
        !openk4a_json_is_number(radius))
    {
        return false;
    }
    if (!fill_extrinsics(rt, &out->extrinsics) || !fill_intrinsics(intrinsics, &out->intrinsics))
    {
        return false;
    }
    out->resolution_width = (int)openk4a_json_number(width);
    out->resolution_height = (int)openk4a_json_number(height);
    out->metric_radius = (float)openk4a_json_number(radius);
    /* A metric radius of zero means the fit failed to estimate it; the SDK
     * uses the value that corresponds to a 120-degree field of view. */
    if (out->metric_radius <= 0.0001f)
    {
        out->metric_radius = 1.7f;
    }
    return true;
}

static bool fill_imu(openk4a_json_t *sensor, openk4a_imu_calibration_t *out)
{
    openk4a_json_t *bias = openk4a_json_get(sensor, "BiasTemperatureModel");
    openk4a_json_t *bias_uncertainty = openk4a_json_get(sensor, "BiasUncertainty");
    openk4a_json_t *mixing = openk4a_json_get(sensor, "MixingMatrixTemperatureModel");
    openk4a_json_t *mask = openk4a_json_get(sensor, "ModelTypeMask");
    openk4a_json_t *noise = openk4a_json_get(sensor, "Noise");
    openk4a_json_t *rt = openk4a_json_get(sensor, "Rt");
    openk4a_json_t *scaling = openk4a_json_get(sensor, "SecondOrderScaling");
    openk4a_json_t *bounds = openk4a_json_get(sensor, "TemperatureBounds");
    openk4a_json_t *temperature = openk4a_json_get(sensor, "TemperatureC");
    if (bias == NULL || bias_uncertainty == NULL || mixing == NULL || !openk4a_json_is_number(mask) || noise == NULL ||
        rt == NULL || scaling == NULL || bounds == NULL || !openk4a_json_is_number(temperature))
    {
        return false;
    }
    return openk4a_json_floats(bias, out->bias_temperature_model, 3 * OPENK4A_IMU_MODEL_COEFFICIENTS) ==
               3 * OPENK4A_IMU_MODEL_COEFFICIENTS &&
           openk4a_json_floats(bias_uncertainty, out->bias_uncertainty, 3) == 3 &&
           openk4a_json_floats(mixing, out->mixing_matrix_temperature_model, 9 * OPENK4A_IMU_MODEL_COEFFICIENTS) ==
               9 * OPENK4A_IMU_MODEL_COEFFICIENTS &&
           openk4a_json_floats(noise, out->noise, 6) == 6 &&
           openk4a_json_floats(scaling, out->second_order_scaling, 9) == 9 &&
           openk4a_json_floats(bounds, out->temperature_bounds, 2) == 2 && fill_extrinsics(rt, &out->depth_to_imu) &&
           (out->temperature_in_c = (float)openk4a_json_number(temperature), true) &&
           (out->model_type_mask = (uint32_t)openk4a_json_number(mask), true);
}

/* One camera or one inertial sensor, found by the name the document gives its
 * kind. The depth camera is D0 and the colour one PV0; older firmware calls
 * the field Purpose rather than Location, so both spellings are tried. */
static bool fill_named(openk4a_json_t *array,
                       const char *const *keys,
                       const char *value,
                       bool (*fill)(openk4a_json_t *, void *),
                       void *out)
{
    for (size_t i = 0; i < openk4a_json_count(array); i++)
    {
        openk4a_json_t *entry = openk4a_json_at(array, i);
        for (size_t k = 0; keys[k] != NULL; k++)
        {
            openk4a_json_t *name = openk4a_json_get(entry, keys[k]);
            if (name != NULL && openk4a_json_is_string(name) && strcmp(openk4a_json_string(name), value) == 0)
            {
                return fill(entry, out);
            }
        }
    }
    return false;
}

static bool fill_camera_thunk(openk4a_json_t *entry, void *out)
{
    return fill_camera(entry, (k4a_calibration_camera_t *)out);
}

static bool fill_imu_thunk(openk4a_json_t *entry, void *out)
{
    return fill_imu(entry, (openk4a_imu_calibration_t *)out);
}

bool openk4a_calibration_parse(const char *json, size_t json_size, openk4a_calibration_t *calibration)
{
    memset(calibration, 0, sizeof(*calibration));

    openk4a_json_t *document = openk4a_json_parse(json, json_size);
    if (document == NULL)
    {
        openk4a_log(OPENK4A_LOG_ERROR, "the calibration document did not parse");
        return false;
    }

    openk4a_json_t *info = openk4a_json_get(document, "CalibrationInformation");
    openk4a_json_t *cameras = info != NULL ? openk4a_json_get(info, "Cameras") : NULL;
    openk4a_json_t *inertial = info != NULL ? openk4a_json_get(info, "InertialSensors") : NULL;
    if (info == NULL || cameras == NULL)
    {
        openk4a_log(OPENK4A_LOG_ERROR, "the calibration document has no camera list");
        openk4a_json_free(document);
        return false;
    }

    const char *cam_keys[] = { "Location", "Purpose", NULL };
    const char *imu_keys[] = { "SensorType", NULL };

    calibration->valid = fill_named(cameras, cam_keys, "CALIBRATION_CameraLocationD0", fill_camera_thunk,
                                    &calibration->depth) &&
                         fill_named(cameras, cam_keys, "CALIBRATION_CameraLocationPV0", fill_camera_thunk,
                                    &calibration->color);
    if (!calibration->valid)
    {
        /* Very old firmware names the two cameras by purpose only. */
        calibration->valid = fill_named(cameras, cam_keys, "CALIBRATION_CameraPurposeDepth", fill_camera_thunk,
                                        &calibration->depth) &&
                             fill_named(cameras, cam_keys, "CALIBRATION_CameraPurposeColor", fill_camera_thunk,
                                        &calibration->color);
    }

    calibration->gyro.depth_to_imu.rotation[0] = 1.0f;
    calibration->gyro.depth_to_imu.rotation[4] = 1.0f;
    calibration->gyro.depth_to_imu.rotation[8] = 1.0f;
    calibration->accel = calibration->gyro;
    if (inertial != NULL)
    {
        fill_named(inertial, imu_keys, "CALIBRATION_InertialSensorType_Gyro", fill_imu_thunk, &calibration->gyro);
        fill_named(inertial, imu_keys, "CALIBRATION_InertialSensorType_Accelerometer", fill_imu_thunk,
                   &calibration->accel);
    }

    /* The document is kept whole, because k4a_device_get_raw_calibration hands
     * it back byte for byte. */
    calibration->json = (char *)openk4a_alloc(json_size + 1);
    if (calibration->json != NULL)
    {
        memcpy(calibration->json, json, json_size);
        calibration->json[json_size] = '\0';
        calibration->json_size = json_size + 1;
    }

    const bool valid = calibration->valid;
    if (valid)
    {
        openk4a_log(OPENK4A_LOG_INFO,
                "calibration: %d depth parameters, %dx%d sensor, %d colour parameters, %dx%d sensor",
                (int)calibration->depth.intrinsics.parameter_count,
                calibration->depth.resolution_width,
                calibration->depth.resolution_height,
                (int)calibration->color.intrinsics.parameter_count,
                calibration->color.resolution_width,
                calibration->color.resolution_height);
    }
    openk4a_json_free(document);
    return valid;
}

void openk4a_calibration_free(openk4a_calibration_t *calibration)
{
    if (calibration == NULL)
    {
        return;
    }
    openk4a_free(calibration->json);
    memset(calibration, 0, sizeof(*calibration));
}

/*=============================================================================
  Denormalizing to a mode's grid
=============================================================================*/

typedef struct
{
    int binned_width;
    int binned_height;
    int crop_x;
    int crop_y;
    int output_width;
    int output_height;
} openk4a_mode_info_t;

/* The raw document is unitized and corner-centred; the API is pixelized and
 * pixel-centred. Multiplying by the sensor, taking off the crop, and taking
 * off the half pixel is the whole conversion. */
static bool denormalize(const k4a_calibration_camera_t *raw,
                        const openk4a_mode_info_t *info,
                        k4a_calibration_camera_t *out)
{
    if (info->binned_width <= 0 || info->binned_height <= 0 || info->output_width <= 0 || info->output_height <= 0)
    {
        return false;
    }
    memcpy(out, raw, sizeof(*out));

    k4a_calibration_intrinsic_parameters_t *params = &out->intrinsics.parameters;
    const float cx = params->param.cx * (float)info->binned_width - (float)info->crop_x;
    const float cy = params->param.cy * (float)info->binned_height - (float)info->crop_y;
    const float fx = params->param.fx * (float)info->binned_width;
    const float fy = params->param.fy * (float)info->binned_height;

    params->param.cx = cx - 0.5f;
    params->param.cy = cy - 0.5f;
    params->param.fx = fx;
    params->param.fy = fy;

    out->resolution_width = info->output_width;
    out->resolution_height = info->output_height;
    return true;
}

bool openk4a_calibration_grid(k4a_depth_mode_t mode, int *width, int *height)
{
    const openk4a_mode_t *mode_info = openk4a_mode_of(mode);
    if (mode_info == NULL)
    {
        return false;
    }
    if (width != NULL)
    {
        *width = mode_info->width;
    }
    if (height != NULL)
    {
        *height = mode_info->height;
    }
    return true;
}

bool openk4a_calibration_for_depth_mode(const openk4a_calibration_t *calibration,
                                    k4a_depth_mode_t mode,
                                    k4a_calibration_camera_t *out)
{
    if (calibration == NULL || !calibration->valid)
    {
        return false;
    }
    if (calibration->depth.resolution_width != 1024 || calibration->depth.resolution_height != 1024)
    {
        openk4a_log(OPENK4A_LOG_ERROR, "the depth calibration is %dx%d, not the sensor's 1024x1024",
                calibration->depth.resolution_width, calibration->depth.resolution_height);
        return false;
    }

    openk4a_mode_info_t info;
    switch (mode)
    {
    case K4A_DEPTH_MODE_NFOV_2X2BINNED:
        info = (openk4a_mode_info_t){ 512, 512, 96, 90, 320, 288 };
        break;
    case K4A_DEPTH_MODE_NFOV_UNBINNED:
        info = (openk4a_mode_info_t){ 1024, 1024, 192, 180, 640, 576 };
        break;
    case K4A_DEPTH_MODE_WFOV_2X2BINNED:
        info = (openk4a_mode_info_t){ 512, 512, 0, 0, 512, 512 };
        break;
    case K4A_DEPTH_MODE_WFOV_UNBINNED:
    case K4A_DEPTH_MODE_PASSIVE_IR:
        info = (openk4a_mode_info_t){ 1024, 1024, 0, 0, 1024, 1024 };
        break;
    default:
        return false;
    }
    return denormalize(&calibration->depth, &info, out);
}

bool openk4a_color_grid(k4a_color_resolution_t resolution, int *width, int *height)
{
    switch (resolution)
    {
    case K4A_COLOR_RESOLUTION_720P:
        *width = 1280;
        *height = 720;
        return true;
    case K4A_COLOR_RESOLUTION_1080P:
        *width = 1920;
        *height = 1080;
        return true;
    case K4A_COLOR_RESOLUTION_1440P:
        *width = 2560;
        *height = 1440;
        return true;
    case K4A_COLOR_RESOLUTION_1536P:
        *width = 2048;
        *height = 1536;
        return true;
    case K4A_COLOR_RESOLUTION_2160P:
        *width = 3840;
        *height = 2160;
        return true;
    case K4A_COLOR_RESOLUTION_3072P:
        *width = 4096;
        *height = 3072;
        return true;
    default:
        return false;
    }
}

bool openk4a_calibration_for_color_resolution(const openk4a_calibration_t *calibration,
                                          k4a_color_resolution_t resolution,
                                          k4a_calibration_camera_t *out)
{
    if (calibration == NULL || !calibration->valid)
    {
        return false;
    }

    /* The colour calibration's own grid, and whether it is the legacy 16:9
     * one, which has to be taken to 4:3 first - the same two steps the SDK
     * takes. */
    k4a_calibration_camera_t base = calibration->color;
    const int raw_width = base.resolution_width;
    const int raw_height = base.resolution_height;
    if (raw_width * 9 / 16 == raw_height)
    {
        const openk4a_mode_info_t legacy = { 4096, 2304, 0, -384, 4096, 3072 };
        k4a_calibration_camera_t converted;
        memcpy(&converted, &base, sizeof(converted));
        k4a_calibration_intrinsic_parameters_t *params = &converted.intrinsics.parameters;
        params->param.cx = params->param.cx * (float)legacy.binned_width / (float)legacy.output_width;
        params->param.cy = (params->param.cy * (float)legacy.binned_height - (float)legacy.crop_y) /
                          (float)legacy.output_height;
        params->param.fx = params->param.fx * (float)legacy.binned_width / (float)legacy.output_width;
        params->param.fy = params->param.fy * (float)legacy.binned_height / (float)legacy.output_height;
        converted.resolution_width = legacy.output_width;
        converted.resolution_height = legacy.output_height;
        base = converted;
    }
    else if (raw_width * 3 / 4 != raw_height)
    {
        openk4a_log(OPENK4A_LOG_ERROR, "the colour calibration is %dx%d, neither 16:9 nor 4:3", raw_width, raw_height);
        return false;
    }

    openk4a_mode_info_t info;
    switch (resolution)
    {
    case K4A_COLOR_RESOLUTION_720P:
        info = (openk4a_mode_info_t){ 1280, 960, 0, 120, 1280, 720 };
        break;
    case K4A_COLOR_RESOLUTION_1080P:
        info = (openk4a_mode_info_t){ 1920, 1440, 0, 180, 1920, 1080 };
        break;
    case K4A_COLOR_RESOLUTION_1440P:
        info = (openk4a_mode_info_t){ 2560, 1920, 0, 240, 2560, 1440 };
        break;
    case K4A_COLOR_RESOLUTION_1536P:
        info = (openk4a_mode_info_t){ 2048, 1536, 0, 0, 2048, 1536 };
        break;
    case K4A_COLOR_RESOLUTION_2160P:
        info = (openk4a_mode_info_t){ 3840, 2880, 0, 360, 3840, 2160 };
        break;
    case K4A_COLOR_RESOLUTION_3072P:
        info = (openk4a_mode_info_t){ 4096, 3072, 0, 0, 4096, 3072 };
        break;
    default:
        return false;
    }
    return denormalize(&base, &info, out);
}

/*=============================================================================
  Extrinsics
=============================================================================*/

static void extrinsics_multiply(const k4a_calibration_extrinsics_t *a,
                                const k4a_calibration_extrinsics_t *b,
                                k4a_calibration_extrinsics_t *ab)
{
    for (int row = 0; row < 3; row++)
    {
        ab->translation[row] = a->rotation[row * 3] * b->translation[0] +
                               a->rotation[row * 3 + 1] * b->translation[1] +
                               a->rotation[row * 3 + 2] * b->translation[2] + a->translation[row];
        for (int column = 0; column < 3; column++)
        {
            ab->rotation[row * 3 + column] = a->rotation[row * 3] * b->rotation[column] +
                                             a->rotation[row * 3 + 1] * b->rotation[3 + column] +
                                             a->rotation[row * 3 + 2] * b->rotation[6 + column];
        }
    }
}

static void extrinsics_invert(const k4a_calibration_extrinsics_t *x, k4a_calibration_extrinsics_t *inverse)
{
    for (int row = 0; row < 3; row++)
    {
        for (int column = 0; column < 3; column++)
        {
            inverse->rotation[row * 3 + column] = x->rotation[column * 3 + row];
        }
    }
    for (int row = 0; row < 3; row++)
    {
        inverse->translation[row] = -(inverse->rotation[row * 3] * x->translation[0] +
                                      inverse->rotation[row * 3 + 1] * x->translation[1] +
                                      inverse->rotation[row * 3 + 2] * x->translation[2]);
    }
}

/* source_to_target from the two poses the document gives, which are both
 * world_to_camera. */
static void extrinsic_between(const k4a_calibration_extrinsics_t *world_to_source,
                              const k4a_calibration_extrinsics_t *world_to_target,
                              k4a_calibration_extrinsics_t *source_to_target)
{
    k4a_calibration_extrinsics_t source_to_world;
    extrinsics_invert(world_to_source, &source_to_world);
    extrinsics_multiply(world_to_target, &source_to_world, source_to_target);
}

bool openk4a_calibration_build(const openk4a_calibration_t *calibration,
                           k4a_depth_mode_t depth_mode,
                           k4a_color_resolution_t color_resolution,
                           k4a_calibration_t *out)
{
    memset(out, 0, sizeof(*out));
    if (depth_mode == K4A_DEPTH_MODE_OFF && color_resolution == K4A_COLOR_RESOLUTION_OFF)
    {
        return false;
    }
    if (depth_mode != K4A_DEPTH_MODE_OFF &&
        !openk4a_calibration_for_depth_mode(calibration, depth_mode, &out->depth_camera_calibration))
    {
        return false;
    }
    if (color_resolution != K4A_COLOR_RESOLUTION_OFF &&
        !openk4a_calibration_for_color_resolution(calibration, color_resolution, &out->color_camera_calibration))
    {
        return false;
    }

    const k4a_calibration_extrinsics_t *extrinsics[K4A_CALIBRATION_TYPE_NUM];
    extrinsics[K4A_CALIBRATION_TYPE_DEPTH] = &out->depth_camera_calibration.extrinsics;
    extrinsics[K4A_CALIBRATION_TYPE_COLOR] = &out->color_camera_calibration.extrinsics;
    extrinsics[K4A_CALIBRATION_TYPE_GYRO] = &calibration->gyro.depth_to_imu;
    extrinsics[K4A_CALIBRATION_TYPE_ACCEL] = &calibration->accel.depth_to_imu;
    for (int source = 0; source < (int)K4A_CALIBRATION_TYPE_NUM; source++)
    {
        for (int target = 0; target < (int)K4A_CALIBRATION_TYPE_NUM; target++)
        {
            extrinsic_between(extrinsics[source], extrinsics[target], &out->extrinsics[source][target]);
        }
    }
    out->depth_mode = depth_mode;
    out->color_resolution = color_resolution;
    return true;
}

/*=============================================================================
  Projecting and unprojecting
=============================================================================*/

static bool intrinsics_valid(const k4a_calibration_camera_t *camera)
{
    const k4a_calibration_model_type_t type = camera->intrinsics.type;
    const bool model_ok = type == K4A_CALIBRATION_LENS_DISTORTION_MODEL_BROWN_CONRADY ||
                          type == K4A_CALIBRATION_LENS_DISTORTION_MODEL_RATIONAL_6KT;
    if (!model_ok || camera->intrinsics.parameter_count < 14)
    {
        openk4a_log(OPENK4A_LOG_ERROR, "the camera's distortion model %d with %u parameters is not readable",
                (int)type, camera->intrinsics.parameter_count);
        return false;
    }
    return true;
}

/* The Brown-Conrady model with the Jacobian, which the unprojecting step
 * needs and which is the SDK's own arithmetic. */
static bool project_internal(const k4a_calibration_camera_t *camera,
                             const float xy[2],
                             float uv[2],
                             int *valid,
                             float jacobian[4])
{
    if (!intrinsics_valid(camera))
    {
        return false;
    }

    const k4a_calibration_intrinsic_parameters_t *params = &camera->intrinsics.parameters;
    const float cx = params->param.cx;
    const float cy = params->param.cy;
    const float fx = params->param.fx;
    const float fy = params->param.fy;
    const float k1 = params->param.k1;
    const float k2 = params->param.k2;
    const float k3 = params->param.k3;
    const float k4 = params->param.k4;
    const float k5 = params->param.k5;
    const float k6 = params->param.k6;
    const float codx = params->param.codx;
    const float cody = params->param.cody;
    const float p1 = params->param.p1;
    const float p2 = params->param.p2;
    const float max_radius = camera->metric_radius;

    if (!(fx > 0.0f && fy > 0.0f))
    {
        openk4a_log(OPENK4A_LOG_ERROR, "fx %f and fy %f are not both positive", (double)fx, (double)fy);
        return false;
    }

    *valid = 1;
    const float xp = xy[0] - codx;
    const float yp = xy[1] - cody;
    const float xp2 = xp * xp;
    const float yp2 = yp * yp;
    const float xyp = xp * yp;
    const float rs = xp2 + yp2;
    if (rs > max_radius * max_radius)
    {
        *valid = 0;
        return true;
    }
    const float rss = rs * rs;
    const float rsc = rss * rs;
    const float a = 1.0f + k1 * rs + k2 * rss + k3 * rsc;
    const float b = 1.0f + k4 * rs + k5 * rss + k6 * rsc;
    const float bi = b != 0.0f ? 1.0f / b : 1.0f;
    const float d = a * bi;

    float xp_d = xp * d;
    float yp_d = yp * d;
    const float rs_2xp2 = rs + 2.0f * xp2;
    const float rs_2yp2 = rs + 2.0f * yp2;
    const bool rational = camera->intrinsics.type == K4A_CALIBRATION_LENS_DISTORTION_MODEL_RATIONAL_6KT;
    const float tangential = rational ? 1.0f : 2.0f;
    xp_d += rs_2xp2 * p2 + tangential * xyp * p1;
    yp_d += rs_2yp2 * p1 + tangential * xyp * p2;

    const float xp_d_cx = xp_d + codx;
    const float yp_d_cy = yp_d + cody;
    uv[0] = xp_d_cx * fx + cx;
    uv[1] = yp_d_cy * fy + cy;

    if (jacobian == NULL)
    {
        return true;
    }

    const float dudrs = k1 + 2.0f * k2 * rs + 3.0f * k3 * rss;
    const float dvdrs = k4 + 2.0f * k5 * rs + 3.0f * k6 * rss;
    const float bis = bi * bi;
    const float dddrs = (dudrs * b - a * dvdrs) * bis;
    const float dddrs_2 = dddrs * 2.0f;
    const float xp_dddrs_2 = xp * dddrs_2;
    const float yp_xp_dddrs_2 = yp * xp_dddrs_2;
    jacobian[0] = fx * (d + xp * xp_dddrs_2 + 6.0f * xp * p2 + tangential * yp * p1);
    jacobian[1] = fx * (yp_xp_dddrs_2 + 2.0f * yp * p2 + tangential * xp * p1);
    jacobian[2] = fy * (yp_xp_dddrs_2 + 2.0f * xp * p1 + tangential * yp * p2);
    jacobian[3] = fy * (d + yp * yp * dddrs_2 + 6.0f * yp * p1 + tangential * xp * p2);
    return true;
}

/* Newton's method for the inverse, which is what the SDK does too: the model
 * has no closed-form inverse. */
static bool unproject_internal(const k4a_calibration_camera_t *camera, const float uv[2], float xy[2], int *valid)
{
    if (!intrinsics_valid(camera))
    {
        return false;
    }

    const k4a_calibration_intrinsic_parameters_t *params = &camera->intrinsics.parameters;
    const float cx = params->param.cx;
    const float cy = params->param.cy;
    const float fx = params->param.fx;
    const float fy = params->param.fy;
    const float k1 = params->param.k1;
    const float k2 = params->param.k2;
    const float k3 = params->param.k3;
    const float k4 = params->param.k4;
    const float k5 = params->param.k5;
    const float k6 = params->param.k6;
    const float codx = params->param.codx;
    const float cody = params->param.cody;
    const float p1 = params->param.p1;
    const float p2 = params->param.p2;

    if (!(fx > 0.0f && fy > 0.0f))
    {
        openk4a_log(OPENK4A_LOG_ERROR, "fx %f and fy %f are not both positive", (double)fx, (double)fy);
        return false;
    }

    const float xp_d = (uv[0] - cx) / fx - codx;
    const float yp_d = (uv[1] - cy) / fy - cody;
    const float rs = xp_d * xp_d + yp_d * yp_d;
    const float rss = rs * rs;
    const float rsc = rss * rs;
    const float a = 1.0f + k1 * rs + k2 * rss + k3 * rsc;
    const float b = 1.0f + k4 * rs + k5 * rss + k6 * rsc;
    const float ai = a != 0.0f ? 1.0f / a : 1.0f;
    const float di = ai * b;
    xy[0] = xp_d * di;
    xy[1] = yp_d * di;

    /* First-order correction for the tangential terms. */
    const float two_xy = 2.0f * xy[0] * xy[1];
    const float xx = xy[0] * xy[0];
    const float yy = xy[1] * xy[1];
    xy[0] -= (yy + 3.0f * xx) * p2 + two_xy * p1;
    xy[1] -= (xx + 3.0f * yy) * p1 + two_xy * p2;
    xy[0] += codx;
    xy[1] += cody;

    *valid = 1;
    float best_xy[2] = { 0.0f, 0.0f };
    float best_err = OPENK4A_DISTANT;
    for (int pass = 0; pass < 20; pass++)
    {
        float projected[2];
        float jacobian[4];
        if (!project_internal(camera, xy, projected, valid, jacobian))
        {
            return false;
        }
        if (*valid == 0)
        {
            return true;
        }
        const float err_x = uv[0] - projected[0];
        const float err_y = uv[1] - projected[1];
        const float err = err_x * err_x + err_y * err_y;
        if (err >= best_err)
        {
            xy[0] = best_xy[0];
            xy[1] = best_xy[1];
            break;
        }
        best_err = err;
        best_xy[0] = xy[0];
        best_xy[1] = xy[1];
        if (pass == 19 || best_err < 1e-22f)
        {
            break;
        }

        const float determinant = jacobian[0] * jacobian[3] - jacobian[1] * jacobian[2];
        const float inverse = 1.0f / determinant;
        const float inverse_jacobian[4] = { inverse * jacobian[3], -inverse * jacobian[1],
                                            -inverse * jacobian[2], inverse * jacobian[0] };
        xy[0] += inverse_jacobian[0] * err_x + inverse_jacobian[1] * err_y;
        xy[1] += inverse_jacobian[2] * err_x + inverse_jacobian[3] * err_y;
    }

    if (best_err > 1e-6f)
    {
        *valid = 0;
    }
    return true;
}

bool openk4a_calibration_project(const k4a_calibration_camera_t *camera, const float point3d[3], float point2d[2],
                             int *valid)
{
    if (point3d[2] <= 0.0f)
    {
        point2d[0] = 0.0f;
        point2d[1] = 0.0f;
        *valid = 0;
        return true;
    }
    const float xy[2] = { point3d[0] / point3d[2], point3d[1] / point3d[2] };
    return project_internal(camera, xy, point2d, valid, NULL);
}

bool openk4a_calibration_unproject(const k4a_calibration_camera_t *camera,
                               const float point2d[2],
                               float depth,
                               float point3d[3],
                               int *valid)
{
    if (depth == 0.0f)
    {
        point3d[0] = point3d[1] = point3d[2] = 0.0f;
        *valid = 0;
        return true;
    }
    if (!unproject_internal(camera, point2d, point3d, valid))
    {
        return false;
    }
    point3d[0] *= depth;
    point3d[1] *= depth;
    point3d[2] = depth;
    return true;
}

static bool point_camera_ok(const k4a_calibration_t *calibration, k4a_calibration_type_t camera)
{
    if (camera < 0 || camera >= K4A_CALIBRATION_TYPE_NUM)
    {
        return false;
    }
    if (camera == K4A_CALIBRATION_TYPE_DEPTH && calibration->depth_mode == K4A_DEPTH_MODE_OFF)
    {
        return false;
    }
    if (camera == K4A_CALIBRATION_TYPE_COLOR && calibration->color_resolution == K4A_COLOR_RESOLUTION_OFF)
    {
        return false;
    }
    return true;
}

static bool point3d_to_3d(const k4a_calibration_t *calibration,
                          const float source[3],
                          k4a_calibration_type_t source_camera,
                          k4a_calibration_type_t target_camera,
                          float target[3])
{
    if (!point_camera_ok(calibration, source_camera) || !point_camera_ok(calibration, target_camera))
    {
        return false;
    }
    const k4a_calibration_extrinsics_t *extrinsics = &calibration->extrinsics[source_camera][target_camera];
    for (int row = 0; row < 3; row++)
    {
        target[row] = extrinsics->rotation[row * 3] * source[0] + extrinsics->rotation[row * 3 + 1] * source[1] +
                      extrinsics->rotation[row * 3 + 2] * source[2] + extrinsics->translation[row];
    }
    return true;
}

openk4a_result_t openk4a_calibration_math_3d_to_3d(const k4a_calibration_t *calibration,
                                           const float source_point3d_mm[3],
                                           k4a_calibration_type_t source_camera,
                                           k4a_calibration_type_t target_camera,
                                           float target_point3d_mm[3])
{
    return point3d_to_3d(calibration, source_point3d_mm, source_camera, target_camera, target_point3d_mm) ? OPENK4A_OK
                                                                                                       : OPENK4A_FAILED;
}

openk4a_result_t openk4a_calibration_math_2d_to_3d(const k4a_calibration_t *calibration,
                                           const float source_point2d[2],
                                           float source_depth_mm,
                                           k4a_calibration_type_t source_camera,
                                           k4a_calibration_type_t target_camera,
                                           float target_point3d_mm[3],
                                           int *valid)
{
    if (!point_camera_ok(calibration, source_camera) || !point_camera_ok(calibration, target_camera))
    {
        return OPENK4A_FAILED;
    }
    const k4a_calibration_camera_t *camera = source_camera == K4A_CALIBRATION_TYPE_DEPTH
                                                ? &calibration->depth_camera_calibration
                                                : &calibration->color_camera_calibration;
    float point3d[3];
    if (!openk4a_calibration_unproject(camera, source_point2d, source_depth_mm, point3d, valid))
    {
        return OPENK4A_FAILED;
    }
    if (*valid == 0)
    {
        target_point3d_mm[0] = target_point3d_mm[1] = target_point3d_mm[2] = 0.0f;
        return OPENK4A_OK;
    }
    return point3d_to_3d(calibration, point3d, source_camera, target_camera, target_point3d_mm) ? OPENK4A_OK : OPENK4A_FAILED;
}

openk4a_result_t openk4a_calibration_math_3d_to_2d(const k4a_calibration_t *calibration,
                                           const float source_point3d_mm[3],
                                           k4a_calibration_type_t source_camera,
                                           k4a_calibration_type_t target_camera,
                                           float target_point2d[2],
                                           int *valid)
{
    if (!point_camera_ok(calibration, source_camera) || !point_camera_ok(calibration, target_camera))
    {
        return OPENK4A_FAILED;
    }
    float point3d[3];
    if (!point3d_to_3d(calibration, source_point3d_mm, source_camera, target_camera, point3d))
    {
        return OPENK4A_FAILED;
    }
    const k4a_calibration_camera_t *camera = target_camera == K4A_CALIBRATION_TYPE_DEPTH
                                                ? &calibration->depth_camera_calibration
                                                : &calibration->color_camera_calibration;
    return openk4a_calibration_project(camera, point3d, target_point2d, valid) ? OPENK4A_OK : OPENK4A_FAILED;
}

openk4a_result_t openk4a_calibration_math_2d_to_2d(const k4a_calibration_t *calibration,
                                           const float source_point2d[2],
                                           float source_depth_mm,
                                           k4a_calibration_type_t source_camera,
                                           k4a_calibration_type_t target_camera,
                                           float target_point2d[2],
                                           int *valid)
{
    float point3d[3];
    openk4a_result_t result = openk4a_calibration_math_2d_to_3d(calibration,
                                                        source_point2d,
                                                        source_depth_mm,
                                                        source_camera,
                                                        target_camera,
                                                        point3d,
                                                        valid);
    if (result != OPENK4A_OK)
    {
        return result;
    }
    if (*valid == 0)
    {
        target_point2d[0] = target_point2d[1] = 0.0f;
        return OPENK4A_OK;
    }
    const k4a_calibration_camera_t *camera = target_camera == K4A_CALIBRATION_TYPE_DEPTH
                                                ? &calibration->depth_camera_calibration
                                                : &calibration->color_camera_calibration;
    return openk4a_calibration_project(camera, point3d, target_point2d, valid) ? OPENK4A_OK : OPENK4A_FAILED;
}

/*=============================================================================
  The public face
=============================================================================*/

k4a_result_t k4a_calibration_get_from_raw(char *raw_calibration,
                                          size_t raw_calibration_size,
                                          const k4a_depth_mode_t depth_mode,
                                          const k4a_color_resolution_t color_resolution,
                                          k4a_calibration_t *calibration)
{
    if (raw_calibration == NULL || calibration == NULL ||
        !(strnlen(raw_calibration, raw_calibration_size) < raw_calibration_size))
    {
        return K4A_RESULT_FAILED;
    }

    openk4a_calibration_t parsed;
    if (!openk4a_calibration_parse(raw_calibration, raw_calibration_size, &parsed))
    {
        return K4A_RESULT_FAILED;
    }
    const bool built = openk4a_calibration_build(&parsed, depth_mode, color_resolution, calibration);
    openk4a_calibration_free(&parsed);
    return built ? K4A_RESULT_SUCCEEDED : K4A_RESULT_FAILED;
}

k4a_result_t k4a_calibration_3d_to_3d(const k4a_calibration_t *calibration,
                                      const k4a_float3_t *source_point3d_mm,
                                      const k4a_calibration_type_t source_camera,
                                      const k4a_calibration_type_t target_camera,
                                      k4a_float3_t *target_point3d_mm)
{
    if (calibration == NULL || source_point3d_mm == NULL || target_point3d_mm == NULL)
    {
        return K4A_RESULT_FAILED;
    }
    return openk4a_calibration_math_3d_to_3d(calibration,
                                         source_point3d_mm->v,
                                         source_camera,
                                         target_camera,
                                         target_point3d_mm->v) == OPENK4A_OK
               ? K4A_RESULT_SUCCEEDED
               : K4A_RESULT_FAILED;
}

k4a_result_t k4a_calibration_2d_to_3d(const k4a_calibration_t *calibration,
                                      const k4a_float2_t *source_point2d,
                                      const float source_depth_mm,
                                      const k4a_calibration_type_t source_camera,
                                      const k4a_calibration_type_t target_camera,
                                      k4a_float3_t *target_point3d_mm,
                                      int *valid)
{
    if (calibration == NULL || source_point2d == NULL || target_point3d_mm == NULL || valid == NULL)
    {
        return K4A_RESULT_FAILED;
    }
    return openk4a_calibration_math_2d_to_3d(calibration,
                                         source_point2d->v,
                                         source_depth_mm,
                                         source_camera,
                                         target_camera,
                                         target_point3d_mm->v,
                                         valid) == OPENK4A_OK
               ? K4A_RESULT_SUCCEEDED
               : K4A_RESULT_FAILED;
}

k4a_result_t k4a_calibration_3d_to_2d(const k4a_calibration_t *calibration,
                                      const k4a_float3_t *source_point3d_mm,
                                      const k4a_calibration_type_t source_camera,
                                      const k4a_calibration_type_t target_camera,
                                      k4a_float2_t *target_point2d,
                                      int *valid)
{
    if (calibration == NULL || source_point3d_mm == NULL || target_point2d == NULL || valid == NULL)
    {
        return K4A_RESULT_FAILED;
    }
    return openk4a_calibration_math_3d_to_2d(calibration,
                                         source_point3d_mm->v,
                                         source_camera,
                                         target_camera,
                                         target_point2d->v,
                                         valid) == OPENK4A_OK
               ? K4A_RESULT_SUCCEEDED
               : K4A_RESULT_FAILED;
}

k4a_result_t k4a_calibration_2d_to_2d(const k4a_calibration_t *calibration,
                                      const k4a_float2_t *source_point2d,
                                      const float source_depth_mm,
                                      const k4a_calibration_type_t source_camera,
                                      const k4a_calibration_type_t target_camera,
                                      k4a_float2_t *target_point2d,
                                      int *valid)
{
    if (calibration == NULL || source_point2d == NULL || target_point2d == NULL || valid == NULL)
    {
        return K4A_RESULT_FAILED;
    }
    return openk4a_calibration_math_2d_to_2d(calibration,
                                         source_point2d->v,
                                         source_depth_mm,
                                         source_camera,
                                         target_camera,
                                         target_point2d->v,
                                         valid) == OPENK4A_OK
               ? K4A_RESULT_SUCCEEDED
               : K4A_RESULT_FAILED;
}
