/*=============================================================================
  The transformation engine.

  Two cameras, one scene: the depth camera's pixels are not the colour
  camera's pixels, and putting them together means moving points between the
  two coordinate systems through the device's own calibration. The engine
  does that three ways - depth into the colour camera's geometry, colour into
  the depth camera's, and depth into a point cloud - and each is built on the
  same two operations, project and unproject.

  Depth into colour is not a per-pixel map: a depth pixel lands *between*
  four colour pixels, and the nearest one leaves holes wherever the colour
  camera has more pixels than the depth camera has. So the pixels are walked
  a quad at a time - the four corners of a 2x2 block of depth - and each quad
  is rasterised into the colour image by interpolating over its two
  triangles. That is the SDK's own approach, and the reason its transformed
  depth image has no holes: it is drawn, not sampled.

  Three details of that drawing matter and are easy to get wrong. A quad with
  one corner missing is completed by interpolation rather than dropped, so an
  edge does not leave a stripe. A quad whose corners disagree in depth by
  more than a slanted surface could - the SDK's own 0.04693441759 of the
  nearest corner - is not drawn at all, because interpolating across a depth
  discontinuity invents a surface that is not there. And where two quads
  cover one colour pixel, the nearer depth wins, which is what keeps a
  foreground object in front of the wall behind it.
=============================================================================*/

#include "openk4a.h"

#include <math.h>

struct openk4a_transformation
{
    k4a_calibration_t calibration;
    /* The depth camera's own rays: for each depth pixel, x/z and y/z, which
     * is what a depth value is multiplied by to get a point. Built once. */
    float *x_table;
    float *y_table;
    uint8_t *ray_valid;

    float *vertex_u;   /* the colour pixel each depth pixel lands on */
    float *vertex_v;
    float *vertex_z;   /* and its depth in the colour camera's own frame */
    uint8_t *vertex_valid;
    bool have_tables;
};

/* A colour pixel and the depth that belongs to it, while a quad is drawn. */
typedef struct
{
    float u;
    float v;
    float depth;
    bool valid;
} openk4a_vertex_t;

/*=============================================================================
  Building the tables
=============================================================================*/

openk4a_transformation_t *openk4a_transformation_create(const k4a_calibration_t *calibration)
{
    if (calibration == NULL)
    {
        return NULL;
    }
    openk4a_transformation_t *transformation = (openk4a_transformation_t *)openk4a_alloc_zero(sizeof(*transformation));
    if (transformation == NULL)
    {
        return NULL;
    }
    transformation->calibration = *calibration;
    return transformation;
}

void openk4a_transformation_destroy(openk4a_transformation_t *transformation)
{
    if (transformation == NULL)
    {
        return;
    }
    openk4a_free(transformation->x_table);
    openk4a_free(transformation->y_table);
    openk4a_free(transformation->ray_valid);
    openk4a_free(transformation->vertex_u);
    openk4a_free(transformation->vertex_v);
    openk4a_free(transformation->vertex_z);
    openk4a_free(transformation->vertex_valid);
    openk4a_free(transformation);
}

/* One pass over the depth grid, once: the ray each pixel's depth is measured
 * along. Without it every frame would unproject every pixel twice. */
static bool transformation_build_rays(openk4a_transformation_t *transformation, int width, int height)
{
    if (transformation->have_tables)
    {
        return true;
    }
    const size_t pixels = (size_t)width * (size_t)height;
    transformation->x_table = (float *)openk4a_alloc_zero(pixels * sizeof(float));
    transformation->y_table = (float *)openk4a_alloc_zero(pixels * sizeof(float));
    transformation->ray_valid = (uint8_t *)openk4a_alloc_zero(pixels);
    transformation->vertex_u = (float *)openk4a_alloc_zero(pixels * sizeof(float));
    transformation->vertex_v = (float *)openk4a_alloc_zero(pixels * sizeof(float));
    transformation->vertex_z = (float *)openk4a_alloc_zero(pixels * sizeof(float));
    transformation->vertex_valid = (uint8_t *)openk4a_alloc_zero(pixels);
    if (transformation->x_table == NULL || transformation->y_table == NULL || transformation->ray_valid == NULL ||
        transformation->vertex_u == NULL || transformation->vertex_v == NULL || transformation->vertex_z == NULL ||
        transformation->vertex_valid == NULL)
    {
        return false;
    }

    const k4a_calibration_camera_t *depth = &transformation->calibration.depth_camera_calibration;
    for (int row = 0; row < height; row++)
    {
        for (int column = 0; column < width; column++)
        {
            const size_t pixel = (size_t)row * (size_t)width + (size_t)column;
            const float point2d[2] = { (float)column, (float)row };
            float point3d[3];
            int valid = 0;
            if (!openk4a_calibration_unproject(depth, point2d, 1.0f, point3d, &valid) || valid == 0 || point3d[2] <= 0.0f)
            {
                transformation->x_table[pixel] = NAN;
                transformation->y_table[pixel] = NAN;
                continue;
            }
            transformation->x_table[pixel] = point3d[0] / point3d[2];
            transformation->y_table[pixel] = point3d[1] / point3d[2];
            transformation->ray_valid[pixel] = 1;
        }
    }
    transformation->have_tables = true;
    return true;
}

/*=============================================================================
  Depth into the colour camera
=============================================================================*/

static openk4a_vertex_t transformation_vertex(const openk4a_transformation_t *transformation, size_t pixel, uint16_t depth)
{
    openk4a_vertex_t vertex;
    memset(&vertex, 0, sizeof(vertex));
    if (depth == 0 || !transformation->ray_valid[pixel])
    {
        return vertex;
    }

    const float point3d[3] = { transformation->x_table[pixel] * (float)depth,
                               transformation->y_table[pixel] * (float)depth,
                               (float)depth };
    float color_point[3];
    if (openk4a_calibration_math_3d_to_3d(&transformation->calibration,
                                      point3d,
                                      K4A_CALIBRATION_TYPE_DEPTH,
                                      K4A_CALIBRATION_TYPE_COLOR,
                                      color_point) != OPENK4A_OK)
    {
        return vertex;
    }
    vertex.depth = color_point[2];
    float color_point2d[2];
    int valid = 0;
    if (!openk4a_calibration_project(&transformation->calibration.color_camera_calibration,
                                 color_point,
                                 color_point2d,
                                 &valid))
    {
        return vertex;
    }
    if (valid == 0)
    {
        return vertex;
    }
    vertex.u = color_point2d[0];
    vertex.v = color_point2d[1];
    vertex.valid = true;
    return vertex;
}

/* The custom image beside the depth: 8-bit or 16-bit, and read the same way
 * either side of the pair it is drawn with. */
static float custom_at(const uint8_t *image8, const uint16_t *image16, int width, int row, int column)
{
    return image8 != NULL ? (float)image8[(size_t)row * (size_t)width + (size_t)column]
                          : (float)image16[(size_t)row * (size_t)width + (size_t)column];
}

/* Two triangles, barycentric coordinates, and the near depth wins. */
static void draw_quad(const openk4a_vertex_t *top_left,
                      const openk4a_vertex_t *top_right,
                      const openk4a_vertex_t *bottom_right,
                      const openk4a_vertex_t *bottom_left,
                      float custom_top_left,
                      float custom_top_right,
                      float custom_bottom_right,
                      float custom_bottom_left,
                      bool have_custom,
                      uint16_t *depth_out,
                      uint8_t *custom8_out,
                      uint16_t *custom16_out,
                      int out_width,
                      int out_height)
{
    const openk4a_vertex_t *const triangle[2][3] = { { top_left, top_right, bottom_right },
                                                 { top_left, bottom_right, bottom_left } };
    const float custom[2][3] = { { custom_top_left, custom_top_right, custom_bottom_right },
                                 { custom_top_left, custom_bottom_right, custom_bottom_left } };

    float min_u = top_left->u;
    float max_u = top_left->u;
    float min_v = top_left->v;
    float max_v = top_left->v;
    const openk4a_vertex_t *const corners[4] = { top_left, top_right, bottom_right, bottom_left };
    for (int i = 0; i < 4; i++)
    {
        if (corners[i]->u < min_u)
        {
            min_u = corners[i]->u;
        }
        if (corners[i]->u > max_u)
        {
            max_u = corners[i]->u;
        }
        if (corners[i]->v < min_v)
        {
            min_v = corners[i]->v;
        }
        if (corners[i]->v > max_v)
        {
            max_v = corners[i]->v;
        }
    }

    int first_row = (int)floorf(min_v);
    int last_row = (int)ceilf(max_v);
    int first_column = (int)floorf(min_u);
    int last_column = (int)ceilf(max_u);
    if (first_row < 0)
    {
        first_row = 0;
    }
    if (first_column < 0)
    {
        first_column = 0;
    }
    if (last_row > out_height - 1)
    {
        last_row = out_height - 1;
    }
    if (last_column > out_width - 1)
    {
        last_column = out_width - 1;
    }

    for (int row = first_row; row <= last_row; row++)
    {
        for (int column = first_column; column <= last_column; column++)
        {
            bool inside = false;
            float depth = 0.0f;
            float custom_value = 0.0f;
            for (int t = 0; t < 2 && !inside; t++)
            {
                const openk4a_vertex_t *a = triangle[t][0];
                const openk4a_vertex_t *b = triangle[t][1];
                const openk4a_vertex_t *c = triangle[t][2];
                const float area = (b->u - a->u) * (c->v - a->v) - (c->u - a->u) * (b->v - a->v);
                if (area == 0.0f)
                {
                    continue;
                }
                const float point_u = (float)column;
                const float point_v = (float)row;
                float w0 = ((b->u - a->u) * (point_v - a->v) - (point_u - a->u) * (b->v - a->v)) / area;
                float w1 = ((point_u - a->u) * (c->v - a->v) - (c->u - a->u) * (point_v - a->v)) / area;
                float w2 = 1.0f - w0 - w1;
                if (w0 < -1e-4f || w1 < -1e-4f || w2 < -1e-4f)
                {
                    continue;
                }
                inside = true;
                depth = w2 * a->depth + w1 * b->depth + w0 * c->depth;
                if (have_custom)
                {
                    custom_value = w2 * custom[t][0] + w1 * custom[t][1] + w0 * custom[t][2];
                }
            }
            if (!inside)
            {
                continue;
            }

            /* Occlusion: the surface nearest the colour camera wins. */
            const size_t pixel = (size_t)row * (size_t)out_width + (size_t)column;
            const uint16_t value = (uint16_t)(depth + 0.5f);
            if (depth_out[pixel] == 0 || value < depth_out[pixel])
            {
                depth_out[pixel] = value;
                if (have_custom && custom8_out != NULL)
                {
                    custom8_out[pixel] = (uint8_t)(custom_value + 0.5f);
                }
                else if (have_custom && custom16_out != NULL)
                {
                    custom16_out[pixel] = (uint16_t)(custom_value + 0.5f);
                }
            }
        }
    }
}

/* The SDK's own four corners, with a missing one filled in so that an edge
 * does not leave a stripe. */
static bool fix_corners(openk4a_vertex_t *top_left,
                        openk4a_vertex_t *top_right,
                        openk4a_vertex_t *bottom_right,
                        openk4a_vertex_t *bottom_left,
                        float *custom_top_left,
                        float *custom_top_right,
                        float *custom_bottom_right,
                        float *custom_bottom_left,
                        bool have_custom)
{
    int invalid = 0;
    if (!top_left->valid)
    {
        invalid++;
        top_left->u = (top_right->u + bottom_left->u) * 0.5f;
        top_left->v = (top_right->v + bottom_left->v) * 0.5f;
        top_left->depth = (top_right->depth + bottom_left->depth) * 0.5f;
        if (have_custom)
        {
            *custom_top_left = (*custom_top_right + *custom_bottom_left) * 0.5f;
        }
    }
    if (!top_right->valid)
    {
        invalid++;
        *top_right = *bottom_right;
        bottom_right->u = (bottom_right->u + bottom_left->u) * 0.5f;
        bottom_right->v = (bottom_right->v + bottom_left->v) * 0.5f;
        bottom_right->depth = (bottom_right->depth + bottom_left->depth) * 0.5f;
        if (have_custom)
        {
            *custom_top_right = *custom_bottom_right;
            *custom_bottom_right = (*custom_bottom_right + *custom_bottom_left) * 0.5f;
        }
    }
    if (!bottom_right->valid)
    {
        invalid++;
        bottom_right->u = (top_right->u + bottom_left->u) * 0.5f;
        bottom_right->v = (top_right->v + bottom_left->v) * 0.5f;
        bottom_right->depth = (top_right->depth + bottom_left->depth) * 0.5f;
        if (have_custom)
        {
            *custom_bottom_right = (*custom_top_right + *custom_bottom_left) * 0.5f;
        }
    }
    if (!bottom_left->valid)
    {
        invalid++;
        *bottom_left = *bottom_right;
        bottom_right->u = (top_right->u + bottom_right->u) * 0.5f;
        bottom_right->v = (top_right->v + bottom_right->v) * 0.5f;
        bottom_right->depth = (top_right->depth + bottom_right->depth) * 0.5f;
        if (have_custom)
        {
            *custom_bottom_left = *custom_bottom_right;
            *custom_bottom_right = (*custom_top_right + *custom_bottom_right) * 0.5f;
        }
    }
    return invalid < 2;
}

/* Interpolating across a depth discontinuity invents a surface that is not
 * there, so a quad whose corners disagree by more than a slanted surface
 * could is not drawn. The ratio is the SDK's own. */
#define OPENK4A_SKIP_INTERPOLATION_RATIO 0.04693441759f

static openk4a_result_t transformation_depth_to_color(openk4a_transformation_t *transformation,
                                                 k4a_image_t depth_image,
                                                 k4a_image_t transformed_depth_image,
                                                 k4a_image_t custom_image,
                                                 k4a_image_t transformed_custom_image,
                                                 uint16_t invalid_custom_value)
{
    const int depth_width = k4a_image_get_width_pixels(depth_image);
    const int depth_height = k4a_image_get_height_pixels(depth_image);
    const int color_width = k4a_image_get_width_pixels(transformed_depth_image);
    const int color_height = k4a_image_get_height_pixels(transformed_depth_image);
    if (depth_width <= 0 || depth_height <= 0 || color_width <= 0 || color_height <= 0)
    {
        return OPENK4A_FAILED;
    }
    if (!transformation_build_rays(transformation, depth_width, depth_height))
    {
        return OPENK4A_FAILED;
    }

    const uint16_t *depth = (const uint16_t *)k4a_image_get_buffer(depth_image);
    uint16_t *out_depth = (uint16_t *)k4a_image_get_buffer(transformed_depth_image);
    memset(out_depth, 0, (size_t)k4a_image_get_stride_bytes(transformed_depth_image) * (size_t)color_height);

    const size_t out_pixels = (size_t)color_width * (size_t)color_height;
    const bool have_custom = custom_image != NULL && transformed_custom_image != NULL;
    if (have_custom)
    {
        const k4a_image_format_t format = k4a_image_get_format(transformed_custom_image);
        if (format == K4A_IMAGE_FORMAT_CUSTOM8)
        {
            memset(k4a_image_get_buffer(transformed_custom_image), (int)(invalid_custom_value & 0xFF), out_pixels);
        }
        else
        {
            uint16_t *values = (uint16_t *)k4a_image_get_buffer(transformed_custom_image);
            for (size_t i = 0; i < out_pixels; i++)
            {
                values[i] = invalid_custom_value;
            }
        }
    }

    /* Every depth pixel's landing point, once. */
    const size_t depth_pixels = (size_t)depth_width * (size_t)depth_height;
    for (size_t pixel = 0; pixel < depth_pixels; pixel++)
    {
        const openk4a_vertex_t vertex = transformation_vertex(transformation, pixel, depth[pixel]);
        transformation->vertex_u[pixel] = vertex.u;
        transformation->vertex_v[pixel] = vertex.v;
        transformation->vertex_z[pixel] = vertex.depth;
        transformation->vertex_valid[pixel] = vertex.valid ? 1 : 0;
    }

    const k4a_image_format_t format = have_custom ? k4a_image_get_format(transformed_custom_image)
                                                  : K4A_IMAGE_FORMAT_CUSTOM;
    const uint8_t *custom8 = have_custom && format == K4A_IMAGE_FORMAT_CUSTOM8
                                 ? (const uint8_t *)k4a_image_get_buffer(custom_image)
                                 : NULL;
    const uint16_t *custom16 = have_custom && format != K4A_IMAGE_FORMAT_CUSTOM8
                                   ? (const uint16_t *)k4a_image_get_buffer(custom_image)
                                   : NULL;
    uint8_t *out8 = have_custom && format == K4A_IMAGE_FORMAT_CUSTOM8
                        ? (uint8_t *)k4a_image_get_buffer(transformed_custom_image)
                        : NULL;
    uint16_t *out16 = have_custom && format != K4A_IMAGE_FORMAT_CUSTOM8
                          ? (uint16_t *)k4a_image_get_buffer(transformed_custom_image)
                          : NULL;

    for (int row = 1; row < depth_height; row++)
    {
        for (int column = 1; column < depth_width; column++)
        {
            const size_t tl = (size_t)(row - 1) * (size_t)depth_width + (size_t)(column - 1);
            const size_t tr = (size_t)(row - 1) * (size_t)depth_width + (size_t)column;
            const size_t bl = (size_t)row * (size_t)depth_width + (size_t)(column - 1);
            const size_t br = (size_t)row * (size_t)depth_width + (size_t)column;

            openk4a_vertex_t top_left = { transformation->vertex_u[tl], transformation->vertex_v[tl],
                                      transformation->vertex_z[tl], transformation->vertex_valid[tl] != 0 };
            openk4a_vertex_t top_right = { transformation->vertex_u[tr], transformation->vertex_v[tr],
                                       transformation->vertex_z[tr], transformation->vertex_valid[tr] != 0 };
            openk4a_vertex_t bottom_right = { transformation->vertex_u[br], transformation->vertex_v[br],
                                          transformation->vertex_z[br], transformation->vertex_valid[br] != 0 };
            openk4a_vertex_t bottom_left = { transformation->vertex_u[bl], transformation->vertex_v[bl],
                                         transformation->vertex_z[bl], transformation->vertex_valid[bl] != 0 };

            float custom_tl = 0.0f;
            float custom_tr = 0.0f;
            float custom_br = 0.0f;
            float custom_bl = 0.0f;
            if (have_custom)
            {
                custom_tl = custom_at(custom8, custom16, depth_width, row - 1, column - 1);
                custom_tr = custom_at(custom8, custom16, depth_width, row - 1, column);
                custom_br = custom_at(custom8, custom16, depth_width, row, column);
                custom_bl = custom_at(custom8, custom16, depth_width, row, column - 1);
            }

            if (!fix_corners(&top_left, &top_right, &bottom_right, &bottom_left, &custom_tl, &custom_tr, &custom_br,
                             &custom_bl, have_custom))
            {
                continue;
            }

            float lowest = top_left.depth;
            float highest = top_left.depth;
            const openk4a_vertex_t *const corners[4] = { &top_right, &bottom_right, &bottom_left, &top_left };
            for (int i = 0; i < 4; i++)
            {
                if (corners[i]->depth < lowest)
                {
                    lowest = corners[i]->depth;
                }
                if (corners[i]->depth > highest)
                {
                    highest = corners[i]->depth;
                }
            }
            if (highest - lowest > OPENK4A_SKIP_INTERPOLATION_RATIO * lowest)
            {
                continue;
            }

            draw_quad(&top_left, &top_right, &bottom_right, &bottom_left, custom_tl, custom_tr, custom_br,
                      custom_bl, have_custom, out_depth, out8, out16, color_width, color_height);
        }
    }
    return OPENK4A_OK;
}

/*=============================================================================
  The public face
=============================================================================*/

openk4a_result_t openk4a_transformation_depth_to_color(openk4a_transformation_t *transformation,
                                               k4a_image_t depth_image,
                                               k4a_image_t transformed_depth_image)
{
    if (transformation == NULL || depth_image == NULL || transformed_depth_image == NULL)
    {
        return OPENK4A_FAILED;
    }
    return transformation_depth_to_color(transformation, depth_image, transformed_depth_image, NULL, NULL, 0);
}

k4a_transformation_t k4a_transformation_create(const k4a_calibration_t *calibration)
{
    return (k4a_transformation_t)openk4a_transformation_create(calibration);
}

void k4a_transformation_destroy(k4a_transformation_t transformation_handle)
{
    openk4a_transformation_destroy((openk4a_transformation_t *)transformation_handle);
}

k4a_result_t k4a_transformation_depth_image_to_color_camera(k4a_transformation_t transformation_handle,
                                                            const k4a_image_t depth_image,
                                                            k4a_image_t transformed_depth_image)
{
    return openk4a_transformation_depth_to_color((openk4a_transformation_t *)transformation_handle,
                                             depth_image,
                                             transformed_depth_image) == OPENK4A_OK
               ? K4A_RESULT_SUCCEEDED
               : K4A_RESULT_FAILED;
}

k4a_result_t k4a_transformation_depth_image_to_color_camera_custom(k4a_transformation_t transformation_handle,
                                                                  const k4a_image_t depth_image,
                                                                  const k4a_image_t custom_image,
                                                                  k4a_image_t transformed_depth_image,
                                                                  k4a_image_t transformed_custom_image,
                                                                  k4a_transformation_interpolation_type_t interpolation_type,
                                                                  uint32_t invalid_custom_value)
{
    if (custom_image == NULL || transformed_custom_image == NULL)
    {
        return K4A_RESULT_FAILED;
    }
    if (interpolation_type != K4A_TRANSFORMATION_INTERPOLATION_TYPE_NEAREST &&
        interpolation_type != K4A_TRANSFORMATION_INTERPOLATION_TYPE_LINEAR)
    {
        return K4A_RESULT_FAILED;
    }
    return transformation_depth_to_color((openk4a_transformation_t *)transformation_handle,
                                        depth_image,
                                        transformed_depth_image,
                                        custom_image,
                                        transformed_custom_image,
                                        (uint16_t)invalid_custom_value) == OPENK4A_OK
               ? K4A_RESULT_SUCCEEDED
               : K4A_RESULT_FAILED;
}

/*-----------------------------------------------------------------------------
  Colour into the depth camera: the other direction, and the easy one. Here
  every depth pixel asks for a colour value, so the colour image is sampled
  bilinearly - the one place in this file where interpolation is safe,
  because there is no depth involved.
---------------------------------------------------------------------------*/

static uint8_t bilinear(const uint8_t *image, int stride, float u, float v)
{
    const int x0 = (int)floorf(u);
    const int y0 = (int)floorf(v);
    const float fx = u - (float)x0;
    const float fy = v - (float)y0;
    const uint8_t *row0 = image + (size_t)y0 * (size_t)stride;
    const uint8_t *row1 = row0 + stride;
    const float top = (float)row0[x0] * (1.0f - fx) + (float)row0[x0 + 1] * fx;
    const float bottom = (float)row1[x0] * (1.0f - fx) + (float)row1[x0 + 1] * fx;
    return (uint8_t)(top * (1.0f - fy) + bottom * fy + 0.5f);
}

k4a_result_t k4a_transformation_color_image_to_depth_camera(k4a_transformation_t transformation_handle,
                                                            const k4a_image_t depth_image,
                                                            const k4a_image_t color_image,
                                                            k4a_image_t transformed_color_image)
{
    openk4a_transformation_t *transformation = (openk4a_transformation_t *)transformation_handle;
    if (transformation == NULL || depth_image == NULL || color_image == NULL || transformed_color_image == NULL)
    {
        return K4A_RESULT_FAILED;
    }
    if (k4a_image_get_format(color_image) != K4A_IMAGE_FORMAT_COLOR_BGRA32 &&
        k4a_image_get_format(color_image) != K4A_IMAGE_FORMAT_CUSTOM8)
    {
        openk4a_log(OPENK4A_LOG_ERROR, "the colour image must be BGRA32 or CUSTOM8 to be resampled");
        return K4A_RESULT_FAILED;
    }

    const int depth_width = k4a_image_get_width_pixels(depth_image);
    const int depth_height = k4a_image_get_height_pixels(depth_image);
    const int color_width = k4a_image_get_width_pixels(color_image);
    const int color_height = k4a_image_get_height_pixels(color_image);
    const int color_stride = k4a_image_get_stride_bytes(color_image);
    const int components = k4a_image_get_format(color_image) == K4A_IMAGE_FORMAT_COLOR_BGRA32 ? 4 : 1;
    const uint8_t *source = k4a_image_get_buffer(color_image);
    uint8_t *destination = k4a_image_get_buffer(transformed_color_image);
    if (source == NULL || destination == NULL || color_width < 2 || color_height < 2)
    {
        return K4A_RESULT_FAILED;
    }

    const uint16_t *depth = (const uint16_t *)k4a_image_get_buffer(depth_image);
    const int out_stride = k4a_image_get_stride_bytes(transformed_color_image);
    for (int row = 0; row < depth_height; row++)
    {
        for (int column = 0; column < depth_width; column++)
        {
            const uint16_t value = depth[(size_t)row * (size_t)depth_width + (size_t)column];
            uint8_t *out = destination + (size_t)row * (size_t)out_stride + (size_t)column * components;
            if (value == 0)
            {
                memset(out, 0, (size_t)components);
                continue;
            }

            float point2d[2] = { (float)column, (float)row };
            float color_point2d[2];
            int valid = 0;
            if (openk4a_calibration_math_2d_to_2d(&transformation->calibration,
                                              point2d,
                                              (float)value,
                                              K4A_CALIBRATION_TYPE_DEPTH,
                                              K4A_CALIBRATION_TYPE_COLOR,
                                              color_point2d,
                                              &valid) != OPENK4A_OK ||
                valid == 0 || color_point2d[0] < 0.0f || color_point2d[1] < 0.0f ||
                color_point2d[0] >= (float)(color_width - 1) || color_point2d[1] >= (float)(color_height - 1))
            {
                memset(out, 0, (size_t)components);
                continue;
            }

            for (int component = 0; component < components; component++)
            {
                out[component] = bilinear(source + component, color_stride, color_point2d[0], color_point2d[1]);
            }
            if (components == 4)
            {
                out[3] = 0xFF;
            }
        }
    }
    return K4A_RESULT_SUCCEEDED;
}

/*-----------------------------------------------------------------------------
  And the point cloud: every depth pixel becomes (X, Y, Z) in millimetres,
  which is its ray times its depth. Three planes of 16-bit values, X then Y
  then Z, each the size of the depth image.
---------------------------------------------------------------------------*/

/*-----------------------------------------------------------------------------
  A colour pixel back to a depth pixel.

  One depth pixel's colour is known; the question is which depth pixel a
  colour pixel came from, and the answer is a search. The colour camera's
  pixel defines a ray in the depth camera's frame, and the depth map says how
  far along it the surface is. The two are compared pixel by pixel along the
  epipolar line - the line the ray projects to - and the first depth pixel
  whose reprojection lands back on the colour pixel wins, which is the SDK's
  own method.
---------------------------------------------------------------------------*/

static bool depth_pinhole(const k4a_calibration_t *calibration, float *px, float *py, float *fx, float *fy)
{
    float fov[2];
    switch (calibration->depth_mode)
    {
    case K4A_DEPTH_MODE_NFOV_2X2BINNED:
    case K4A_DEPTH_MODE_NFOV_UNBINNED:
        fov[0] = 75.0f;
        fov[1] = 65.0f;
        break;
    case K4A_DEPTH_MODE_WFOV_2X2BINNED:
    case K4A_DEPTH_MODE_WFOV_UNBINNED:
    case K4A_DEPTH_MODE_PASSIVE_IR:
        fov[0] = 120.0f;
        fov[1] = 120.0f;
        break;
    default:
        return false;
    }
    const int width = calibration->depth_camera_calibration.resolution_width;
    const int height = calibration->depth_camera_calibration.resolution_height;
    const float radians_per_degree = 3.14159265f / 180.0f;
    *fx = (0.5f / tanf(0.5f * fov[0] * radians_per_degree)) * (float)width;
    *fy = (0.5f / tanf(0.5f * fov[1] * radians_per_degree)) * (float)height;
    *px = (float)width / 2.0f;
    *py = (float)height / 2.0f;
    return true;
}

openk4a_result_t openk4a_calibration_math_color_2d_to_depth_2d(const k4a_calibration_t *calibration,
                                                       const float source_point2d[2],
                                                       k4a_image_t depth_image,
                                                       float target_point2d[2],
                                                       int *valid)
{
    float px;
    float py;
    float fx;
    float fy;
    if (!depth_pinhole(calibration, &px, &py, &fx, &fy))
    {
        return OPENK4A_FAILED;
    }

    /* The two ends of the search: the nearest and the furthest the depth
     * sensor can report, taken all the way into the depth camera. */
    const float range[2] = { 50.0f, 14000.0f };
    float start[3];
    float stop[3];
    int start_valid = 0;
    int stop_valid = 0;
    if (openk4a_calibration_math_2d_to_3d(calibration,
                                      source_point2d,
                                      range[0],
                                      K4A_CALIBRATION_TYPE_COLOR,
                                      K4A_CALIBRATION_TYPE_DEPTH,
                                      start,
                                      &start_valid) != OPENK4A_OK ||
        openk4a_calibration_math_2d_to_3d(calibration,
                                      source_point2d,
                                      range[1],
                                      K4A_CALIBRATION_TYPE_COLOR,
                                      K4A_CALIBRATION_TYPE_DEPTH,
                                      stop,
                                      &stop_valid) != OPENK4A_OK)
    {
        return OPENK4A_FAILED;
    }
    *valid = start_valid && stop_valid;
    if (*valid == 0 || start[2] <= 0.0f || stop[2] <= 0.0f)
    {
        *valid = 0;
        return OPENK4A_OK;
    }

    float start2d[2] = { start[0] / start[2] * fx + px, start[1] / start[2] * fy + py };
    float stop2d[2] = { stop[0] / stop[2] * fx + px, stop[1] / stop[2] * fy + py };
    if (stop2d[0] == start2d[0])
    {
        return OPENK4A_FAILED;
    }
    const float slope = (stop2d[1] - start2d[1]) / (stop2d[0] - start2d[0]);
    const bool step_x = fabsf(slope) < 1.0f;
    const bool forward = step_x ? stop2d[0] > start2d[0] : stop2d[1] > start2d[1];

    const int depth_width = k4a_image_get_width_pixels(depth_image);
    const int depth_height = k4a_image_get_height_pixels(depth_image);
    const uint16_t *depth = (const uint16_t *)k4a_image_get_buffer(depth_image);
    float point[2] = { start2d[0], start2d[1] };
    float best_error = OPENK4A_DISTANT;
    float best_point[2] = { 0.0f, 0.0f };
    bool found = false;

    const float start_along = step_x ? start2d[0] : start2d[1];
    const float stop_along = step_x ? stop2d[0] : stop2d[1];
    const float direction = forward ? 1.0f : -1.0f;
    for (float along = start_along; forward ? along <= stop_along : along >= stop_along; along += direction)
    {
        point[0] = step_x ? along : start2d[0] + (along - start2d[1]) / slope;
        point[1] = step_x ? start2d[1] + (along - start2d[0]) * slope : along;

        /* The ray through this searching pixel, taken into the depth image's
         * own distorted coordinates. */
        const float ray[3] = { (point[0] - px) / fx, (point[1] - py) / fy, 1.0f };
        float depth_point2d[2];
        int ray_valid = 0;
        if (openk4a_calibration_project(&calibration->depth_camera_calibration, ray, depth_point2d, &ray_valid) &&
            ray_valid != 0)
        {
            const int u = (int)floorf(depth_point2d[0] + 0.5f);
            const int v = (int)floorf(depth_point2d[1] + 0.5f);
            if (u >= 0 && v >= 0 && u < depth_width && v < depth_height)
            {
                const uint16_t value = depth[(size_t)v * (size_t)depth_width + (size_t)u];
                if (value != 0)
                {
                    /* Where that depth pixel reprojects to in the colour
                     * image: the nearer to the pixel asked about, the better. */
                    float reprojected[2];
                    int reprojected_valid = 0;
                    if (openk4a_calibration_math_2d_to_2d(calibration,
                                                      depth_point2d,
                                                      (float)value,
                                                      K4A_CALIBRATION_TYPE_DEPTH,
                                                      K4A_CALIBRATION_TYPE_COLOR,
                                                      reprojected,
                                                      &reprojected_valid) == OPENK4A_OK &&
                        reprojected_valid != 0)
                    {
                        const float error = fabsf(reprojected[0] - source_point2d[0]) +
                                            fabsf(reprojected[1] - source_point2d[1]);
                        if (error < best_error)
                        {
                            best_error = error;
                            best_point[0] = (float)u;
                            best_point[1] = (float)v;
                            found = true;
                        }
                    }
                }
            }
        }
    }

    if (!found)
    {
        *valid = 0;
        return OPENK4A_OK;
    }
    target_point2d[0] = best_point[0];
    target_point2d[1] = best_point[1];
    return OPENK4A_OK;
}

k4a_result_t k4a_calibration_color_2d_to_depth_2d(const k4a_calibration_t *calibration,
                                                  const k4a_float2_t *source_point2d,
                                                  const k4a_image_t depth_image,
                                                  k4a_float2_t *target_point2d,
                                                  int *valid)
{
    if (calibration == NULL || source_point2d == NULL || depth_image == NULL || target_point2d == NULL ||
        valid == NULL)
    {
        return K4A_RESULT_FAILED;
    }
    return openk4a_calibration_math_color_2d_to_depth_2d(calibration,
                                                     source_point2d->v,
                                                     depth_image,
                                                     target_point2d->v,
                                                     valid) == OPENK4A_OK
               ? K4A_RESULT_SUCCEEDED
               : K4A_RESULT_FAILED;
}

k4a_result_t k4a_transformation_depth_image_to_point_cloud(k4a_transformation_t transformation_handle,
                                                           const k4a_image_t depth_image,
                                                           const k4a_calibration_type_t camera,
                                                           k4a_image_t xyz_image)
{
    openk4a_transformation_t *transformation = (openk4a_transformation_t *)transformation_handle;
    if (transformation == NULL || depth_image == NULL || xyz_image == NULL)
    {
        return K4A_RESULT_FAILED;
    }
    if (camera != K4A_CALIBRATION_TYPE_DEPTH && camera != K4A_CALIBRATION_TYPE_COLOR)
    {
        return K4A_RESULT_FAILED;
    }

    const int width = k4a_image_get_width_pixels(depth_image);
    const int height = k4a_image_get_height_pixels(depth_image);
    const size_t pixels = (size_t)width * (size_t)height;
    const int out_stride = k4a_image_get_stride_bytes(xyz_image);
    uint8_t *out = k4a_image_get_buffer(xyz_image);
    const uint16_t *depth = (const uint16_t *)k4a_image_get_buffer(depth_image);
    if (out == NULL || depth == NULL || out_stride < width * 2 * (int)sizeof(int16_t))
    {
        return K4A_RESULT_FAILED;
    }

    /* The three planes are one after another, each the image's own size. */
    int16_t *x_plane = (int16_t *)out;
    int16_t *y_plane = (int16_t *)(out + (size_t)out_stride * (size_t)height);
    int16_t *z_plane = (int16_t *)(out + 2u * (size_t)out_stride * (size_t)height);
    memset(out, 0, 3u * (size_t)out_stride * (size_t)height);

    const k4a_calibration_camera_t *depth_camera = &transformation->calibration.depth_camera_calibration;
    for (int row = 0; row < height; row++)
    {
        for (int column = 0; column < width; column++)
        {
            const size_t pixel = (size_t)row * (size_t)width + (size_t)column;
            const uint16_t value = depth[pixel];
            if (value == 0)
            {
                continue;
            }

            const float point2d[2] = { (float)column, (float)row };
            float point3d[3];
            int valid = 0;
            if (!openk4a_calibration_unproject(depth_camera, point2d, (float)value, point3d, &valid) || valid == 0)
            {
                continue;
            }
            if (camera == K4A_CALIBRATION_TYPE_COLOR)
            {
                if (openk4a_calibration_math_3d_to_3d(&transformation->calibration,
                                                  point3d,
                                                  K4A_CALIBRATION_TYPE_DEPTH,
                                                  K4A_CALIBRATION_TYPE_COLOR,
                                                  point3d) != OPENK4A_OK)
                {
                    continue;
                }
            }
            x_plane[pixel] = (int16_t)point3d[0];
            y_plane[pixel] = (int16_t)point3d[1];
            z_plane[pixel] = (int16_t)point3d[2];
        }
    }
    (void)pixels;
    return K4A_RESULT_SUCCEEDED;
}
