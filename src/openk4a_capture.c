/*=============================================================================
  Captures.

  A capture is one frame of each kind, together: the depth plane, the IR
  plane and the colour image that belong to the same moment, plus the
  device's own temperature reading for that frame. The images are referenced
  by the capture, so a program gets the capture, takes the images it wants,
  and releases the capture whenever it likes.
=============================================================================*/

#include "openk4a.h"

typedef struct
{
    int refs;
    float temperature_c;
    k4a_image_t color;
    k4a_image_t depth;
    k4a_image_t ir;
    k4a_image_t imu;
} openk4a_capture_t;

enum
{
    OPENK4A_CAPTURE_COLOR = 0,
    OPENK4A_CAPTURE_DEPTH,
    OPENK4A_CAPTURE_IR,
    OPENK4A_CAPTURE_IMU
};

static k4a_image_t *capture_slot(openk4a_capture_t *capture, int which)
{
    if (capture == NULL)
    {
        return NULL;
    }
    switch (which)
    {
    case OPENK4A_CAPTURE_COLOR:
        return &capture->color;
    case OPENK4A_CAPTURE_DEPTH:
        return &capture->depth;
    case OPENK4A_CAPTURE_IR:
        return &capture->ir;
    default:
        return &capture->imu;
    }
}

static void capture_set(k4a_capture_t capture_handle, int which, k4a_image_t image)
{
    k4a_image_t *slot = capture_slot((openk4a_capture_t *)capture_handle, which);
    if (slot == NULL)
    {
        return;
    }
    /* The capture takes its own reference, and drops whatever was there. */
    if (image != NULL)
    {
        openk4a_image_add_ref(image);
    }
    if (*slot != NULL)
    {
        openk4a_image_dec_ref(*slot);
    }
    *slot = image;
}

static k4a_image_t capture_get(k4a_capture_t capture_handle, int which)
{
    k4a_image_t *slot = capture_slot((openk4a_capture_t *)capture_handle, which);
    if (slot == NULL || *slot == NULL)
    {
        return NULL;
    }
    openk4a_image_add_ref(*slot);
    return *slot;
}

k4a_result_t k4a_capture_create(k4a_capture_t *capture_handle)
{
    if (capture_handle == NULL)
    {
        return K4A_RESULT_FAILED;
    }
    openk4a_capture_t *capture = (openk4a_capture_t *)openk4a_alloc_zero(sizeof(*capture));
    if (capture == NULL)
    {
        return K4A_RESULT_FAILED;
    }
    capture->refs = 1;
    *capture_handle = (k4a_capture_t)capture;
    return K4A_RESULT_SUCCEEDED;
}

void k4a_capture_release(k4a_capture_t capture_handle)
{
    openk4a_capture_t *capture = (openk4a_capture_t *)capture_handle;
    if (capture == NULL || --capture->refs > 0)
    {
        return;
    }
    if (capture->color != NULL)
    {
        openk4a_image_dec_ref(capture->color);
    }
    if (capture->depth != NULL)
    {
        openk4a_image_dec_ref(capture->depth);
    }
    if (capture->ir != NULL)
    {
        openk4a_image_dec_ref(capture->ir);
    }
    if (capture->imu != NULL)
    {
        openk4a_image_dec_ref(capture->imu);
    }
    openk4a_free(capture);
}

void k4a_capture_reference(k4a_capture_t capture_handle)
{
    openk4a_capture_t *capture = (openk4a_capture_t *)capture_handle;
    if (capture != NULL)
    {
        capture->refs++;
    }
}

k4a_image_t k4a_capture_get_color_image(k4a_capture_t capture_handle)
{
    return capture_get(capture_handle, OPENK4A_CAPTURE_COLOR);
}

k4a_image_t k4a_capture_get_depth_image(k4a_capture_t capture_handle)
{
    return capture_get(capture_handle, OPENK4A_CAPTURE_DEPTH);
}

k4a_image_t k4a_capture_get_ir_image(k4a_capture_t capture_handle)
{
    return capture_get(capture_handle, OPENK4A_CAPTURE_IR);
}

void k4a_capture_set_color_image(k4a_capture_t capture_handle, k4a_image_t image_handle)
{
    capture_set(capture_handle, OPENK4A_CAPTURE_COLOR, image_handle);
}

void k4a_capture_set_depth_image(k4a_capture_t capture_handle, k4a_image_t image_handle)
{
    capture_set(capture_handle, OPENK4A_CAPTURE_DEPTH, image_handle);
}

void k4a_capture_set_ir_image(k4a_capture_t capture_handle, k4a_image_t image_handle)
{
    capture_set(capture_handle, OPENK4A_CAPTURE_IR, image_handle);
}

void k4a_capture_set_temperature_c(k4a_capture_t capture_handle, float temperature_c)
{
    openk4a_capture_t *capture = (openk4a_capture_t *)capture_handle;
    if (capture != NULL)
    {
        capture->temperature_c = temperature_c;
    }
}

float k4a_capture_get_temperature_c(k4a_capture_t capture_handle)
{
    openk4a_capture_t *capture = (openk4a_capture_t *)capture_handle;
    return capture != NULL ? capture->temperature_c : 0.0f;
}

/*-----------------------------------------------------------------------------
  The IMU image the SDK keeps inside a capture. It is not part of the public
  API, but the SDK's own IMU path is a capture with one image in it, so the
  shape is kept and openk4a_imu.c uses it.
---------------------------------------------------------------------------*/

void openk4a_capture_set_imu_image(k4a_capture_t capture_handle, k4a_image_t image_handle)
{
    capture_set(capture_handle, OPENK4A_CAPTURE_IMU, image_handle);
}

k4a_image_t openk4a_capture_get_imu_image(k4a_capture_t capture_handle)
{
    return capture_get(capture_handle, OPENK4A_CAPTURE_IMU);
}
