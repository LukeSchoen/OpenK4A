/*=============================================================================
  Images.

  An image is a description of a buffer plus whoever owns the buffer: one
  this tree allocated, or one a caller supplied with a release callback. It
  carries the frame's own metadata - the device timestamp from the MCU's
  footer, the host timestamp from the moment the frame arrived, and for a
  colour frame the exposure, white balance and ISO the sensor reported - so a
  program never has to invent any of it.
=============================================================================*/

#include "openk4a.h"

typedef struct
{
    k4a_image_format_t format;
    int width;
    int height;
    int stride_bytes;
    size_t size;
    uint8_t *buffer;
    openk4a_image_free_fn *free_fn;
    void *free_context;
    int refs;

    uint64_t device_timestamp_usec;
    uint64_t system_timestamp_nsec;
    uint64_t exposure_usec;
    uint32_t white_balance;
    uint32_t iso_speed;
} openk4a_image_t;

int openk4a_format_stride(k4a_image_format_t format, int width)
{
    switch (format)
    {
    case K4A_IMAGE_FORMAT_DEPTH16:
    case K4A_IMAGE_FORMAT_IR16:
    case K4A_IMAGE_FORMAT_COLOR_YUY2:
    case K4A_IMAGE_FORMAT_CUSTOM16:
        return width * 2;
    case K4A_IMAGE_FORMAT_COLOR_BGRA32:
        return width * 4;
    case K4A_IMAGE_FORMAT_COLOR_NV12:
    case K4A_IMAGE_FORMAT_COLOR_MJPG:
        /* Compressed or planar: a JPEG's stride is meaningless, and NV12 is
         * one byte a pixel for the luma plane. */
        return format == K4A_IMAGE_FORMAT_COLOR_NV12 ? width : 0;
    default:
        return width;
    }
}

k4a_image_t openk4a_image_create(k4a_image_format_t format,
                             int width,
                             int height,
                             int stride_bytes,
                             size_t size,
                             uint8_t *buffer,
                             openk4a_image_free_fn *free_fn,
                             void *free_context)
{
    openk4a_image_t *image = (openk4a_image_t *)openk4a_alloc_zero(sizeof(*image));
    if (image == NULL)
    {
        return NULL;
    }
    image->format = format;
    image->width = width;
    image->height = height;
    image->stride_bytes = stride_bytes;
    image->size = size;
    image->buffer = buffer;
    image->free_fn = free_fn;
    image->free_context = free_context;
    image->refs = 1;
    return (k4a_image_t)image;
}

k4a_image_t openk4a_image_alloc(k4a_image_format_t format, int width, int height, int stride_bytes)
{
    const int stride = stride_bytes != 0 ? stride_bytes : openk4a_format_stride(format, width);
    const size_t size = (size_t)stride * (size_t)height;
    uint8_t *buffer = (uint8_t *)openk4a_alloc_zero(size);
    if (buffer == NULL)
    {
        return NULL;
    }
    k4a_image_t image = openk4a_image_create(format, width, height, stride, size, buffer, NULL, NULL);
    if (image == NULL)
    {
        openk4a_free(buffer);
        return NULL;
    }
    return image;
}

void openk4a_image_add_ref(k4a_image_t image)
{
    openk4a_image_t *self = (openk4a_image_t *)image;
    if (self != NULL)
    {
        self->refs++;
    }
}

void openk4a_image_dec_ref(k4a_image_t image)
{
    openk4a_image_t *self = (openk4a_image_t *)image;
    if (self == NULL)
    {
        return;
    }
    if (--self->refs > 0)
    {
        return;
    }
    if (self->free_fn != NULL)
    {
        self->free_fn(self->buffer, self->free_context);
    }
    else
    {
        openk4a_free(self->buffer);
    }
    openk4a_free(self);
}

/*-----------------------------------------------------------------------------
  The public face
---------------------------------------------------------------------------*/

k4a_result_t k4a_image_create(k4a_image_format_t format,
                              int width_pixels,
                              int height_pixels,
                              int stride_bytes,
                              k4a_image_t *image_handle)
{
    if (image_handle == NULL || width_pixels <= 0 || height_pixels <= 0 || stride_bytes <= 0)
    {
        return K4A_RESULT_FAILED;
    }
    k4a_image_t image = openk4a_image_alloc(format, width_pixels, height_pixels, stride_bytes);
    if (image == NULL)
    {
        return K4A_RESULT_FAILED;
    }
    *image_handle = image;
    return K4A_RESULT_SUCCEEDED;
}

k4a_result_t k4a_image_create_from_buffer(k4a_image_format_t format,
                                          int width_pixels,
                                          int height_pixels,
                                          int stride_bytes,
                                          uint8_t *buffer,
                                          size_t buffer_size,
                                          k4a_memory_destroy_cb_t *buffer_release_cb,
                                          void *buffer_release_cb_context,
                                          k4a_image_t *image_handle)
{
    if (image_handle == NULL || buffer == NULL || width_pixels <= 0 || height_pixels <= 0 || stride_bytes <= 0 ||
        buffer_size == 0)
    {
        return K4A_RESULT_FAILED;
    }
    k4a_image_t image = openk4a_image_create(format,
                                         width_pixels,
                                         height_pixels,
                                         stride_bytes,
                                         buffer_size,
                                         buffer,
                                         (openk4a_image_free_fn *)buffer_release_cb,
                                         buffer_release_cb_context);
    if (image == NULL)
    {
        return K4A_RESULT_FAILED;
    }
    *image_handle = image;
    return K4A_RESULT_SUCCEEDED;
}

uint8_t *k4a_image_get_buffer(k4a_image_t image_handle)
{
    openk4a_image_t *image = (openk4a_image_t *)image_handle;
    return image != NULL ? image->buffer : NULL;
}

size_t k4a_image_get_size(k4a_image_t image_handle)
{
    openk4a_image_t *image = (openk4a_image_t *)image_handle;
    return image != NULL ? image->size : 0;
}

k4a_image_format_t k4a_image_get_format(k4a_image_t image_handle)
{
    openk4a_image_t *image = (openk4a_image_t *)image_handle;
    return image != NULL ? image->format : K4A_IMAGE_FORMAT_CUSTOM;
}

int k4a_image_get_width_pixels(k4a_image_t image_handle)
{
    openk4a_image_t *image = (openk4a_image_t *)image_handle;
    return image != NULL ? image->width : 0;
}

int k4a_image_get_height_pixels(k4a_image_t image_handle)
{
    openk4a_image_t *image = (openk4a_image_t *)image_handle;
    return image != NULL ? image->height : 0;
}

int k4a_image_get_stride_bytes(k4a_image_t image_handle)
{
    openk4a_image_t *image = (openk4a_image_t *)image_handle;
    return image != NULL ? image->stride_bytes : 0;
}

uint64_t k4a_image_get_device_timestamp_usec(k4a_image_t image_handle)
{
    openk4a_image_t *image = (openk4a_image_t *)image_handle;
    return image != NULL ? image->device_timestamp_usec : 0;
}

uint64_t k4a_image_get_system_timestamp_nsec(k4a_image_t image_handle)
{
    openk4a_image_t *image = (openk4a_image_t *)image_handle;
    return image != NULL ? image->system_timestamp_nsec : 0;
}

uint64_t k4a_image_get_exposure_usec(k4a_image_t image_handle)
{
    openk4a_image_t *image = (openk4a_image_t *)image_handle;
    return image != NULL ? image->exposure_usec : 0;
}

uint32_t k4a_image_get_white_balance(k4a_image_t image_handle)
{
    openk4a_image_t *image = (openk4a_image_t *)image_handle;
    return image != NULL ? image->white_balance : 0;
}

uint32_t k4a_image_get_iso_speed(k4a_image_t image_handle)
{
    openk4a_image_t *image = (openk4a_image_t *)image_handle;
    return image != NULL ? image->iso_speed : 0;
}

void k4a_image_set_device_timestamp_usec(k4a_image_t image_handle, uint64_t timestamp_usec)
{
    openk4a_image_t *image = (openk4a_image_t *)image_handle;
    if (image != NULL)
    {
        image->device_timestamp_usec = timestamp_usec;
    }
}

void k4a_image_set_system_timestamp_nsec(k4a_image_t image_handle, uint64_t timestamp_nsec)
{
    openk4a_image_t *image = (openk4a_image_t *)image_handle;
    if (image != NULL)
    {
        image->system_timestamp_nsec = timestamp_nsec;
    }
}

void k4a_image_set_exposure_usec(k4a_image_t image_handle, uint64_t exposure_usec)
{
    openk4a_image_t *image = (openk4a_image_t *)image_handle;
    if (image != NULL)
    {
        image->exposure_usec = exposure_usec;
    }
}

void k4a_image_set_white_balance(k4a_image_t image_handle, uint32_t white_balance)
{
    openk4a_image_t *image = (openk4a_image_t *)image_handle;
    if (image != NULL)
    {
        image->white_balance = white_balance;
    }
}

void k4a_image_set_iso_speed(k4a_image_t image_handle, uint32_t iso_speed)
{
    openk4a_image_t *image = (openk4a_image_t *)image_handle;
    if (image != NULL)
    {
        image->iso_speed = iso_speed;
    }
}

void k4a_image_reference(k4a_image_t image_handle)
{
    openk4a_image_add_ref(image_handle);
}

void k4a_image_release(k4a_image_t image_handle)
{
    openk4a_image_dec_ref(image_handle);
}
