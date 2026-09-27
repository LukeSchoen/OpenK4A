/*=============================================================================
  The colour camera.

  The colour device is a UVC camera, and Windows has it bound to its own
  driver - which is what makes this the honest way to read it: Media
  Foundation is how a program gets a UVC camera's frames on this machine
  without installing a driver over someone else's.

  What comes back is the sensor's own MJPG blocks, undecoded. A JPEG is not a
  picture until a program decodes it, and the raw blocks are smaller and
  lossless in the sense that matters: they are what the camera saw, before
  anyone's conversion. A caller that asks for BGRA gets Media Foundation's
  own conversion instead, which is the same decoder the closed SDK uses.

  The camera's controls - exposure, white balance, ISO, gain and the rest -
  are kernel streaming properties on the media source, reached through
  IKsControl. The values are the sensor's own, and the exposure in particular
  is a logarithmic exponent on this side and microseconds on the API's, so
  the conversion table the SDK carries is kept here.
=============================================================================*/

#include "openk4a.h"
#include "openk4a_mf.h"

#define OPENK4A_COLOR_QUEUE 8

typedef struct openk4a_color_callback openk4a_color_callback_t;

typedef struct
{
    openk4a_mf_unknown_vtbl_t unknown;
    HRESULT (*OnReadSample)(openk4a_color_callback_t *self, HRESULT status, DWORD stream_index, DWORD stream_flags,
                            LONGLONG timestamp, openk4a_mf_sample_t *sample);
    HRESULT (*OnFlush)(openk4a_color_callback_t *self, DWORD stream_index);
    HRESULT (*OnEvent)(openk4a_color_callback_t *self, DWORD stream_index, void *event);
} openk4a_color_callback_vtbl_t;

struct openk4a_color
{
    openk4a_mf_activate_t *activate;
    struct openk4a_mf_media_source *source;
    openk4a_mf_source_reader_t *reader;
    openk4a_mf_ks_control_t *ks;
    openk4a_color_callback_t *callback;

    CRITICAL_SECTION lock;
    HANDLE flushed;
    HANDLE arrived;   /* set when the queue gains a frame */
    bool mf_started;
    bool reader_created;
    bool started;
    bool flushing;
    bool failed;

    int width;
    int height;
    int stride;
    k4a_image_format_t format;
    GUID device_subtype;
    GUID output_subtype;
    bool use_60hz_power;

    k4a_image_t queue[OPENK4A_COLOR_QUEUE];
    int queue_count;

    openk4a_color_control_capabilities_t capabilities[K4A_COLOR_CONTROL_POWERLINE_FREQUENCY + 1];
};

struct openk4a_color_callback
{
    const openk4a_color_callback_vtbl_t *vtable;
    LONG refs;
    openk4a_color_t *color;
};

/*=============================================================================
  The reader's callback
=============================================================================*/

/* The interfaces this object answers to. Media Foundation asks: a callback
 * that says no to everything is a callback the reader throws away, and the
 * stream then fails with E_POINTER because nobody is there to receive a
 * sample. */
static const GUID OPENK4A_IID_IUNKNOWN = {
    0x00000000, 0x0000, 0x0000, { 0xc0, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x46 }
};
static const GUID OPENK4A_IID_IMFSOURCEREADERCALLBACK = {
    0xdeec8d99, 0xfa1d, 0x4d82, { 0x84, 0xc2, 0x2c, 0x89, 0x69, 0x94, 0x48, 0x67 }
};

static ULONG callback_add_ref(openk4a_color_callback_t *self);

static HRESULT callback_query_interface(openk4a_color_callback_t *self, const GUID *riid, void **object)
{
    if (object == NULL)
    {
        return (HRESULT)0x80004003L; /* E_POINTER */
    }
    if (memcmp(riid, &OPENK4A_IID_IUNKNOWN, sizeof(GUID)) == 0 ||
        memcmp(riid, &OPENK4A_IID_IMFSOURCEREADERCALLBACK, sizeof(GUID)) == 0)
    {
        callback_add_ref(self);
        *object = self;
        return 0;
    }
    *object = NULL;
    return (HRESULT)0x80004002L; /* E_NOINTERFACE */
}

static ULONG callback_add_ref(openk4a_color_callback_t *self)
{
    return (ULONG)InterlockedIncrement(&self->refs);
}

static ULONG callback_release(openk4a_color_callback_t *self)
{
    const LONG refs = InterlockedDecrement(&self->refs);
    if (refs == 0)
    {
        openk4a_free(self);
    }
    return (ULONG)refs;
}

/* The device's own frame time, in 90 kHz ticks, out of the metadata list
 * Media Foundation attaches to the sample. It is what puts a colour frame on
 * the same clock as a depth frame. */
static UINT64 color_frame_pts(openk4a_mf_attributes_t *metadata)
{
    openk4a_mf_media_buffer_t *raw = NULL;
    if (metadata->vtable->GetUnknown(metadata, &OPENK4A_MF_CAPTURE_METADATA_FRAME_RAWSTREAM, &OPENK4A_IID_IUNKNOWN,
                                     (void **)&raw) < 0 ||
        raw == NULL)
    {
        return 0;
    }

    UINT64 frame_pts = 0;
    BYTE *data = NULL;
    DWORD length = 0;
    if (raw->vtable->Lock(raw, &data, NULL, &length) >= 0 && data != NULL)
    {
        LONG left = (LONG)length;
        const BYTE *at = data;
        while (left >= (LONG)sizeof(openk4a_ks_metadata_header_t))
        {
            const openk4a_ks_metadata_header_t *item = (const openk4a_ks_metadata_header_t *)at;
            if (item->metadata_id == OPENK4A_METADATA_ID_FRAME_ALIGN_INFO &&
                left >= (LONG)sizeof(openk4a_ks_frame_align_t))
            {
                frame_pts = ((const openk4a_ks_frame_align_t *)item)->frame_pts;
            }
            if (item->size == 0)
            {
                break;
            }
            left -= (LONG)item->size;
            at += item->size;
        }
        (void)raw->vtable->Unlock(raw);
    }
    raw->vtable->unknown.Release(raw);
    return frame_pts;
}

/* Reads a sample's bytes and metadata into an image this tree owns. The bytes
 * are copied rather than held: Media Foundation recycles its samples as soon
 * as the callback returns, and a frame a caller is still holding must not be
 * one of them. */
static void color_accept_sample(openk4a_color_t *color, openk4a_mf_sample_t *sample)
{
    UINT64 frame_pts = 0;
    UINT64 device_timestamp_100nsec = 0;
    UINT64 exposure_hns = 0;
    UINT32 white_balance = 0;
    UINT32 iso_speed = 0;

    openk4a_mf_attributes_t *metadata = NULL;
    if (sample->vtable->attributes.GetUnknown(sample, &OPENK4A_MFSampleExtension_CaptureMetadata, &OPENK4A_IID_IUNKNOWN,
                                              (void **)&metadata) >= 0 &&
        metadata != NULL)
    {
        (void)metadata->vtable->GetUINT64(metadata, &OPENK4A_MF_CAPTURE_METADATA_EXPOSURE_TIME, &exposure_hns);
        (void)metadata->vtable->GetUINT32(metadata, &OPENK4A_MF_CAPTURE_METADATA_WHITEBALANCE, &white_balance);
        (void)metadata->vtable->GetUINT32(metadata, &OPENK4A_MF_CAPTURE_METADATA_ISO_SPEED, &iso_speed);
        frame_pts = color_frame_pts(metadata);
        metadata->vtable->unknown.Release(metadata);
    }
    (void)sample->vtable->attributes.GetUINT64(sample, &OPENK4A_MFSampleExtension_DeviceTimestamp,
                                              &device_timestamp_100nsec);

    /* A sample with no frame time is one the pipeline made up; the SDK drops
     * those too, because it cannot be placed on the depth clock. But the
     * camera's own device timestamp is on the same clock and is always there,
     * and a frame that came through the pipeline undecoded - MJPG is handed
     * over as it arrived - does not always carry the raw-stream metadata the
     * frame time is read from. So the device clock is the fallback, and only a
     * sample with neither is dropped. */
    if (frame_pts == 0)
    {
        if (getenv("OPENK4A_TRACE_COLOR") != NULL)
        {
            openk4a_log(OPENK4A_LOG_INFO, "colour: sample with no frame time and no device time, dropped");
        }
        if (device_timestamp_100nsec == 0)
        {
            return;
        }
        /* Hundreds of nanoseconds to the camera's 90 kHz ticks. */
        frame_pts = device_timestamp_100nsec * 9ull / 1000ull;
    }

    openk4a_mf_media_buffer_t *buffer = NULL;
    if (sample->vtable->ConvertToContiguousBuffer(sample, &buffer) < 0 || buffer == NULL)
    {
        return;
    }

    BYTE *data = NULL;
    DWORD length = 0;
    if (buffer->vtable->Lock(buffer, &data, NULL, &length) < 0 || data == NULL || length == 0)
    {
        buffer->vtable->unknown.Release(buffer);
        return;
    }

    const int stride = color->stride;
    const size_t wanted = color->format == K4A_IMAGE_FORMAT_COLOR_MJPG
                              ? (size_t)length
                              : (size_t)stride * (size_t)color->height;
    if (color->format != K4A_IMAGE_FORMAT_COLOR_MJPG && (size_t)length < wanted)
    {
        (void)buffer->vtable->Unlock(buffer);
        buffer->vtable->unknown.Release(buffer);
        return;
    }

    /* An MJPG image is a block of bytes, not a grid: it has no stride, and
     * its size is whatever the block came to. */
    uint8_t *copy = (uint8_t *)openk4a_alloc_zero(wanted);
    k4a_image_t image = copy != NULL
                            ? openk4a_image_create(color->format, color->width, color->height, stride, wanted, copy,
                                               NULL, NULL)
                            : NULL;
    if (image != NULL)
    {
        memcpy(copy, data, wanted);
    }
    else
    {
        openk4a_free(copy);
    }
    (void)buffer->vtable->Unlock(buffer);
    buffer->vtable->unknown.Release(buffer);
    if (image == NULL)
    {
        return;
    }

    k4a_image_set_device_timestamp_usec(image, frame_pts * 100ULL / 9ULL);
    k4a_image_set_system_timestamp_nsec(image, device_timestamp_100nsec * 100ULL);
    k4a_image_set_exposure_usec(image, exposure_hns / 10ULL);
    k4a_image_set_white_balance(image, white_balance);
    k4a_image_set_iso_speed(image, iso_speed);

    EnterCriticalSection(&color->lock);
    if (color->queue_count == OPENK4A_COLOR_QUEUE)
    {
        k4a_image_release(color->queue[0]);
        memmove(color->queue, color->queue + 1, sizeof(color->queue) - sizeof(color->queue[0]));
        color->queue_count--;
    }
    color->queue[color->queue_count++] = image;
    LeaveCriticalSection(&color->lock);
    if (color->arrived != NULL)
    {
        SetEvent(color->arrived);
    }

    openk4a_log(OPENK4A_LOG_TRACE, "colour: frame of %zu bytes, device time %llu usec",
            k4a_image_get_size(image),
            (unsigned long long)k4a_image_get_device_timestamp_usec(image));
}

static HRESULT callback_on_read_sample(openk4a_color_callback_t *self,
                                       HRESULT status,
                                       DWORD stream_index,
                                       DWORD stream_flags,
                                       LONGLONG timestamp,
                                       openk4a_mf_sample_t *sample)
{
    (void)stream_flags;
    (void)timestamp;
    openk4a_color_t *color = self->color;

    if (getenv("OPENK4A_TRACE_COLOR") != NULL)
    {
        openk4a_log(OPENK4A_LOG_INFO,
                "colour: sample status %08lx, flags %lx, %s, started %d",
                (unsigned long)status,
                (unsigned long)stream_flags,
                sample != NULL ? "with a sample" : "no sample",
                (int)color->started);
    }
    if (status >= 0 && sample != NULL && color->started && !color->flushing)
    {
        color_accept_sample(color, sample);
    }
    else if (status < 0)
    {
        openk4a_log(OPENK4A_LOG_ERROR, "the colour pipeline reported 0x%08lx", (unsigned long)status);
        EnterCriticalSection(&color->lock);
        color->failed = true;
        LeaveCriticalSection(&color->lock);
    }

    if (color->started && !color->flushing &&
        color->reader->vtable->ReadSample(color->reader,
                                          OPENK4A_MF_SOURCE_READER_FIRST_VIDEO_STREAM,
                                          0,
                                          NULL,
                                          NULL,
                                          NULL,
                                          NULL) < 0)
    {
        openk4a_log(OPENK4A_LOG_ERROR, "the colour stream would not take the next read");
    }
    (void)stream_index;
    return 0;
}

static HRESULT callback_on_flush(openk4a_color_callback_t *self, DWORD stream_index)
{
    (void)stream_index;
    openk4a_color_t *color = self->color;
    if (!color->started)
    {
        (void)color->reader->vtable->SetStreamSelection(color->reader,
                                                        OPENK4A_MF_SOURCE_READER_FIRST_VIDEO_STREAM,
                                                        FALSE);
    }
    if (color->flushed != NULL)
    {
        SetEvent(color->flushed);
    }
    color->flushing = false;
    return 0;
}

static HRESULT callback_on_event(openk4a_color_callback_t *self, DWORD stream_index, void *event)
{
    (void)self;
    (void)stream_index;
    (void)event;
    return 0;
}

static const openk4a_color_callback_vtbl_t OPENK4A_COLOR_CALLBACK_VTABLE = {
    { (HRESULT (*)(void *, const GUID *, void **))(void *)callback_query_interface,
      (ULONG (*)(void *))(void *)callback_add_ref,
      (ULONG (*)(void *))(void *)callback_release },
    callback_on_read_sample,
    callback_on_flush,
    callback_on_event,
};

/*=============================================================================
  Finding the camera
=============================================================================*/

static bool wide_contains(const WCHAR *text, const WCHAR *needle)
{
    const size_t length = wcslen(text);
    const size_t wanted = wcslen(needle);
    if (wanted > length)
    {
        return false;
    }
    for (size_t i = 0; i + wanted <= length; i++)
    {
        size_t j = 0;
        while (j < wanted)
        {
            WCHAR a = text[i + j];
            WCHAR b = needle[j];
            if (a >= L'A' && a <= L'Z')
            {
                a = (WCHAR)(a - L'A' + L'a');
            }
            if (b >= L'A' && b <= L'Z')
            {
                b = (WCHAR)(b - L'A' + L'a');
            }
            if (a != b)
            {
                break;
            }
            j++;
        }
        if (j == wanted)
        {
            return true;
        }
    }
    return false;
}

/* The depth processor's container, which the colour camera shares: two
 * functions of one camera have the same container id, and that is what pairs
 * them when more than one camera is attached. */
static bool depth_container(const char *depth_path, GUID *container_id)
{
    WCHAR interface_path[512];
    MultiByteToWideChar(CP_ACP, 0, depth_path, -1, interface_path, 512);
    WCHAR instance[512];
    if (!openk4a_win_instance_from_interface(interface_path, instance, 512))
    {
        return false;
    }
    return openk4a_win_container_id_from_instance(instance, container_id);
}

static bool find_color_activate(const char *depth_path, openk4a_mf_activate_t **out)
{
    *out = NULL;
    GUID wanted;
    const bool have_container = depth_container(depth_path, &wanted);

    openk4a_mf_attributes_t *attributes = NULL;
    if (openk4a_mf.CreateAttributes(&attributes, 2) < 0)
    {
        return false;
    }
    if (attributes->vtable->SetGUID(attributes, &OPENK4A_MF_DEVSOURCE_ATTRIBUTE_SOURCE_TYPE,
                                    &OPENK4A_MF_DEVSOURCE_ATTRIBUTE_SOURCE_TYPE_VIDCAP_GUID) < 0)
    {
        attributes->vtable->unknown.Release(attributes);
        return false;
    }

    openk4a_mf_activate_t **devices = NULL;
    UINT32 count = 0;
    if (openk4a_mf.EnumDeviceSources(attributes, &devices, &count) < 0)
    {
        attributes->vtable->unknown.Release(attributes);
        return false;
    }

    openk4a_mf_activate_t *fallback = NULL;
    for (UINT32 i = 0; i < count; i++)
    {
        UINT32 length = 0;
        if (devices[i]->vtable->attributes.GetStringLength(
                devices[i], &OPENK4A_MF_DEVSOURCE_ATTRIBUTE_SOURCE_TYPE_VIDCAP_SYMBOLIC_LINK, &length) < 0)
        {
            continue;
        }
        WCHAR *link = (WCHAR *)openk4a_alloc(((size_t)length + 1) * sizeof(WCHAR));
        if (link == NULL)
        {
            continue;
        }
        UINT32 written = 0;
        if (devices[i]->vtable->attributes.GetString(
                devices[i], &OPENK4A_MF_DEVSOURCE_ATTRIBUTE_SOURCE_TYPE_VIDCAP_SYMBOLIC_LINK, link, length + 1,
                &written) < 0)
        {
            openk4a_free(link);
            continue;
        }

        if (wide_contains(link, L"vid_045e&pid_097d"))
        {
            if (fallback == NULL)
            {
                fallback = devices[i];
                fallback->vtable->attributes.unknown.AddRef(fallback);
            }
            GUID container;
            WCHAR instance[512];
            memset(instance, 0, sizeof(instance));
            const bool have_instance = openk4a_win_instance_from_interface(link, instance, 512);
            if (have_container && have_instance &&
                openk4a_win_container_id_from_instance(instance, &container) && memcmp(&container, &wanted,
                                                                                  sizeof(GUID)) == 0)
            {
                *out = devices[i];
                (*out)->vtable->attributes.unknown.AddRef(*out);
                openk4a_free(link);
                break;
            }
        }
        openk4a_free(link);
    }

    if (*out == NULL && fallback != NULL)
    {
        *out = fallback;
        fallback = NULL;
    }
    if (fallback != NULL)
    {
        fallback->vtable->attributes.unknown.Release(fallback);
    }
    /* The array is COM's: every activate in it carries a reference of ours,
     * and the array itself is COM's memory. */
    if (devices != NULL)
    {
        for (UINT32 i = 0; i < count; i++)
        {
            devices[i]->vtable->attributes.unknown.Release(devices[i]);
        }
        if (openk4a_win.co_task_mem_free != NULL)
        {
            typedef void(WINAPI * co_task_mem_free_t)(void *);
            ((co_task_mem_free_t)openk4a_win.co_task_mem_free)(devices);
        }
    }
    attributes->vtable->unknown.Release(attributes);
    return *out != NULL;
}

/*=============================================================================
  Creating and starting
=============================================================================*/

static bool color_init_com(void)
{
    static bool done;
    if (done)
    {
        return true;
    }
    if (openk4a_win.co_initialize_ex != NULL)
    {
        typedef HRESULT(WINAPI * co_initialize_ex_t)(void *, DWORD);
        /* An apartment already exists in most hosts, and that is not an
         * error here. */
        (void)((co_initialize_ex_t)openk4a_win.co_initialize_ex)(NULL, 0x0 /* COINIT_MULTITHREADED */);
    }
    done = true;
    return true;
}

openk4a_color_t *openk4a_color_create(const openk4a_usb_t *color_mcu, uint32_t device_index)
{
    (void)device_index;
    if (!openk4a_mf_load())
    {
        return NULL;
    }
    color_init_com();

    openk4a_color_t *color = (openk4a_color_t *)openk4a_alloc_zero(sizeof(openk4a_color_t));
    if (color == NULL)
    {
        return NULL;
    }
    InitializeCriticalSection(&color->lock);
    color->flushed = CreateEventA(NULL, FALSE, FALSE, NULL);
    /* Automatic reset: one wait is answered by one frame, so a caller that has
     * already taken a frame is not told about it twice. */
    color->arrived = CreateEventA(NULL, FALSE, FALSE, NULL);

    /* MFSTARTUP_LITE: the socket library is not needed for a local camera. */
    if (openk4a_mf.Startup(0x00020070, 1) < 0)
    {
        openk4a_log(OPENK4A_LOG_ERROR, "Media Foundation would not start");
        openk4a_color_destroy(color);
        return NULL;
    }
    color->mf_started = true;

    if (!find_color_activate(color_mcu != NULL ? color_mcu->path : "", &color->activate))
    {
        openk4a_log(OPENK4A_LOG_ERROR, "no Azure Kinect colour camera is available - is it in use?");
        openk4a_color_destroy(color);
        return NULL;
    }

    if (openk4a_mf.CreateDeviceSource(color->activate, &color->source) < 0 || color->source == NULL)
    {
        openk4a_log(OPENK4A_LOG_ERROR, "the colour camera would not activate");
        openk4a_color_destroy(color);
        return NULL;
    }

    openk4a_mf_attributes_t *attributes = NULL;
    if (openk4a_mf.CreateAttributes(&attributes, 3) < 0)
    {
        openk4a_color_destroy(color);
        return NULL;
    }

    color->callback = (openk4a_color_callback_t *)openk4a_alloc_zero(sizeof(openk4a_color_callback_t));
    if (color->callback == NULL)
    {
        attributes->vtable->unknown.Release(attributes);
        openk4a_color_destroy(color);
        return NULL;
    }
    color->callback->vtable = &OPENK4A_COLOR_CALLBACK_VTABLE;
    color->callback->refs = 1;
    color->callback->color = color;

    const HRESULT callback_set =
        attributes->vtable->SetUnknown(attributes, &OPENK4A_MF_SOURCE_READER_ASYNC_CALLBACK, color->callback);
    const HRESULT advanced =
        attributes->vtable->SetUINT32(attributes, &OPENK4A_MF_SOURCE_READER_ENABLE_ADVANCED_VIDEO_PROCESSING, 1);
    const HRESULT no_frc = attributes->vtable->SetUINT32(attributes, &OPENK4A_MF_XVP_DISABLE_FRC, 1);
    if (callback_set < 0 || advanced < 0 || no_frc < 0)
    {
        openk4a_log(OPENK4A_LOG_TRACE,
                "colour: reader attributes: callback %08lx, advanced %08lx, no-frc %08lx",
                (unsigned long)callback_set, (unsigned long)advanced, (unsigned long)no_frc);
    }

    if (openk4a_mf.CreateSourceReaderFromMediaSource(color->source, attributes, &color->reader) < 0)
    {
        openk4a_log(OPENK4A_LOG_ERROR, "the colour reader would not be created");
        attributes->vtable->unknown.Release(attributes);
        openk4a_color_destroy(color);
        return NULL;
    }
    attributes->vtable->unknown.Release(attributes);
    color->reader_created = true;

    (void)color->reader->vtable->SetStreamSelection(color->reader, (DWORD)OPENK4A_MF_SOURCE_READER_ALL_STREAMS, FALSE);

    /* The controls come off the media source itself, so they are there before
     * any frame is. */
    void *ks = NULL;
    if (color->reader->vtable->GetServiceForStream(color->reader,
                                                   OPENK4A_MF_SOURCE_READER_MEDIASOURCE,
                                                   &(GUID){ 0x00000000, 0x0000, 0x0000,
                                                            { 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00 } },
                                                   &OPENK4A_IID_IKSCONTROL,
                                                   &ks) >= 0)
    {
        color->ks = (openk4a_mf_ks_control_t *)ks;
    }
    color->use_60hz_power = true;
    return color;
}

void openk4a_color_destroy(openk4a_color_t *color)
{
    if (color == NULL)
    {
        return;
    }
    openk4a_color_stop(color);

    for (int i = 0; i < color->queue_count; i++)
    {
        k4a_image_release(color->queue[i]);
    }
    color->queue_count = 0;

    if (color->ks != NULL)
    {
        color->ks->vtable->unknown.Release(color->ks);
    }
    if (color->reader != NULL)
    {
        color->reader->vtable->unknown.Release(color->reader);
    }
    if (color->callback != NULL)
    {
        callback_release(color->callback);
    }
    if (color->source != NULL)
    {
        ((openk4a_mf_object_t *)color->source)->vtable->Release(color->source);
        color->source = NULL;
    }
    if (color->activate != NULL)
    {
        color->activate->vtable->attributes.unknown.Release(color->activate);
    }
    if (color->mf_started)
    {
        (void)openk4a_mf.Shutdown();
    }
    if (color->flushed != NULL)
    {
        CloseHandle(color->flushed);
    }
    if (color->arrived != NULL)
    {
        CloseHandle(color->arrived);
    }
    DeleteCriticalSection(&color->lock);
    openk4a_free(color);
}

/*=============================================================================
  Starting and stopping the stream
=============================================================================*/

static bool color_pick_format(openk4a_color_t *color, const k4a_device_configuration_t *config)
{
    int width = 0;
    int height = 0;
    if (!openk4a_color_grid(config->color_resolution, &width, &height))
    {
        return false;
    }

    switch (config->color_format)
    {
    case K4A_IMAGE_FORMAT_COLOR_NV12:
        color->device_subtype = OPENK4A_MFVideoFormat_NV12;
        color->output_subtype = OPENK4A_MFVideoFormat_NV12;
        break;
    case K4A_IMAGE_FORMAT_COLOR_YUY2:
        color->device_subtype = OPENK4A_MFVideoFormat_YUY2;
        color->output_subtype = OPENK4A_MFVideoFormat_YUY2;
        break;
    case K4A_IMAGE_FORMAT_COLOR_MJPG:
        color->device_subtype = OPENK4A_MFVideoFormat_MJPG;
        color->output_subtype = OPENK4A_MFVideoFormat_MJPG;
        break;
    case K4A_IMAGE_FORMAT_COLOR_BGRA32:
        /* Always the sensor's own blocks, whatever the resolution: the colour
         * stream shares the cable with the depth stream, which is 5.3 MB a
         * frame, and uncompressed colour at 720p is enough to cost the depth
         * stream every other frame. Decoding a tenth as many bytes is worth
         * far more than the JPEG decode costs. */
        color->device_subtype = OPENK4A_MFVideoFormat_MJPG;
        color->output_subtype = OPENK4A_MFVideoFormat_ARGB32;
        break;
    default:
        openk4a_log(OPENK4A_LOG_ERROR, "colour format %d is not one this camera has", (int)config->color_format);
        return false;
    }

    const float fps = config->camera_fps == K4A_FRAMES_PER_SECOND_5 ? 5.0f
                      : config->camera_fps == K4A_FRAMES_PER_SECOND_15 ? 15.0f
                                                                       : 30.0f;

    openk4a_mf_media_type_t *chosen = NULL;
    for (DWORD index = 0;; index++)
    {
        openk4a_mf_media_type_t *type = NULL;
        const HRESULT hr = color->reader->vtable->GetNativeMediaType(color->reader,
                                                                     OPENK4A_MF_SOURCE_READER_FIRST_VIDEO_STREAM,
                                                                     index,
                                                                     &type);
        if (hr < 0 || type == NULL)
        {
            break;
        }
        UINT32 type_width = 0;
        UINT32 type_height = 0;
        UINT32 numerator = 0;
        UINT32 denominator = 0;
        GUID subtype = { 0 };
        if (openk4a_mf_get_size(type, &OPENK4A_MF_MT_FRAME_SIZE, &type_width, &type_height) >= 0 &&
            openk4a_mf_get_ratio(type, &OPENK4A_MF_MT_FRAME_RATE, &numerator, &denominator) >= 0 &&
            type->vtable->attributes.GetGUID(type, &OPENK4A_MF_MT_SUBTYPE, &subtype) >= 0 &&
            (int)type_width == width && (int)type_height == height && denominator != 0 &&
            fps == (float)numerator / (float)denominator && memcmp(&subtype, &color->device_subtype, sizeof(GUID)) == 0)
        {
            chosen = type;
            break;
        }
        type->vtable->attributes.unknown.Release(type);
    }

    if (chosen == NULL)
    {
        /* Nothing matched: say what the camera does offer, because the answer
         * is always one of these. */
        openk4a_log(OPENK4A_LOG_ERROR, "the colour camera does not offer %dx%d at %g fps in the format asked for; it has:",
                width, height, (double)fps);
        for (DWORD index = 0;; index++)
        {
            openk4a_mf_media_type_t *type = NULL;
            if (color->reader->vtable->GetNativeMediaType(color->reader,
                                                          OPENK4A_MF_SOURCE_READER_FIRST_VIDEO_STREAM,
                                                          index,
                                                          &type) < 0 ||
                type == NULL)
            {
                break;
            }
            UINT32 type_width = 0;
            UINT32 type_height = 0;
            UINT32 numerator = 0;
            UINT32 denominator = 0;
            GUID subtype = { 0 };
            if (openk4a_mf_get_size(type, &OPENK4A_MF_MT_FRAME_SIZE, &type_width, &type_height) >= 0 &&
                openk4a_mf_get_ratio(type, &OPENK4A_MF_MT_FRAME_RATE, &numerator, &denominator) >= 0 &&
                type->vtable->attributes.GetGUID(type, &OPENK4A_MF_MT_SUBTYPE, &subtype) >= 0)
            {
                openk4a_log(OPENK4A_LOG_ERROR, "  %ux%u at %u/%u fps, subtype %08lx-%04x",
                        (unsigned)type_width, (unsigned)type_height, (unsigned)numerator, (unsigned)denominator,
                        (unsigned long)subtype.Data1, (unsigned)subtype.Data2);
            }
            type->vtable->attributes.unknown.Release(type);
        }
        return false;
    }

    openk4a_mf_media_type_t *output = chosen;
    if (memcmp(&color->device_subtype, &color->output_subtype, sizeof(GUID)) != 0)
    {
        if (openk4a_mf.CreateMediaType(&output) < 0)
        {
            chosen->vtable->attributes.unknown.Release(chosen);
            return false;
        }
        (void)chosen->vtable->attributes.CopyAllItems(chosen, output);
        (void)output->vtable->attributes.SetGUID(output, &OPENK4A_MF_MT_SUBTYPE, &color->output_subtype);
    }

    const HRESULT set = color->reader->vtable->SetCurrentMediaType(color->reader,
                                                                  OPENK4A_MF_SOURCE_READER_FIRST_VIDEO_STREAM,
                                                                  NULL,
                                                                  output);
    if (output != chosen)
    {
        output->vtable->attributes.unknown.Release(output);
    }
    chosen->vtable->attributes.unknown.Release(chosen);
    if (set < 0)
    {
        openk4a_log(OPENK4A_LOG_ERROR, "the colour camera would not take %dx%d", width, height);
        return false;
    }

    color->width = width;
    color->height = height;
    color->format = config->color_format;
    color->stride = openk4a_format_stride(config->color_format, width);
    return true;
}

bool openk4a_color_start(openk4a_color_t *color, const k4a_device_configuration_t *config)
{
    if (color == NULL || !color->reader_created || color->started)
    {
        return false;
    }
    if (!color_pick_format(color, config))
    {
        return false;
    }
    const HRESULT selected =
        color->reader->vtable->SetStreamSelection(color->reader, OPENK4A_MF_SOURCE_READER_FIRST_VIDEO_STREAM, TRUE);
    if (selected < 0)
    {
        openk4a_log(OPENK4A_LOG_ERROR, "colour: the camera's stream would not be selected (%08lx)", (unsigned long)selected);
        return false;
    }

    EnterCriticalSection(&color->lock);
    color->started = true;
    color->flushing = false;
    color->failed = false;
    LeaveCriticalSection(&color->lock);

    const HRESULT read = color->reader->vtable->ReadSample(color->reader,
                                                           OPENK4A_MF_SOURCE_READER_FIRST_VIDEO_STREAM,
                                                           0,
                                                           NULL,
                                                           NULL,
                                                           NULL,
                                                           NULL);
    if (read < 0)
    {
        color->started = false;
        openk4a_log(OPENK4A_LOG_ERROR, "the colour stream would not start (%08lx)", (unsigned long)read);
        return false;
    }
    return true;
}

void openk4a_color_stop(openk4a_color_t *color)
{
    if (color == NULL || !color->reader_created || !color->started)
    {
        return;
    }
    EnterCriticalSection(&color->lock);
    color->started = false;
    color->flushing = true;
    LeaveCriticalSection(&color->lock);

    if (color->reader->vtable->Flush(color->reader, OPENK4A_MF_SOURCE_READER_FIRST_VIDEO_STREAM) >= 0)
    {
        /* Flushing completes on the reader's own thread. */
        int waits = 0;
        while (color->flushing && waits < 10 && WaitForSingleObject(color->flushed, 500) == WAIT_TIMEOUT)
        {
            waits++;
        }
        if (color->flushing)
        {
            openk4a_log(OPENK4A_LOG_WARNING, "the colour stream did not finish flushing");
        }
    }
    color->flushing = false;
}

/* The newest frame, with everything older let go: the queue is a window on
 * the last few frames, and a caller that is not pairing a colour frame with a
 * depth one wants the picture now rather than the oldest one waiting. */
bool openk4a_color_take_latest(openk4a_color_t *color, openk4a_color_frame_t *out)
{
    out->image = NULL;
    out->device_timestamp_usec = 0;
    if (color == NULL)
    {
        return false;
    }

    EnterCriticalSection(&color->lock);
    if (color->queue_count == 0)
    {
        LeaveCriticalSection(&color->lock);
        return false;
    }
    out->image = color->queue[color->queue_count - 1];
    out->device_timestamp_usec = k4a_image_get_device_timestamp_usec(out->image);
    for (int i = 0; i + 1 < color->queue_count; i++)
    {
        k4a_image_release(color->queue[i]);
    }
    color->queue_count = 0;
    LeaveCriticalSection(&color->lock);
    return true;
}

bool openk4a_color_wait(openk4a_color_t *color, int timeout_ms)
{
    if (color == NULL || color->arrived == NULL)
    {
        return false;
    }
    EnterCriticalSection(&color->lock);
    const bool waiting = color->queue_count == 0;
    LeaveCriticalSection(&color->lock);
    if (!waiting)
    {
        return true;
    }
    const DWORD wait = timeout_ms < 0 ? INFINITE : (DWORD)timeout_ms;
    return WaitForSingleObject(color->arrived, wait) == WAIT_OBJECT_0;
}

bool openk4a_color_take(openk4a_color_t *color, uint64_t depth_device_usec, openk4a_color_frame_t *out)
{
    out->image = NULL;
    out->device_timestamp_usec = 0;
    if (color == NULL)
    {
        return false;
    }

    EnterCriticalSection(&color->lock);
    if (color->queue_count == 0)
    {
        LeaveCriticalSection(&color->lock);
        return false;
    }

    /* The frame nearest the depth frame, out of everything that has arrived:
     * the two sensors are on one clock when the cable says so, and within a
     * frame interval of each other when it does not. */
    int best = 0;
    uint64_t best_distance = UINT64_MAX;
    for (int i = 0; i < color->queue_count; i++)
    {
        const uint64_t timestamp = k4a_image_get_device_timestamp_usec(color->queue[i]);
        const uint64_t distance = timestamp > depth_device_usec ? timestamp - depth_device_usec
                                                                : depth_device_usec - timestamp;
        if (distance < best_distance)
        {
            best_distance = distance;
            best = i;
        }
    }
    out->image = color->queue[best];
    out->device_timestamp_usec = k4a_image_get_device_timestamp_usec(out->image);
    /* Keeping the newest is what makes a late caller catch up rather than
     * walk through a backlog. */
    for (int i = best + 1; i < color->queue_count; i++)
    {
        color->queue[i - 1] = color->queue[i];
    }
    color->queue_count--;
    LeaveCriticalSection(&color->lock);
    return true;
}

void openk4a_color_release(openk4a_color_frame_t *frame)
{
    if (frame != NULL && frame->image != NULL)
    {
        k4a_image_release(frame->image);
        frame->image = NULL;
    }
}

/*=============================================================================
  The controls
=============================================================================*/

/* Windows asks for exposure as a base-2 exponent of a second; the API asks
 * for microseconds. This is the SDK's own table, and the two columns are the
 * exposures a 50 Hz and a 60 Hz supply can actually hold. */
typedef struct
{
    int exponent;
    int usec;
    int usec_50hz;
    int usec_60hz;
} openk4a_exposure_mapping_t;

static const openk4a_exposure_mapping_t OPENK4A_EXPOSURE_MAPPING[] = {
    { -11, 488, 500, 500 },
    { -10, 977, 1250, 1250 },
    { -9, 1953, 2500, 2500 },
    { -8, 3906, 10000, 8330 },
    { -7, 7813, 20000, 16670 },
    { -6, 15625, 30000, 33330 },
    { -5, 31250, 40000, 41670 },
    { -4, 62500, 50000, 50000 },
    { -3, 125000, 60000, 66670 },
    { -2, 250000, 80000, 83330 },
    { -1, 500000, 100000, 100000 },
    { 0, 1000000, 120000, 116670 },
    { 1, 2000000, 130000, 133330 },
};

static LONG exposure_to_exponent(int32_t usec, bool use_60hz)
{
    for (size_t i = 0; i < sizeof(OPENK4A_EXPOSURE_MAPPING) / sizeof(OPENK4A_EXPOSURE_MAPPING[0]); i++)
    {
        const int mapped = use_60hz ? OPENK4A_EXPOSURE_MAPPING[i].usec_60hz : OPENK4A_EXPOSURE_MAPPING[i].usec_50hz;
        if (usec <= mapped)
        {
            return OPENK4A_EXPOSURE_MAPPING[i].exponent;
        }
    }
    return OPENK4A_EXPOSURE_MAPPING[sizeof(OPENK4A_EXPOSURE_MAPPING) / sizeof(OPENK4A_EXPOSURE_MAPPING[0]) - 1].exponent;
}

static LONG exponent_to_exposure(LONG exponent, bool use_60hz)
{
    for (size_t i = 0; i < sizeof(OPENK4A_EXPOSURE_MAPPING) / sizeof(OPENK4A_EXPOSURE_MAPPING[0]); i++)
    {
        if (exponent <= OPENK4A_EXPOSURE_MAPPING[i].exponent)
        {
            return use_60hz ? OPENK4A_EXPOSURE_MAPPING[i].usec_60hz : OPENK4A_EXPOSURE_MAPPING[i].usec_50hz;
        }
    }
    return use_60hz ? OPENK4A_EXPOSURE_MAPPING[sizeof(OPENK4A_EXPOSURE_MAPPING) / sizeof(OPENK4A_EXPOSURE_MAPPING[0]) - 1]
                                       .usec_60hz
                    : OPENK4A_EXPOSURE_MAPPING[sizeof(OPENK4A_EXPOSURE_MAPPING) / sizeof(OPENK4A_EXPOSURE_MAPPING[0]) - 1]
                          .usec_50hz;
}

static bool control_property(k4a_color_control_command_t command, GUID *set, ULONG *id)
{
    switch (command)
    {
    case K4A_COLOR_CONTROL_EXPOSURE_TIME_ABSOLUTE:
        *set = OPENK4A_PROPSETID_VIDCAP_CAMERACONTROL;
        *id = OPENK4A_KSPROPERTY_CAMERACONTROL_EXPOSURE;
        return true;
    case K4A_COLOR_CONTROL_BRIGHTNESS:
        *set = OPENK4A_PROPSETID_VIDCAP_VIDEOPROCAMP;
        *id = OPENK4A_KSPROPERTY_VIDEOPROCAMP_BRIGHTNESS;
        return true;
    case K4A_COLOR_CONTROL_CONTRAST:
        *set = OPENK4A_PROPSETID_VIDCAP_VIDEOPROCAMP;
        *id = OPENK4A_KSPROPERTY_VIDEOPROCAMP_CONTRAST;
        return true;
    case K4A_COLOR_CONTROL_SATURATION:
        *set = OPENK4A_PROPSETID_VIDCAP_VIDEOPROCAMP;
        *id = OPENK4A_KSPROPERTY_VIDEOPROCAMP_SATURATION;
        return true;
    case K4A_COLOR_CONTROL_SHARPNESS:
        *set = OPENK4A_PROPSETID_VIDCAP_VIDEOPROCAMP;
        *id = OPENK4A_KSPROPERTY_VIDEOPROCAMP_SHARPNESS;
        return true;
    case K4A_COLOR_CONTROL_WHITEBALANCE:
        *set = OPENK4A_PROPSETID_VIDCAP_VIDEOPROCAMP;
        *id = OPENK4A_KSPROPERTY_VIDEOPROCAMP_WHITEBALANCE;
        return true;
    case K4A_COLOR_CONTROL_BACKLIGHT_COMPENSATION:
        *set = OPENK4A_PROPSETID_VIDCAP_VIDEOPROCAMP;
        *id = OPENK4A_KSPROPERTY_VIDEOPROCAMP_BACKLIGHT_COMPENSATION;
        return true;
    case K4A_COLOR_CONTROL_GAIN:
        *set = OPENK4A_PROPSETID_VIDCAP_VIDEOPROCAMP;
        *id = OPENK4A_KSPROPERTY_VIDEOPROCAMP_GAIN;
        return true;
    case K4A_COLOR_CONTROL_POWERLINE_FREQUENCY:
        *set = OPENK4A_PROPSETID_VIDCAP_VIDEOPROCAMP;
        *id = OPENK4A_KSPROPERTY_VIDEOPROCAMP_POWERLINE_FREQUENCY;
        return true;
    case K4A_COLOR_CONTROL_AUTO_EXPOSURE_PRIORITY:
        /* Deprecated in the SDK and does nothing here either. */
        return false;
    default:
        return false;
    }
}

static bool ks_get(openk4a_color_t *color, const GUID *set, ULONG id, LONG *value, ULONG *flags, ULONG *capabilities)
{
    if (color->ks == NULL)
    {
        return false;
    }
    openk4a_ks_camera_control_t control;
    memset(&control, 0, sizeof(control));
    control.property.set = *set;
    control.property.id = id;
    control.property.flags = OPENK4A_KSPROPERTY_TYPE_GET;
    control.value = -1;
    ULONG returned = 0;
    if (color->ks->vtable->KsProperty(color->ks, &control, sizeof(control), &control, sizeof(control), &returned) < 0)
    {
        return false;
    }
    if (value != NULL)
    {
        *value = control.value;
    }
    if (flags != NULL)
    {
        *flags = control.flags;
    }
    if (capabilities != NULL)
    {
        *capabilities = control.capabilities;
    }
    return true;
}

static bool ks_set(openk4a_color_t *color, const GUID *set, ULONG id, LONG value, ULONG flags)
{
    if (color->ks == NULL)
    {
        return false;
    }
    openk4a_ks_camera_control_t control;
    memset(&control, 0, sizeof(control));
    control.property.set = *set;
    control.property.id = id;
    control.property.flags = OPENK4A_KSPROPERTY_TYPE_SET;
    control.value = value;
    control.flags = flags;
    ULONG returned = 0;
    return color->ks->vtable->KsProperty(color->ks, &control, sizeof(control), &control, sizeof(control), &returned) >= 0;
}

bool openk4a_color_control_capabilities(openk4a_color_t *color,
                                    k4a_color_control_command_t command,
                                    openk4a_color_control_capabilities_t *out)
{
    if (color == NULL || out == NULL || command < K4A_COLOR_CONTROL_EXPOSURE_TIME_ABSOLUTE ||
        command > K4A_COLOR_CONTROL_POWERLINE_FREQUENCY)
    {
        return false;
    }
    if (color->capabilities[command].valid)
    {
        *out = color->capabilities[command];
        return true;
    }

    GUID set;
    ULONG id;
    if (!control_property(command, &set, &id))
    {
        return false;
    }

    openk4a_ks_member_list_t members;
    openk4a_ks_default_value_t defaults;
    memset(&members, 0, sizeof(members));
    memset(&defaults, 0, sizeof(defaults));

    openk4a_ks_camera_control_t control;
    memset(&control, 0, sizeof(control));
    control.property.set = set;
    control.property.id = id;
    control.property.flags = OPENK4A_KSPROPERTY_TYPE_BASICSUPPORT;
    ULONG returned = 0;
    if (color->ks == NULL ||
        color->ks->vtable->KsProperty(color->ks, &control, sizeof(control), &members, sizeof(members), &returned) < 0)
    {
        return false;
    }

    control.property.flags = OPENK4A_KSPROPERTY_TYPE_DEFAULTVALUES;
    returned = 0;
    if (color->ks->vtable->KsProperty(color->ks, &control, sizeof(control), &defaults, sizeof(defaults), &returned) < 0)
    {
        return false;
    }

    ULONG capabilities = 0;
    (void)ks_get(color, &set, id, NULL, NULL, &capabilities);

    openk4a_color_control_capabilities_t result;
    memset(&result, 0, sizeof(result));
    result.supports_auto = (capabilities & OPENK4A_KS_FLAGS_AUTO) != 0;
    result.min_value = members.step.bounds.signed_minimum;
    result.max_value = members.step.bounds.signed_maximum;
    result.step_value = (int32_t)members.step.stepping_delta;
    result.default_value = defaults.value;
    result.default_mode = K4A_COLOR_CONTROL_MODE_MANUAL;

    if (command == K4A_COLOR_CONTROL_EXPOSURE_TIME_ABSOLUTE)
    {
        result.min_value = exponent_to_exposure(result.min_value, true);
        result.max_value = exponent_to_exposure(result.max_value, true);
        result.default_value = exponent_to_exposure(result.default_value, true);
        result.step_value = 1;
        result.default_mode = K4A_COLOR_CONTROL_MODE_AUTO;
    }
    else if (command == K4A_COLOR_CONTROL_WHITEBALANCE)
    {
        result.default_mode = K4A_COLOR_CONTROL_MODE_AUTO;
    }

    result.valid = true;
    color->capabilities[command] = result;
    *out = result;
    return true;
}

bool openk4a_color_control_get(openk4a_color_t *color, k4a_color_control_command_t command, k4a_color_control_mode_t *mode, int32_t *value)
{
    if (color == NULL || mode == NULL || value == NULL)
    {
        return false;
    }
    GUID set;
    ULONG id;
    if (!control_property(command, &set, &id))
    {
        *mode = K4A_COLOR_CONTROL_MODE_MANUAL;
        *value = 0;
        return command == K4A_COLOR_CONTROL_AUTO_EXPOSURE_PRIORITY;
    }

    LONG raw = 0;
    ULONG flags = 0;
    if (!ks_get(color, &set, id, &raw, &flags, NULL))
    {
        return false;
    }

    *mode = (flags & OPENK4A_KS_FLAGS_MANUAL) != 0 ? K4A_COLOR_CONTROL_MODE_MANUAL
              : (flags & OPENK4A_KS_FLAGS_AUTO) != 0 ? K4A_COLOR_CONTROL_MODE_AUTO
                                                 : K4A_COLOR_CONTROL_MODE_MANUAL;
    *value = command == K4A_COLOR_CONTROL_EXPOSURE_TIME_ABSOLUTE
                 ? exponent_to_exposure(raw, color->use_60hz_power)
                 : (int32_t)raw;
    return true;
}

bool openk4a_color_control_set(openk4a_color_t *color, k4a_color_control_command_t command, k4a_color_control_mode_t mode, int32_t value)
{
    if (color == NULL || (mode != K4A_COLOR_CONTROL_MODE_AUTO && mode != K4A_COLOR_CONTROL_MODE_MANUAL))
    {
        return false;
    }
    GUID set;
    ULONG id;
    if (!control_property(command, &set, &id))
    {
        return command == K4A_COLOR_CONTROL_AUTO_EXPOSURE_PRIORITY;
    }

    ULONG flags = OPENK4A_KS_FLAGS_MANUAL;
    if (mode == K4A_COLOR_CONTROL_MODE_AUTO)
    {
        if (command != K4A_COLOR_CONTROL_EXPOSURE_TIME_ABSOLUTE && command != K4A_COLOR_CONTROL_WHITEBALANCE)
        {
            openk4a_log(OPENK4A_LOG_ERROR, "colour control %d has no automatic mode", (int)command);
            return false;
        }
        flags = OPENK4A_KS_FLAGS_AUTO;
    }

    const LONG raw = command == K4A_COLOR_CONTROL_EXPOSURE_TIME_ABSOLUTE
                         ? exposure_to_exponent(value, color->use_60hz_power)
                         : (LONG)value;
    if (!ks_set(color, &set, id, raw, flags))
    {
        return false;
    }
    if (command == K4A_COLOR_CONTROL_POWERLINE_FREQUENCY)
    {
        color->use_60hz_power = value == 2;
    }
    return true;
}
