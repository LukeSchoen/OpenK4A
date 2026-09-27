/*=============================================================================
  The part of Media Foundation the colour camera needs, named rather than
  included.

  Media Foundation is a COM API, so what a caller needs is the exact order of
  the methods in each interface's vtable. These are that order, taken from the
  Windows SDK's own mfobjects.h, mfidl.h, mfreadwrite.h and ksproxy.h, with
  the GUIDs and property numbers read out of the same headers by
  tools/windows_constants.ps1. They are written here because neither compiler
  in this tree carries those headers, and because a build with no import
  libraries is the point.

  Only the methods this tree calls are used; the rest of each vtable is
  present because the order is what matters, not the list.
=============================================================================*/

#ifndef OPENK4A_MF_H
#define OPENK4A_MF_H

#include "openk4a_win.h"

typedef struct openk4a_mf_object openk4a_mf_object_t;
typedef struct openk4a_mf_attributes openk4a_mf_attributes_t;
typedef struct openk4a_mf_media_type openk4a_mf_media_type_t;
typedef struct openk4a_mf_media_buffer openk4a_mf_media_buffer_t;
typedef struct openk4a_mf_sample openk4a_mf_sample_t;
typedef struct openk4a_mf_activate openk4a_mf_activate_t;
typedef struct openk4a_mf_source_reader openk4a_mf_source_reader_t;
typedef struct openk4a_mf_ks_control openk4a_mf_ks_control_t;

/* IUnknown, the first three entries of every vtable. The self pointer is
 * void* rather than each interface's own type, because these are the entries
 * every other interface inherits: naming the base type here means a derived
 * interface's vtable can be used without a cast, and the layout is the same
 * either way. */
typedef struct
{
    HRESULT (*QueryInterface)(void *self, const GUID *riid, void **object);
    ULONG (*AddRef)(void *self);
    ULONG (*Release)(void *self);
} openk4a_mf_unknown_vtbl_t;

struct openk4a_mf_object
{
    const openk4a_mf_unknown_vtbl_t *vtable;
};

/* IMFAttributes. */
typedef struct
{
    openk4a_mf_unknown_vtbl_t unknown;
    HRESULT (*GetItem)(void *self, const GUID *key, void *value);
    HRESULT (*GetItemType)(void *self, const GUID *key, ULONG *type);
    HRESULT (*CompareItem)(void *self, const GUID *key, const void *value, BOOL *result);
    HRESULT (*Compare)(void *self, void *theirs, ULONG match_type, BOOL *result);
    HRESULT (*GetUINT32)(void *self, const GUID *key, UINT32 *value);
    HRESULT (*GetUINT64)(void *self, const GUID *key, UINT64 *value);
    HRESULT (*GetDouble)(void *self, const GUID *key, double *value);
    HRESULT (*GetGUID)(void *self, const GUID *key, GUID *value);
    HRESULT (*GetStringLength)(void *self, const GUID *key, UINT32 *length);
    HRESULT (*GetString)(void *self, const GUID *key, LPWSTR value, UINT32 size, UINT32 *length);
    HRESULT (*GetAllocatedString)(void *self, const GUID *key, LPWSTR *value, UINT32 *length);
    HRESULT (*GetBlobSize)(void *self, const GUID *key, UINT32 *size);
    HRESULT (*GetBlob)(void *self, const GUID *key, UINT8 *buffer, UINT32 size, UINT32 *blob_size);
    HRESULT (*GetAllocatedBlob)(void *self, const GUID *key, UINT8 **buffer, UINT32 *size);
    HRESULT (*GetUnknown)(void *self, const GUID *key, const GUID *riid, void **object);
    HRESULT (*SetItem)(void *self, const GUID *key, const void *value);
    HRESULT (*DeleteItem)(void *self, const GUID *key);
    HRESULT (*DeleteAllItems)(void *self);
    HRESULT (*SetUINT32)(void *self, const GUID *key, UINT32 value);
    HRESULT (*SetUINT64)(void *self, const GUID *key, UINT64 value);
    HRESULT (*SetDouble)(void *self, const GUID *key, double value);
    HRESULT (*SetGUID)(void *self, const GUID *key, const GUID *value);
    HRESULT (*SetString)(void *self, const GUID *key, LPCWSTR value);
    HRESULT (*SetBlob)(void *self, const GUID *key, const UINT8 *buffer, UINT32 size);
    HRESULT (*SetUnknown)(void *self, const GUID *key, void *object);
    HRESULT (*LockStore)(void *self);
    HRESULT (*UnlockStore)(void *self);
    HRESULT (*GetCount)(void *self, UINT32 *count);
    HRESULT (*GetItemByIndex)(void *self, UINT32 index, GUID *key, void *value);
    HRESULT (*CopyAllItems)(void *self, void *destination);
} openk4a_mf_attributes_vtbl_t;

struct openk4a_mf_attributes
{
    const openk4a_mf_attributes_vtbl_t *vtable;
};

/* IMFMediaType: IMFAttributes plus five. */
typedef struct
{
    openk4a_mf_attributes_vtbl_t attributes;
    HRESULT (*GetMajorType)(void *self, GUID *major_type);
    HRESULT (*IsCompressedFormat)(void *self, BOOL *compressed);
    HRESULT (*IsEqual)(void *self, void *other, DWORD *flags);
    HRESULT (*GetRepresentation)(void *self, GUID representation, void **value);
    HRESULT (*FreeRepresentation)(void *self, GUID representation, void *value);
} openk4a_mf_media_type_vtbl_t;

struct openk4a_mf_media_type
{
    const openk4a_mf_media_type_vtbl_t *vtable;
};

/* IMFMediaBuffer and the 2D extension. */
typedef struct
{
    openk4a_mf_unknown_vtbl_t unknown;
    HRESULT (*Lock)(void *self, BYTE **buffer, DWORD *max_length, DWORD *current_length);
    HRESULT (*Unlock)(void *self);
    HRESULT (*GetCurrentLength)(void *self, DWORD *length);
    HRESULT (*SetCurrentLength)(void *self, DWORD length);
    HRESULT (*GetMaxLength)(void *self, DWORD *length);
    HRESULT (*Lock2D)(void *self, BYTE **scanline0, LONG *pitch);
    HRESULT (*Unlock2D)(void *self);
    HRESULT (*GetScanline0AndPitch)(void *self, BYTE **scanline0, LONG *pitch);
    HRESULT (*IsContiguousFormat)(void *self, BOOL *contiguous);
    HRESULT (*GetContiguousLength)(void *self, DWORD *length);
    HRESULT (*ContiguousCopyTo)(void *self, BYTE *destination, DWORD size);
    HRESULT (*ContiguousCopyFrom)(void *self, const BYTE *source, DWORD size);
    HRESULT (*Lock2DSize)(void *self, DWORD flags, BYTE **scanline0, LONG *pitch, BYTE **buffer,
                          DWORD *length);
    HRESULT (*Copy2DTo)(void *self, void *destination);
    HRESULT (*Copy2DFrom)(void *self, void *source);
} openk4a_mf_media_buffer_vtbl_t;

struct openk4a_mf_media_buffer
{
    const openk4a_mf_media_buffer_vtbl_t *vtable;
};

#define OPENK4A_MF2DBUFFER_LOCKFLAGS_READ 0x1
#define OPENK4A_MF2DBUFFER_LOCKFLAGS_WRITE 0x2
#define OPENK4A_MF2DBUFFER_LOCKFLAGS_READWRITE 0x3

/* IMFSample: IMFAttributes plus fourteen. */
typedef struct
{
    openk4a_mf_attributes_vtbl_t attributes;
    HRESULT (*GetSampleFlags)(void *self, DWORD *flags);
    HRESULT (*SetSampleFlags)(void *self, DWORD flags);
    HRESULT (*GetSampleTime)(void *self, LONGLONG *time);
    HRESULT (*SetSampleTime)(void *self, LONGLONG time);
    HRESULT (*GetSampleDuration)(void *self, LONGLONG *duration);
    HRESULT (*SetSampleDuration)(void *self, LONGLONG duration);
    HRESULT (*GetBufferCount)(void *self, DWORD *count);
    HRESULT (*GetBufferByIndex)(void *self, DWORD index, openk4a_mf_media_buffer_t **buffer);
    HRESULT (*ConvertToContiguousBuffer)(void *self, openk4a_mf_media_buffer_t **buffer);
    HRESULT (*AddBuffer)(void *self, openk4a_mf_media_buffer_t *buffer);
    HRESULT (*RemoveBufferByIndex)(void *self, DWORD index);
    HRESULT (*RemoveAllBuffers)(void *self);
    HRESULT (*GetTotalLength)(void *self, DWORD *length);
    HRESULT (*CopyToBuffer)(void *self, openk4a_mf_media_buffer_t *buffer);
} openk4a_mf_sample_vtbl_t;

struct openk4a_mf_sample
{
    const openk4a_mf_sample_vtbl_t *vtable;
};

/* IMFActivate: IMFAttributes plus three. */
typedef struct
{
    openk4a_mf_attributes_vtbl_t attributes;
    HRESULT (*ActivateObject)(void *self, const GUID *riid, void **object);
    HRESULT (*ShutdownObject)(void *self);
    HRESULT (*DetachObject)(void *self);
} openk4a_mf_activate_vtbl_t;

struct openk4a_mf_activate
{
    const openk4a_mf_activate_vtbl_t *vtable;
};

/* IMFSourceReader: IUnknown plus ten. It is the one interface here that does
 * not inherit IMFAttributes, which is worth knowing before counting slots. */
typedef struct
{
    openk4a_mf_unknown_vtbl_t unknown;
    HRESULT (*GetStreamSelection)(void *self, DWORD index, BOOL *selected);
    HRESULT (*SetStreamSelection)(void *self, DWORD index, BOOL selected);
    HRESULT (*GetNativeMediaType)(void *self, DWORD index, DWORD type_index, openk4a_mf_media_type_t **type);
    HRESULT (*GetCurrentMediaType)(void *self, DWORD index, openk4a_mf_media_type_t **type);
    HRESULT (*SetCurrentMediaType)(void *self, DWORD index, DWORD *reserved, openk4a_mf_media_type_t *type);
    HRESULT (*SetCurrentPosition)(void *self, const GUID *format, const void *position);
    HRESULT (*ReadSample)(void *self, DWORD index, DWORD flags, DWORD *actual_index,
                          DWORD *stream_flags, LONGLONG *timestamp, openk4a_mf_sample_t **sample);
    HRESULT (*Flush)(void *self, DWORD index);
    HRESULT (*GetServiceForStream)(void *self, DWORD index, const GUID *service, const GUID *riid,
                                   void **object);
    HRESULT (*GetPresentationAttribute)(void *self, DWORD index, const GUID *attribute, void *value);
} openk4a_mf_source_reader_vtbl_t;

struct openk4a_mf_source_reader
{
    const openk4a_mf_source_reader_vtbl_t *vtable;
};

#define OPENK4A_MF_SOURCE_READER_FIRST_VIDEO_STREAM 0xFFFFFFFC
#define OPENK4A_MF_SOURCE_READER_MEDIASOURCE 0xFFFFFFFF
#define OPENK4A_MF_SOURCE_READER_ALL_STREAMS 0xFFFFFFFE

/* Where a media source's own methods begin; only Release is called on one. */
struct openk4a_mf_media_source
{
    const void *vtable;
};

/* IKsControl: IUnknown plus three, which is how a camera's controls are read
   and written. */
typedef struct
{
    openk4a_mf_unknown_vtbl_t unknown;
    HRESULT (*KsProperty)(void *self, void *property, ULONG property_length, void *data,
                          ULONG data_length, ULONG *bytes_returned);
    HRESULT (*KsMethod)(void *self, void *method, ULONG method_length, void *data, ULONG data_length,
                        ULONG *bytes_returned);
    HRESULT (*KsEvent)(void *self, void *event, ULONG event_length, void *data, ULONG data_length,
                       ULONG *bytes_returned);
} openk4a_mf_ks_control_vtbl_t;

struct openk4a_mf_ks_control
{
    const openk4a_mf_ks_control_vtbl_t *vtable;
};

/*=============================================================================
  The entry points, from mfplat.dll, mfreadwrite.dll and mf.dll.
=============================================================================*/

typedef struct
{
    HRESULT (*Startup)(ULONG version, DWORD flags);
    HRESULT (*Shutdown)(void);
    HRESULT (*CreateAttributes)(openk4a_mf_attributes_t **attributes, UINT32 size);
    HRESULT (*CreateMediaType)(openk4a_mf_media_type_t **type);
    HRESULT (*CreateSourceReaderFromMediaSource)(struct openk4a_mf_media_source *source, openk4a_mf_attributes_t *attributes,
                                                 openk4a_mf_source_reader_t **reader);
    HRESULT (*EnumDeviceSources)(openk4a_mf_attributes_t *attributes, openk4a_mf_activate_t ***devices, UINT32 *count);
    HRESULT (*CreateDeviceSource)(openk4a_mf_activate_t *activate, struct openk4a_mf_media_source **source);
    bool loaded;
} openk4a_mf_t;

extern openk4a_mf_t openk4a_mf;

/* Loads Media Foundation. False means no colour camera, and the rest of the
 * tree is unaffected. */
bool openk4a_mf_load(void);

/* The same two helpers the SDK's own code uses for the size and rate
 * attributes, which Media Foundation stores packed into a UINT64. */
static inline HRESULT openk4a_mf_get_size(openk4a_mf_media_type_t *type, const GUID *key, UINT32 *width, UINT32 *height)
{
    UINT64 packed = 0;
    const HRESULT hr = type->vtable->attributes.GetUINT64(type, key, &packed);
    if (hr < 0)
    {
        return hr;
    }
    *width = (UINT32)(packed >> 32);
    *height = (UINT32)(packed & 0xFFFFFFFFu);
    return hr;
}

static inline HRESULT openk4a_mf_get_ratio(openk4a_mf_media_type_t *type,
                                       const GUID *key,
                                       UINT32 *numerator,
                                       UINT32 *denominator)
{
    UINT64 packed = 0;
    const HRESULT hr = type->vtable->attributes.GetUINT64(type, key, &packed);
    if (hr < 0)
    {
        return hr;
    }
    *numerator = (UINT32)(packed >> 32);
    *denominator = (UINT32)(packed & 0xFFFFFFFFu);
    return hr;
}

/*=============================================================================
  The constants, from the SDK's headers by way of
  tools/windows_constants.ps1.
=============================================================================*/

extern const GUID OPENK4A_MF_DEVSOURCE_ATTRIBUTE_SOURCE_TYPE;
extern const GUID OPENK4A_MF_DEVSOURCE_ATTRIBUTE_SOURCE_TYPE_VIDCAP_GUID;
extern const GUID OPENK4A_MF_DEVSOURCE_ATTRIBUTE_SOURCE_TYPE_VIDCAP_SYMBOLIC_LINK;
extern const GUID OPENK4A_MF_MT_FRAME_SIZE;
extern const GUID OPENK4A_MF_MT_FRAME_RATE;
extern const GUID OPENK4A_MF_MT_SUBTYPE;
extern const GUID OPENK4A_MF_MT_MAJOR_TYPE;
extern const GUID OPENK4A_MF_SOURCE_READER_ASYNC_CALLBACK;
extern const GUID OPENK4A_MF_SOURCE_READER_ENABLE_ADVANCED_VIDEO_PROCESSING;
extern const GUID OPENK4A_MF_XVP_DISABLE_FRC;
extern const GUID OPENK4A_MFMediaType_Video;
extern const GUID OPENK4A_MFVideoFormat_MJPG;
extern const GUID OPENK4A_MFVideoFormat_NV12;
extern const GUID OPENK4A_MFVideoFormat_YUY2;
extern const GUID OPENK4A_MFVideoFormat_ARGB32;
extern const GUID OPENK4A_MF_CAPTURE_METADATA_EXPOSURE_TIME;
extern const GUID OPENK4A_MF_CAPTURE_METADATA_WHITEBALANCE;
extern const GUID OPENK4A_MF_CAPTURE_METADATA_ISO_SPEED;
extern const GUID OPENK4A_MF_CAPTURE_METADATA_FRAME_RAWSTREAM;
extern const GUID OPENK4A_MFSampleExtension_CaptureMetadata;
extern const GUID OPENK4A_MFSampleExtension_DeviceTimestamp;

extern const GUID OPENK4A_PROPSETID_VIDCAP_CAMERACONTROL;
extern const GUID OPENK4A_PROPSETID_VIDCAP_VIDEOPROCAMP;
extern const GUID OPENK4A_IID_IKSCONTROL;

/* The kernel streaming property numbers, from ksmedia.h and ks.h. */
#define OPENK4A_KSPROPERTY_TYPE_GET 0x00000001
#define OPENK4A_KSPROPERTY_TYPE_SET 0x00000002
#define OPENK4A_KSPROPERTY_TYPE_BASICSUPPORT 0x00000200
#define OPENK4A_KSPROPERTY_TYPE_DEFAULTVALUES 0x00010000

#define OPENK4A_KS_FLAGS_AUTO 0x0001
#define OPENK4A_KS_FLAGS_MANUAL 0x0002

#define OPENK4A_KSPROPERTY_CAMERACONTROL_EXPOSURE 4

#define OPENK4A_KSPROPERTY_VIDEOPROCAMP_BRIGHTNESS 0
#define OPENK4A_KSPROPERTY_VIDEOPROCAMP_CONTRAST 1
#define OPENK4A_KSPROPERTY_VIDEOPROCAMP_SATURATION 3
#define OPENK4A_KSPROPERTY_VIDEOPROCAMP_SHARPNESS 4
#define OPENK4A_KSPROPERTY_VIDEOPROCAMP_WHITEBALANCE 7
#define OPENK4A_KSPROPERTY_VIDEOPROCAMP_BACKLIGHT_COMPENSATION 8
#define OPENK4A_KSPROPERTY_VIDEOPROCAMP_GAIN 9
#define OPENK4A_KSPROPERTY_VIDEOPROCAMP_POWERLINE_FREQUENCY 13

/* The two property structures a camera control is read or written with. */
typedef struct
{
    GUID set;
    ULONG id;
    ULONG flags;
} openk4a_ks_property_t;

typedef struct
{
    openk4a_ks_property_t property;
    LONG value;
    ULONG flags;
    ULONG capabilities;
} openk4a_ks_camera_control_t;

typedef struct
{
    ULONG access_flags;
    ULONG description_size;
    openk4a_ks_property_t property_type_set;
    ULONG members_list_count;
    ULONG reserved;
} openk4a_ks_property_description_t;

typedef struct
{
    ULONG members_flags;
    ULONG members_size;
    ULONG members_count;
    ULONG flags;
} openk4a_ks_members_header_t;

typedef struct
{
    LONG signed_minimum;
    LONG signed_maximum;
} openk4a_ks_bounds_long_t;

typedef struct
{
    ULONG stepping_delta;
    ULONG reserved;
    openk4a_ks_bounds_long_t bounds;
} openk4a_ks_stepping_long_t;

/* What one property's membership list comes back as: the description, the
 * members header, then the stepping range. */
typedef struct
{
    openk4a_ks_property_description_t description;
    openk4a_ks_members_header_t header;
    openk4a_ks_stepping_long_t step;
} openk4a_ks_member_list_t;

typedef struct
{
    openk4a_ks_property_description_t description;
    openk4a_ks_members_header_t header;
    LONG value;
} openk4a_ks_default_value_t;

/* The metadata Media Foundation attaches to a colour sample, in the layout
 * ksmetadata.h gives it. */
typedef struct
{
    UINT32 metadata_id;
    UINT32 size;
} openk4a_ks_metadata_header_t;

typedef struct
{
    openk4a_ks_metadata_header_t header;
    UINT32 flags;
    UINT32 reserved;
    UINT64 frame_pts;
    UINT32 pts_reference;
    UINT64 usb_sof_seq;
    UINT64 usb_sof_pts;
} openk4a_ks_frame_align_t;

#define OPENK4A_METADATA_ID_FRAME_ALIGN_INFO 0x80000001

#endif /* OPENK4A_MF_H */
