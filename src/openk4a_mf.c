/*=============================================================================
  Media Foundation, and the constants it is addressed by.

  The GUIDs and property numbers below are the Windows SDK's own, read out of
  its headers by tools/windows_constants.ps1. FOURCC() and the media type
  GUID template are the two macros in mfapi.h that produce them, spelled out.
=============================================================================*/

#include "openk4a.h"
#include "openk4a_mf.h"

openk4a_mf_t openk4a_mf;

/* mfapi.h: DEFINE_MEDIATYPE_GUID(name, format) is a GUID whose first field is
 * the format code and whose tail is the same eight bytes for every one. */
#define OPENK4A_FOURCC(a, b, c, d) \
    ((uint32_t)(a) | ((uint32_t)(b) << 8) | ((uint32_t)(c) << 16) | ((uint32_t)(d) << 24))
#define OPENK4A_MEDIATYPE_GUID(format) \
    { (format), 0x0000, 0x0010, { 0x80, 0x00, 0x00, 0xaa, 0x00, 0x38, 0x9b, 0x71 } }

const GUID OPENK4A_MF_DEVSOURCE_ATTRIBUTE_SOURCE_TYPE = {
    0xc60ac5fe, 0x252a, 0x478f, { 0xa0, 0xef, 0xbc, 0x8f, 0xa5, 0xf7, 0xca, 0xd3 }
};
const GUID OPENK4A_MF_DEVSOURCE_ATTRIBUTE_SOURCE_TYPE_VIDCAP_GUID = {
    0x8ac3587a, 0x4ae7, 0x42d8, { 0x99, 0xe0, 0x0a, 0x60, 0x13, 0xee, 0xf9, 0x0f }
};
const GUID OPENK4A_MF_DEVSOURCE_ATTRIBUTE_SOURCE_TYPE_VIDCAP_SYMBOLIC_LINK = {
    0x58f0aad8, 0x22bf, 0x4f8a, { 0xbb, 0x3d, 0xd2, 0xc4, 0x97, 0x8c, 0x6e, 0x2f }
};
const GUID OPENK4A_MF_MT_FRAME_SIZE = {
    0x1652c33d, 0xd6b2, 0x4012, { 0xb8, 0x34, 0x72, 0x03, 0x08, 0x49, 0xa3, 0x7d }
};
const GUID OPENK4A_MF_MT_FRAME_RATE = {
    0xc459a2e8, 0x3d2c, 0x4e44, { 0xb1, 0x32, 0xfe, 0xe5, 0x15, 0x6c, 0x7b, 0xb0 }
};
const GUID OPENK4A_MF_MT_SUBTYPE = {
    0xf7e34c9a, 0x42e8, 0x4714, { 0xb7, 0x4b, 0xcb, 0x29, 0xd7, 0x2c, 0x35, 0xe5 }
};
const GUID OPENK4A_MF_MT_MAJOR_TYPE = {
    0x48eba18e, 0xf8c9, 0x4687, { 0xbf, 0x11, 0x0a, 0x74, 0xc9, 0xf9, 0x6a, 0x8f }
};
const GUID OPENK4A_MF_SOURCE_READER_ASYNC_CALLBACK = {
    0x1e3dbeac, 0xbb43, 0x4c35, { 0xb5, 0x07, 0xcd, 0x64, 0x44, 0x64, 0xc9, 0x65 }
};
const GUID OPENK4A_MF_SOURCE_READER_ENABLE_ADVANCED_VIDEO_PROCESSING = {
    0x0f81da2c, 0xb537, 0x4672, { 0xa8, 0xb2, 0xa6, 0x81, 0xb1, 0x73, 0x07, 0xa3 }
};
const GUID OPENK4A_MF_XVP_DISABLE_FRC = {
    0x2c0afa19, 0x7a97, 0x4d5a, { 0x9e, 0xe8, 0x16, 0xd4, 0xfc, 0x51, 0x8d, 0x8c }
};
const GUID OPENK4A_MFMediaType_Video = {
    0x73646976, 0x0000, 0x0010, { 0x80, 0x00, 0x00, 0xaa, 0x00, 0x38, 0x9b, 0x71 }
};
const GUID OPENK4A_MFVideoFormat_MJPG = OPENK4A_MEDIATYPE_GUID(OPENK4A_FOURCC('M', 'J', 'P', 'G'));
const GUID OPENK4A_MFVideoFormat_NV12 = OPENK4A_MEDIATYPE_GUID(OPENK4A_FOURCC('N', 'V', '1', '2'));
const GUID OPENK4A_MFVideoFormat_YUY2 = OPENK4A_MEDIATYPE_GUID(OPENK4A_FOURCC('Y', 'U', 'Y', '2'));
/* D3DFMT_A8R8G8B8. */
const GUID OPENK4A_MFVideoFormat_ARGB32 = OPENK4A_MEDIATYPE_GUID(21);
const GUID OPENK4A_MF_CAPTURE_METADATA_EXPOSURE_TIME = {
    0x16b9ae99, 0xcd84, 0x4063, { 0x87, 0x9d, 0xa2, 0x8c, 0x76, 0x33, 0x72, 0x9e }
};
const GUID OPENK4A_MF_CAPTURE_METADATA_WHITEBALANCE = {
    0xc736fd77, 0x0fb9, 0x4e2e, { 0x97, 0xa2, 0xfc, 0xd4, 0x90, 0x73, 0x9e, 0xe9 }
};
const GUID OPENK4A_MF_CAPTURE_METADATA_ISO_SPEED = {
    0xe528a68f, 0xb2e3, 0x44fe, { 0x8b, 0x65, 0x07, 0xbf, 0x4b, 0x5a, 0x13, 0xff }
};
const GUID OPENK4A_MF_CAPTURE_METADATA_FRAME_RAWSTREAM = {
    0x9252077b, 0x2680, 0x49b9, { 0xae, 0x02, 0xb1, 0x90, 0x75, 0x97, 0x3b, 0x70 }
};
const GUID OPENK4A_MFSampleExtension_CaptureMetadata = {
    0x2ebe23a8, 0xfaf5, 0x444a, { 0xa6, 0xa2, 0xeb, 0x81, 0x08, 0x80, 0xab, 0x5d }
};
const GUID OPENK4A_MFSampleExtension_DeviceTimestamp = {
    0x8f3e35e7, 0x2dcd, 0x4887, { 0x86, 0x22, 0x2a, 0x58, 0xba, 0xa6, 0x52, 0xb0 }
};

const GUID OPENK4A_PROPSETID_VIDCAP_CAMERACONTROL = {
    0xc6e13370, 0x30ac, 0x11d0, { 0xa1, 0x8c, 0x00, 0xa0, 0xc9, 0x11, 0x89, 0x56 }
};
const GUID OPENK4A_PROPSETID_VIDCAP_VIDEOPROCAMP = {
    0xc6e13360, 0x30ac, 0x11d0, { 0xa1, 0x8c, 0x00, 0xa0, 0xc9, 0x11, 0x89, 0x56 }
};
const GUID OPENK4A_IID_IKSCONTROL = {
    0x28f54685, 0x06fd, 0x11d2, { 0xb2, 0x7a, 0x00, 0xa0, 0xc9, 0x22, 0x31, 0x96 }
};

bool openk4a_mf_load(void)
{
    if (openk4a_mf.loaded)
    {
        return true;
    }

    HMODULE mfplat = LoadLibraryA("mfplat.dll");
    HMODULE mfreadwrite = LoadLibraryA("mfreadwrite.dll");
    HMODULE mf = LoadLibraryA("mf.dll");
    if (mfplat == NULL || mfreadwrite == NULL || mf == NULL)
    {
        openk4a_log(OPENK4A_LOG_ERROR, "Media Foundation is not on this machine; there is no colour camera");
        return false;
    }

    openk4a_mf.Startup = (HRESULT (*)(ULONG, DWORD))(void *)GetProcAddress(mfplat, "MFStartup");
    openk4a_mf.Shutdown = (HRESULT (*)(void))(void *)GetProcAddress(mfplat, "MFShutdown");
    openk4a_mf.CreateAttributes =
        (HRESULT (*)(openk4a_mf_attributes_t **, UINT32))(void *)GetProcAddress(mfplat, "MFCreateAttributes");
    openk4a_mf.CreateMediaType = (HRESULT (*)(openk4a_mf_media_type_t **))(void *)GetProcAddress(mfplat, "MFCreateMediaType");
    openk4a_mf.CreateSourceReaderFromMediaSource =
        (HRESULT (*)(struct openk4a_mf_media_source *, openk4a_mf_attributes_t *, openk4a_mf_source_reader_t **))(void *)
            GetProcAddress(mfreadwrite, "MFCreateSourceReaderFromMediaSource");
    openk4a_mf.EnumDeviceSources =
        (HRESULT (*)(openk4a_mf_attributes_t *, openk4a_mf_activate_t ***, UINT32 *))(void *)GetProcAddress(mf,
                                                                                                  "MFEnumDeviceSources");
    openk4a_mf.CreateDeviceSource =
        (HRESULT (*)(openk4a_mf_activate_t *, struct openk4a_mf_media_source **))(void *)GetProcAddress(mf,
                                                                                                "MFCreateDeviceSource");

    if (openk4a_mf.Startup == NULL || openk4a_mf.CreateAttributes == NULL || openk4a_mf.CreateMediaType == NULL ||
        openk4a_mf.CreateSourceReaderFromMediaSource == NULL || openk4a_mf.EnumDeviceSources == NULL ||
        openk4a_mf.CreateDeviceSource == NULL)
    {
        openk4a_log(OPENK4A_LOG_ERROR, "the Media Foundation entry points are not all there");
        return false;
    }

    openk4a_mf.loaded = true;
    return true;
}
