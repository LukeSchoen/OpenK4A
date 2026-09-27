/*=============================================================================
  The Windows side of OpenK4A, named rather than included.

  windows.h is on both compilers (clang finds the Windows SDK's, cpc carries
  its own), so the fundamentals - HANDLE, GUID, OVERLAPPED, the kernel32
  prototypes - come from there. Everything else this tree calls is declared
  here: setupapi and winusb, cfgmgr32's device properties, and the part of
  Media Foundation the colour camera needs in openk4a_mf.h.

  The small libraries are resolved by hand at run time for one reason: a tree
  that imports nothing it does not have to can be built by any compiler with
  any runtime, which is what lets build_cpc.cmd and build_clang.cmd compile
  the same sources and produce the same program.
=============================================================================*/

#ifndef OPENK4A_WIN_H
#define OPENK4A_WIN_H

#ifndef WIN32_LEAN_AND_MEAN
#define WIN32_LEAN_AND_MEAN
#endif
#include <windows.h>

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#ifndef GUID_DEFINED
typedef struct _OPENK4A_GUID
{
    uint32_t Data1;
    uint16_t Data2;
    uint16_t Data3;
    uint8_t Data4[8];
} GUID;
#define GUID_DEFINED
#endif

/*-----------------------------------------------------------------------------
  The entry points, resolved once at start-up. A field is NULL when the
  library it belongs to is not on the machine, which is how a build can run
  without Media Foundation and only lose the colour camera.
---------------------------------------------------------------------------*/

/* setupapi.dll */
typedef struct
{
    DWORD cbSize;
    GUID InterfaceClassGuid;
    DWORD Flags;
    ULONG_PTR Reserved;
} openk4a_sp_interface_data_t;

typedef struct
{
    DWORD cbSize;
    char DevicePath[2]; /* the rest of the path follows this header */
} openk4a_sp_interface_detail_t;

#define OPENK4A_DIGCF_PRESENT 0x00000002
#define OPENK4A_DIGCF_DEVICEINTERFACE 0x00000010

/* winusb.dll */
typedef void *openk4a_winusb_handle_t;

typedef struct
{
    UCHAR bLength;
    UCHAR bDescriptorType;
    UCHAR bInterfaceNumber;
    UCHAR bAlternateSetting;
    UCHAR bNumEndpoints;
    UCHAR bInterfaceClass;
    UCHAR bInterfaceSubClass;
    UCHAR bInterfaceProtocol;
    UCHAR iInterface;
} openk4a_usb_interface_descriptor_t;

typedef struct
{
    int PipeType; /* USBD_PIPE_TYPE */
    UCHAR PipeId;
    USHORT MaximumPacketSize;
    UCHAR Interval;
} openk4a_winusb_pipe_information_t;

#define OPENK4A_PIPE_TRANSFER_TIMEOUT 0x03
#define OPENK4A_SHORT_PACKET_TERMINATE 0x01
#define OPENK4A_AUTO_CLEAR_STALL 0x02
#define OPENK4A_RAW_IO 0x07

/* cfgmgr32.dll: device properties, for pairing a camera with its container. */
#define OPENK4A_CR_SUCCESS 0x00000000
#define OPENK4A_CR_BUFFER_SMALL 0x0000001A
#define OPENK4A_CM_LOCATE_DEVNODE_NORMAL 0x00000000

typedef struct
{
    GUID fmtid;
    ULONG pid;
} openk4a_devpropkey_t;

typedef struct
{
    void *get_class_devs;      /* SetupDiGetClassDevsA */
    void *enum_interfaces;     /* SetupDiEnumDeviceInterfaces */
    void *get_detail;          /* SetupDiGetDeviceInterfaceDetailA */
    void *destroy_list;        /* SetupDiDestroyDeviceInfoList */
    void *winusb_initialize;   /* WinUsb_Initialize */
    void *winusb_free;         /* WinUsb_Free */
    void *query_interface;     /* WinUsb_QueryInterfaceSettings */
    void *query_pipe;          /* WinUsb_QueryPipe */
    void *set_pipe_policy;     /* WinUsb_SetPipePolicy */
    void *read_pipe;           /* WinUsb_ReadPipe */
    void *write_pipe;          /* WinUsb_WritePipe */
    void *abort_pipe;          /* WinUsb_AbortPipe */
    void *get_device_interface_property; /* CM_Get_Device_Interface_PropertyW */
    void *locate_devnode;      /* CM_Locate_DevNodeW */
    void *get_devnode_property; /* CM_Get_DevNode_PropertyW */
    void *co_initialize_ex;    /* CoInitializeEx, from ole32 */
    void *co_task_mem_free;    /* CoTaskMemFree, for what COM hands out */
    bool loaded;
} openk4a_win_t;

extern openk4a_win_t openk4a_win;

/* True once the WinUSB side is there; false means no camera can be opened. */
bool openk4a_win_init(void);

/* The device interface classes a camera can register under, in the order they
 * are tried: WinUSB's own, the standard USB device interface, and the one
 * libusb installs. */
extern const GUID OPENK4A_GUID_DEVINTERFACE_WINUSB;
extern const GUID OPENK4A_GUID_DEVINTERFACE_USB_DEVICE;
extern const GUID OPENK4A_GUID_DEVINTERFACE_LIBUSB;

/* DEVPKEY_Device_InstanceId and DEVPKEY_Device_ContainerId, whose definitions
 * are eight numbers and a page of macro in the SDK's devpkey.h. */
extern const openk4a_devpropkey_t OPENK4A_DEVPKEY_DEVICE_INSTANCE_ID;
extern const openk4a_devpropkey_t OPENK4A_DEVPKEY_DEVICE_CONTAINER_ID;

/* The container a device instance belongs to, as a GUID. Two functions of
 * one camera - its depth processor and its colour camera - share it, which is
 * how they are paired when more than one camera is attached. */
bool openk4a_win_container_id_from_instance(const wchar_t *instance_id, GUID *container_id);
bool openk4a_win_instance_from_interface(const wchar_t *interface_path, wchar_t *instance, size_t instance_count);

/* The kernel32 calls the streaming code wants and the dev compiler's headers
 * are not guaranteed to carry. */
typedef BOOL(WINAPI *openk4a_cancel_io_ex_t)(HANDLE, LPOVERLAPPED);
extern openk4a_cancel_io_ex_t openk4a_cancel_io_ex;

#endif /* OPENK4A_WIN_H */
