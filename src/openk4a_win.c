/*=============================================================================
  The Windows entry points OpenK4A calls, resolved once.

  Nothing here imports a library: setupapi, winusb and cfgmgr32 are loaded by
  name and their functions are looked up, so a build of this tree has no
  import library list to get wrong and the same sources build under either
  compiler. A missing library is reported once, as a sentence, rather than as
  a link error.
=============================================================================*/

#include "openk4a.h"

openk4a_win_t openk4a_win;
openk4a_cancel_io_ex_t openk4a_cancel_io_ex;

/* The WinUSB device interface class. The standard USB interface and libusb's
 * own are registered for the same device, and are tried after it in case a
 * different driver binding turns up. */
const GUID OPENK4A_GUID_DEVINTERFACE_WINUSB = {
    0xcb898bb2, 0xea74, 0x4549, { 0x88, 0xdb, 0xa8, 0xb0, 0x8a, 0x72, 0xa6, 0x33 }
};
const GUID OPENK4A_GUID_DEVINTERFACE_USB_DEVICE = {
    0xa5dcbf10, 0x6530, 0x11d2, { 0x90, 0x1f, 0x00, 0xc0, 0x4f, 0xb9, 0x51, 0xed }
};
const GUID OPENK4A_GUID_DEVINTERFACE_LIBUSB = {
    0xdee824ef, 0x729b, 0x4a0e, { 0x9c, 0x14, 0xb7, 0x11, 0x7d, 0x33, 0xa8, 0x17 }
};

/* devpkey.h, DEFINE_DEVPROPKEY(DEVPKEY_Device_InstanceId, 0x78c34fc8, ...). */
const openk4a_devpropkey_t OPENK4A_DEVPKEY_DEVICE_INSTANCE_ID = {
    { 0x78c34fc8, 0x104a, 0x4aca, { 0x9e, 0xa4, 0x52, 0x4d, 0x52, 0x99, 0x6e, 0x57 } }, 256
};
const openk4a_devpropkey_t OPENK4A_DEVPKEY_DEVICE_CONTAINER_ID = {
    { 0x8c7ed206, 0x3f8a, 0x4827, { 0xb3, 0xab, 0xae, 0x9e, 0x1f, 0xae, 0xfc, 0x6c } }, 2
};

bool openk4a_win_init(void)
{
    if (openk4a_win.loaded)
    {
        return true;
    }

    HMODULE setupapi = LoadLibraryA("setupapi.dll");
    HMODULE winusb = LoadLibraryA("winusb.dll");
    HMODULE cfgmgr32 = LoadLibraryA("cfgmgr32.dll");
    if (setupapi == NULL || winusb == NULL)
    {
        openk4a_log(OPENK4A_LOG_ERROR, "setupapi.dll and winusb.dll are both needed");
        return false;
    }

    openk4a_win.get_class_devs = (void *)GetProcAddress(setupapi, "SetupDiGetClassDevsA");
    openk4a_win.enum_interfaces = (void *)GetProcAddress(setupapi, "SetupDiEnumDeviceInterfaces");
    openk4a_win.get_detail = (void *)GetProcAddress(setupapi, "SetupDiGetDeviceInterfaceDetailA");
    openk4a_win.destroy_list = (void *)GetProcAddress(setupapi, "SetupDiDestroyDeviceInfoList");

    openk4a_win.winusb_initialize = (void *)GetProcAddress(winusb, "WinUsb_Initialize");
    openk4a_win.winusb_free = (void *)GetProcAddress(winusb, "WinUsb_Free");
    openk4a_win.query_interface = (void *)GetProcAddress(winusb, "WinUsb_QueryInterfaceSettings");
    openk4a_win.query_pipe = (void *)GetProcAddress(winusb, "WinUsb_QueryPipe");
    openk4a_win.set_pipe_policy = (void *)GetProcAddress(winusb, "WinUsb_SetPipePolicy");
    openk4a_win.read_pipe = (void *)GetProcAddress(winusb, "WinUsb_ReadPipe");
    openk4a_win.write_pipe = (void *)GetProcAddress(winusb, "WinUsb_WritePipe");
    openk4a_win.abort_pipe = (void *)GetProcAddress(winusb, "WinUsb_AbortPipe");

    if (cfgmgr32 != NULL)
    {
        openk4a_win.get_device_interface_property =
            (void *)GetProcAddress(cfgmgr32, "CM_Get_Device_Interface_PropertyW");
        openk4a_win.locate_devnode = (void *)GetProcAddress(cfgmgr32, "CM_Locate_DevNodeW");
        openk4a_win.get_devnode_property = (void *)GetProcAddress(cfgmgr32, "CM_Get_DevNode_PropertyW");
    }

    openk4a_cancel_io_ex = (openk4a_cancel_io_ex_t)(void *)GetProcAddress(GetModuleHandleA("kernel32.dll"), "CancelIoEx");

    /* COM's own two entry points: an apartment for Media Foundation, and the
     * allocator that frees the arrays COM hands back. */
    HMODULE ole32 = LoadLibraryA("ole32.dll");
    if (ole32 != NULL)
    {
        openk4a_win.co_initialize_ex = (void *)GetProcAddress(ole32, "CoInitializeEx");
        openk4a_win.co_task_mem_free = (void *)GetProcAddress(ole32, "CoTaskMemFree");
    }

    if (openk4a_win.read_pipe == NULL || openk4a_win.write_pipe == NULL || openk4a_win.winusb_initialize == NULL ||
        openk4a_win.get_class_devs == NULL || openk4a_win.enum_interfaces == NULL || openk4a_win.get_detail == NULL)
    {
        openk4a_log(OPENK4A_LOG_ERROR, "the WinUSB entry points are not all there");
        return false;
    }

    openk4a_win.loaded = true;
    return true;
}

/*-----------------------------------------------------------------------------
  Device properties: which node an interface belongs to, and which container
  that node belongs to. The container id is what pairs a camera's depth
  interface with its colour one when there is more than one camera attached.
---------------------------------------------------------------------------*/

bool openk4a_win_instance_from_interface(const wchar_t *interface_path, wchar_t *instance, size_t instance_count)
{
    if (openk4a_win.get_device_interface_property == NULL || interface_path == NULL || instance == NULL)
    {
        return false;
    }

    typedef LONG (*get_interface_property_t)(const wchar_t *, const openk4a_devpropkey_t *, ULONG *, void *, ULONG *,
                                             ULONG);
    get_interface_property_t get_property =
        (get_interface_property_t)openk4a_win.get_device_interface_property;

    ULONG bytes = (ULONG)(instance_count * sizeof(wchar_t));
    ULONG type = 0;
    if (get_property(interface_path, &OPENK4A_DEVPKEY_DEVICE_INSTANCE_ID, &type, instance, &bytes, 0) != OPENK4A_CR_SUCCESS)
    {
        return false;
    }
    instance[instance_count - 1] = L'\0';
    return true;
}

bool openk4a_win_container_id_from_instance(const wchar_t *instance_id, GUID *container_id)
{
    if (openk4a_win.locate_devnode == NULL || openk4a_win.get_devnode_property == NULL || instance_id == NULL ||
        container_id == NULL)
    {
        return false;
    }

    typedef LONG (*locate_devnode_t)(void **, const wchar_t *, ULONG);
    typedef LONG (*get_devnode_property_t)(void *, const openk4a_devpropkey_t *, ULONG *, void *, ULONG *, ULONG);

    locate_devnode_t locate = (locate_devnode_t)openk4a_win.locate_devnode;
    get_devnode_property_t get_property = (get_devnode_property_t)openk4a_win.get_devnode_property;

    void *node = NULL;
    if (locate(&node, instance_id, OPENK4A_CM_LOCATE_DEVNODE_NORMAL) != OPENK4A_CR_SUCCESS)
    {
        return false;
    }

    ULONG bytes = sizeof(*container_id);
    ULONG type = 0;
    if (get_property(node, &OPENK4A_DEVPKEY_DEVICE_CONTAINER_ID, &type, container_id, &bytes, 0) != OPENK4A_CR_SUCCESS)
    {
        return false;
    }
    return bytes == sizeof(*container_id);
}
