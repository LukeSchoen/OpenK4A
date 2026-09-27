/*=============================================================================
  SetupAPI and WinUSB.

  Two of the camera's three command-visible devices are ordinary WinUSB
  devices on this machine: the depth processor (045E:097C) and the colour
  MCU, which is interface 2 of the colour device (045E:097D). Both speak the
  same command protocol - a twenty-byte header, an optional payload, then a
  sixteen-byte response that carries the transaction's own id - and both
  stream frames or IMU samples over a third endpoint.

  Three things about the protocol are worth writing down, because each one
  costs an afternoon if it is assumed away:

    - a session can start with a packet the last one left behind, so a
      command waits for the response carrying its own id rather than the
      first one to arrive;
    - the device ends a transfer with a zero-length packet, so a read that
      comes back empty is read again;
    - a read can come back with fewer bytes than were asked for, and asking
      for more than exists is not an error. Insisting on the full size leaves
      the next command reading this one's leftovers.
=============================================================================*/

#include "openk4a.h"

const openk4a_usb_id_t OPENK4A_USB_DEPTH = {
    0x097C, -1, 0x02, 0x81, 0x83
};

const openk4a_usb_id_t OPENK4A_USB_COLOR = {
    0x097D, 2, 0x04, 0x83, 0x82
};

#pragma pack(push, 1)
typedef struct
{
    uint32_t packet_type;
    uint32_t packet_transaction_id;
    uint32_t payload_size;
    uint32_t command;
    uint32_t reserved;
} openk4a_command_header_t;

typedef struct
{
    openk4a_command_header_t header;
    uint8_t data[OPENK4A_CMD_PACKET_DATA_BYTES];
} openk4a_command_packet_t;

typedef struct
{
    uint32_t packet_type;
    uint32_t packet_transaction_id;
    uint32_t status;
    uint32_t reserved;
} openk4a_command_response_t;
#pragma pack(pop)

/*=============================================================================
  Finding a camera
=============================================================================*/

/* The device paths WinUSB hands back name the device they belong to:
 * \\?\usb#vid_045e&pid_097c#<serial>#{...}, and for a composite device's
 * child \\?\usb#vid_045e&pid_097d&mi_02#<instance>#{...}. */
static bool path_matches(const char *path, const openk4a_usb_id_t *id)
{
    char wanted[32];
    snprintf(wanted, sizeof(wanted), "vid_045e&pid_%04x", (unsigned)id->pid);
    for (const char *at = path; *at != '\0'; at++)
    {
        size_t i = 0;
        while (wanted[i] != '\0' && at[i] != '\0')
        {
            char c = at[i];
            if (c >= 'A' && c <= 'Z')
            {
                c = (char)(c - 'A' + 'a');
            }
            if (c != wanted[i])
            {
                break;
            }
            i++;
        }
        if (wanted[i] == '\0')
        {
            /* Pid matched. A composite child also has to be the interface
             * asked for, or a different function of the same camera. */
            if (id->interface_mi < 0)
            {
                return true;
            }
            char mi[16];
            snprintf(mi, sizeof(mi), "&mi_%02x#", (unsigned)id->interface_mi);
            for (const char *scan = at; *scan != '\0'; scan++)
            {
                size_t j = 0;
                while (mi[j] != '\0' && scan[j] != '\0')
                {
                    char c = scan[j];
                    if (c >= 'A' && c <= 'Z')
                    {
                        c = (char)(c - 'A' + 'a');
                    }
                    if (c != mi[j])
                    {
                        break;
                    }
                    j++;
                }
                if (mi[j] == '\0')
                {
                    return true;
                }
            }
            return false;
        }
    }
    return false;
}

/* The serial number the SDK reports is the one in that path. */
void openk4a_usb_serial_from_path(const char *path, char *serial, size_t serial_size)
{
    serial[0] = '\0';
    const char *first = strchr(path, '#');
    if (first == NULL)
    {
        return;
    }
    const char *second = strchr(first + 1, '#');
    if (second == NULL)
    {
        return;
    }
    const char *end = strchr(second + 1, '#');
    if (end == NULL || end <= second + 1)
    {
        return;
    }
    size_t length = (size_t)(end - (second + 1));
    if (length >= serial_size)
    {
        length = serial_size - 1;
    }
    memcpy(serial, second + 1, length);
    serial[length] = '\0';
}

/* Walks the device interfaces one of the three classes registers and returns
 * the nth whose path names this camera. */
static bool find_interface(const GUID *guid, const openk4a_usb_id_t *id, uint32_t index, char *path, size_t path_size)
{
    if (!openk4a_win_init())
    {
        return false;
    }

    typedef void *(*get_class_devs_t)(const GUID *, const char *, HWND, DWORD);
    typedef BOOL (*enum_interfaces_t)(void *, void *, const GUID *, DWORD, openk4a_sp_interface_data_t *);
    typedef BOOL (*get_detail_t)(void *, openk4a_sp_interface_data_t *, openk4a_sp_interface_detail_t *, DWORD, DWORD *,
                                 void *);
    typedef BOOL (*destroy_list_t)(void *);

    get_class_devs_t get_class_devs = (get_class_devs_t)openk4a_win.get_class_devs;
    enum_interfaces_t enumerate = (enum_interfaces_t)openk4a_win.enum_interfaces;
    get_detail_t get_detail = (get_detail_t)openk4a_win.get_detail;
    destroy_list_t destroy_list = (destroy_list_t)openk4a_win.destroy_list;

    void *set = get_class_devs(guid, NULL, NULL, OPENK4A_DIGCF_PRESENT | OPENK4A_DIGCF_DEVICEINTERFACE);
    if (set == INVALID_HANDLE_VALUE || set == NULL)
    {
        return false;
    }

    bool found = false;
    uint32_t seen = 0;
    for (DWORD i = 0; !found; i++)
    {
        openk4a_sp_interface_data_t data;
        data.cbSize = sizeof(data);
        if (!enumerate(set, NULL, guid, i, &data))
        {
            break;
        }

        DWORD needed = 0;
        get_detail(set, &data, NULL, 0, &needed, NULL);
        if (needed == 0 || needed > 8192)
        {
            continue;
        }

        openk4a_sp_interface_detail_t *detail = (openk4a_sp_interface_detail_t *)openk4a_alloc(needed);
        if (detail == NULL)
        {
            break;
        }
        detail->cbSize = 8; /* the documented size of this header on x64 */
        if (!get_detail(set, &data, detail, needed, NULL, NULL))
        {
            openk4a_free(detail);
            continue;
        }

        if (path_matches(detail->DevicePath, id))
        {
            if (seen == index)
            {
                snprintf(path, path_size, "%s", detail->DevicePath);
                found = true;
            }
            seen++;
        }
        openk4a_free(detail);
    }

    destroy_list(set);
    return found;
}

uint32_t openk4a_usb_count(const openk4a_usb_id_t *id)
{
    uint32_t count = 0;
    char path[512];
    while (count < 16 && find_interface(&OPENK4A_GUID_DEVINTERFACE_WINUSB, id, count, path, sizeof(path)))
    {
        count++;
    }
    return count;
}

bool openk4a_usb_path(const openk4a_usb_id_t *id, uint32_t index, char *out, size_t out_size)
{
    return find_interface(&OPENK4A_GUID_DEVINTERFACE_WINUSB, id, index, out, out_size);
}

/*=============================================================================
  Opening and closing
=============================================================================*/

/* Every endpoint the command set needs has to be there, or the handle is
 * useless: a device that enumerated but lost a pipe is a stale binding. */
static bool endpoint_present(openk4a_winusb_handle_t winusb, uint8_t address)
{
    typedef BOOL (*query_interface_t)(openk4a_winusb_handle_t, UCHAR, openk4a_usb_interface_descriptor_t *);
    typedef BOOL (*query_pipe_t)(openk4a_winusb_handle_t, UCHAR, UCHAR, openk4a_winusb_pipe_information_t *);

    openk4a_usb_interface_descriptor_t descriptor;
    if (!((query_interface_t)openk4a_win.query_interface)(winusb, 0, &descriptor))
    {
        return false;
    }
    for (UCHAR i = 0; i < descriptor.bNumEndpoints; i++)
    {
        openk4a_winusb_pipe_information_t pipe;
        if (!((query_pipe_t)openk4a_win.query_pipe)(winusb, 0, i, &pipe))
        {
            return false;
        }
        if (pipe.PipeId == address)
        {
            return true;
        }
    }
    return false;
}

bool openk4a_usb_open(openk4a_usb_t *usb, const openk4a_usb_id_t *id, uint32_t index)
{
    memset(usb, 0, sizeof(*usb));
    usb->file = INVALID_HANDLE_VALUE;

    if (!openk4a_win_init())
    {
        return false;
    }

    /* The device registers under three interface classes; the WinUSB-bound
     * one is tried first and the others after it, so a machine whose binding
     * differs still opens. */
    const GUID *classes[] = { &OPENK4A_GUID_DEVINTERFACE_WINUSB,
                              &OPENK4A_GUID_DEVINTERFACE_LIBUSB,
                              &OPENK4A_GUID_DEVINTERFACE_USB_DEVICE };
    for (int c = 0; c < 3 && usb->winusb == NULL; c++)
    {
        for (uint32_t attempt = 0; attempt < 8 && usb->winusb == NULL; attempt++)
        {
            /* The caller's index is which camera it wants, so try that one
             * first and then the rest - there is usually only one. */
            const uint32_t which = attempt == 0 ? index : (attempt <= index ? attempt - 1 : attempt);
            char path[512];
            if (!find_interface(classes[c], id, which, path, sizeof(path)))
            {
                break;
            }

            HANDLE file = CreateFileA(path,
                                      GENERIC_READ | GENERIC_WRITE,
                                      FILE_SHARE_READ | FILE_SHARE_WRITE,
                                      NULL,
                                      OPEN_EXISTING,
                                      FILE_ATTRIBUTE_NORMAL | FILE_FLAG_OVERLAPPED,
                                      NULL);
            if (file == INVALID_HANDLE_VALUE)
            {
                openk4a_log(OPENK4A_LOG_TRACE, "cannot open %s (%lu)", path, GetLastError());
                continue;
            }

            openk4a_winusb_handle_t winusb = NULL;
            typedef BOOL (*initialize_t)(HANDLE, openk4a_winusb_handle_t *);
            if (!((initialize_t)openk4a_win.winusb_initialize)(file, &winusb))
            {
                openk4a_log(OPENK4A_LOG_TRACE, "WinUsb_Initialize failed for %s (%lu)", path, GetLastError());
                CloseHandle(file);
                continue;
            }

            usb->file = file;
            usb->winusb = winusb;
            openk4a_usb_serial_from_path(path, usb->serial, sizeof(usb->serial));
            snprintf(usb->path, sizeof(usb->path), "%s", path);
            break;
        }
    }

    if (usb->winusb == NULL)
    {
        openk4a_log(OPENK4A_LOG_ERROR, "no Kinect device 045E:%04X this can open", (unsigned)id->pid);
        return false;
    }

    usb->endpoint_out = id->endpoint_out;
    usb->endpoint_in = id->endpoint_in;
    usb->endpoint_stream = id->endpoint_stream;

    bool ok = endpoint_present(usb->winusb, usb->endpoint_out) &&
              endpoint_present(usb->winusb, usb->endpoint_in) &&
              endpoint_present(usb->winusb, usb->endpoint_stream);
    if (!ok)
    {
        openk4a_log(OPENK4A_LOG_ERROR, "the expected endpoints are not there");
        openk4a_usb_close(usb);
        return false;
    }

    (void)openk4a_usb_set_timeout(usb, usb->endpoint_out, OPENK4A_CMD_TIMEOUT_MS);
    (void)openk4a_usb_set_timeout(usb, usb->endpoint_in, OPENK4A_CMD_TIMEOUT_MS);
    (void)openk4a_usb_set_timeout(usb, usb->endpoint_stream, OPENK4A_CMD_TIMEOUT_MS);
    return true;
}

void openk4a_usb_close(openk4a_usb_t *usb)
{
    if (usb == NULL)
    {
        return;
    }
    if (usb->winusb != NULL)
    {
        typedef BOOL (*free_t)(openk4a_winusb_handle_t);
        ((free_t)openk4a_win.winusb_free)(usb->winusb);
        usb->winusb = NULL;
    }
    if (usb->file != INVALID_HANDLE_VALUE && usb->file != NULL)
    {
        CloseHandle(usb->file);
    }
    usb->file = INVALID_HANDLE_VALUE;
}

bool openk4a_usb_set_timeout(openk4a_usb_t *usb, uint8_t endpoint, uint32_t milliseconds)
{
    typedef BOOL (*set_policy_t)(openk4a_winusb_handle_t, UCHAR, DWORD, DWORD, void *);
    return ((set_policy_t)openk4a_win.set_pipe_policy)(usb->winusb,
                                                   endpoint,
                                                   OPENK4A_PIPE_TRANSFER_TIMEOUT,
                                                   sizeof(milliseconds),
                                                   &milliseconds) != FALSE;
}

/*=============================================================================
  Reading and writing
=============================================================================*/

/* The MCU ends a transfer with a zero-length packet of its own, so one read
 * can come back empty and still be a success. Read until there is something,
 * and treat a run of empties as nothing there. */
static bool read_bulk(openk4a_usb_t *usb, uint8_t endpoint, void *buffer, size_t size, DWORD *received)
{
    typedef BOOL (*read_pipe_t)(openk4a_winusb_handle_t, UCHAR, UCHAR *, DWORD, DWORD *, void *);
    read_pipe_t read_pipe = (read_pipe_t)openk4a_win.read_pipe;

    for (int attempt = 0; attempt < OPENK4A_READ_ATTEMPTS; attempt++)
    {
        DWORD got = 0;
        if (!read_pipe(usb->winusb, endpoint, (UCHAR *)buffer, (DWORD)size, &got, NULL))
        {
            openk4a_log(OPENK4A_LOG_ERROR, "reading %zu bytes failed (%lu)", size, GetLastError());
            return false;
        }
        if (got > 0)
        {
            *received = got;
            return true;
        }
    }
    *received = 0;
    return true;
}

static bool write_bulk(openk4a_usb_t *usb, uint8_t endpoint, const void *buffer, size_t size)
{
    typedef BOOL (*write_pipe_t)(openk4a_winusb_handle_t, UCHAR, UCHAR *, DWORD, DWORD *, void *);
    DWORD written = 0;
    return ((write_pipe_t)openk4a_win.write_pipe)(usb->winusb,
                                              endpoint,
                                              (UCHAR *)buffer,
                                              (DWORD)size,
                                              &written,
                                              NULL) != FALSE;
}

bool openk4a_usb_command(openk4a_usb_t *usb,
                     uint32_t command,
                     const void *command_data,
                     size_t command_data_size,
                     const void *tx_data,
                     size_t tx_size,
                     void *rx_data,
                     size_t rx_size,
                     size_t *bytes_read,
                     uint32_t *status)
{
    if (usb == NULL || usb->winusb == NULL)
    {
        return false;
    }
    if (bytes_read != NULL)
    {
        *bytes_read = 0;
    }
    if (command_data_size > OPENK4A_CMD_PACKET_DATA_BYTES)
    {
        openk4a_log(OPENK4A_LOG_ERROR, "command %08X carries %zu bytes of data, over the packet's %d",
                command, command_data_size, OPENK4A_CMD_PACKET_DATA_BYTES);
        return false;
    }

    /* A read transaction's payload size is what it wants to receive; a write
     * transaction's is what it is about to send. Never both. */
    const size_t payload_size = rx_size != 0 ? rx_size : tx_size;

    openk4a_command_packet_t packet;
    memset(&packet, 0, sizeof(packet));
    packet.header.packet_type = OPENK4A_CMD_PACKET_TYPE;
    packet.header.packet_transaction_id = usb->transaction_id++;
    packet.header.payload_size = (uint32_t)payload_size;
    packet.header.command = command;
    packet.header.reserved = 0;
    if (command_data != NULL && command_data_size > 0)
    {
        memcpy(packet.data, command_data, command_data_size);
    }

    if (!write_bulk(usb, usb->endpoint_out, &packet, sizeof(openk4a_command_header_t) + command_data_size))
    {
        openk4a_log(OPENK4A_LOG_ERROR, "command %08X: sending the header failed (%lu)", command, GetLastError());
        return false;
    }

    if (tx_data != NULL && tx_size > 0)
    {
        if (!write_bulk(usb, usb->endpoint_out, tx_data, tx_size))
        {
            openk4a_log(OPENK4A_LOG_ERROR, "command %08X: sending %zu bytes failed (%lu)", command, tx_size, GetLastError());
            return false;
        }
    }

    if (rx_data != NULL && rx_size > 0)
    {
        DWORD got = 0;
        if (!read_bulk(usb, usb->endpoint_in, rx_data, rx_size, &got))
        {
            return false;
        }
        if (bytes_read != NULL)
        {
            *bytes_read = (size_t)got;
        }
    }

    openk4a_command_response_t response;
    memset(&response, 0, sizeof(response));
    bool matched = false;
    for (int attempt = 0; attempt < 3 && !matched; attempt++)
    {
        DWORD got = 0;
        if (!read_bulk(usb, usb->endpoint_in, &response, sizeof(response), &got))
        {
            return false;
        }
        if (got != sizeof(response))
        {
            openk4a_log(OPENK4A_LOG_ERROR, "command %08X: the response was %lu bytes, not %zu",
                    command, (unsigned long)got, sizeof(response));
            return false;
        }
        matched = response.packet_type == OPENK4A_CMD_PACKET_TYPE_RESPONSE &&
                  response.packet_transaction_id == packet.header.packet_transaction_id;
        if (!matched)
        {
            openk4a_log(OPENK4A_LOG_TRACE,
                    "command %08X: dropped a response for id %u (this command is id %u)",
                    command, response.packet_transaction_id, packet.header.packet_transaction_id);
        }
    }
    if (!matched)
    {
        openk4a_log(OPENK4A_LOG_ERROR,
                "command %08X (id %u): the response never carried its id (last: type %08X id %u status %08X)",
                command, packet.header.packet_transaction_id, response.packet_type,
                response.packet_transaction_id, response.status);
        return false;
    }

    if (status != NULL)
    {
        *status = response.status;
    }
    return response.status == 0;
}

/*=============================================================================
  Overlapped reads
=============================================================================*/

bool openk4a_usb_io_post(openk4a_usb_t *usb, openk4a_io_t *io)
{
    if (io->event == NULL)
    {
        io->event = CreateEventA(NULL, TRUE, FALSE, NULL);
        if (io->event == NULL)
        {
            return false;
        }
    }
    ResetEvent(io->event);
    memset(&io->overlapped, 0, sizeof(io->overlapped));
    io->overlapped.hEvent = io->event;
    io->transferred = 0;
    io->ready = false;
    io->pending = false;

    typedef BOOL (*read_pipe_t)(openk4a_winusb_handle_t, UCHAR, UCHAR *, DWORD, DWORD *, OVERLAPPED *);
    if (((read_pipe_t)openk4a_win.read_pipe)(usb->winusb,
                                         usb->endpoint_stream,
                                         io->buffer,
                                         (DWORD)io->size,
                                         &io->transferred,
                                         &io->overlapped))
    {
        /* The device had a whole frame waiting: it is in the buffer now. */
        io->ready = true;
        return true;
    }
    if (GetLastError() == ERROR_IO_PENDING)
    {
        io->pending = true;
        return true;
    }
    openk4a_log(OPENK4A_LOG_ERROR, "the stream read would not start (%lu)", GetLastError());
    return false;
}

int openk4a_usb_io_wait(openk4a_usb_t *usb, openk4a_io_t *io, int timeout_ms)
{
    if (io->ready)
    {
        return OPENK4A_OK;
    }
    if (!io->pending)
    {
        return OPENK4A_FAILED;
    }

    const DWORD wait = timeout_ms < 0 ? INFINITE : (DWORD)timeout_ms;
    if (WaitForSingleObject(io->event, wait) != WAIT_OBJECT_0)
    {
        /* The transfer is still running: not a failure, and the caller can
         * ask again. */
        return OPENK4A_TIMEOUT;
    }

    DWORD transferred = 0;
    if (!GetOverlappedResult(usb->file, &io->overlapped, &transferred, FALSE))
    {
        const DWORD error = GetLastError();
        io->pending = false;
        /* A transfer the transfer timeout cut short is how a frame that was
         * not there is reported; the caller re-posts and asks again. */
        if (error == ERROR_SEM_TIMEOUT || error == ERROR_OPERATION_ABORTED || error == ERROR_IO_INCOMPLETE)
        {
            return OPENK4A_TIMEOUT;
        }
        openk4a_log(OPENK4A_LOG_ERROR, "the stream read failed (%lu)", error);
        return OPENK4A_FAILED;
    }
    io->pending = false;
    io->transferred = transferred;
    io->ready = true;
    return OPENK4A_OK;
}

void openk4a_usb_io_cancel(openk4a_usb_t *usb, openk4a_io_t *io)
{
    if (io->pending && openk4a_cancel_io_ex != NULL)
    {
        openk4a_cancel_io_ex(usb->file, &io->overlapped);
        DWORD transferred = 0;
        GetOverlappedResult(usb->file, &io->overlapped, &transferred, TRUE);
    }
    io->pending = false;
    io->ready = false;
}

void openk4a_usb_io_close(openk4a_usb_t *usb, openk4a_io_t *io)
{
    openk4a_usb_io_cancel(usb, io);
    if (io->event != NULL)
    {
        CloseHandle(io->event);
        io->event = NULL;
    }
    openk4a_free(io->buffer);
    io->buffer = NULL;
}
