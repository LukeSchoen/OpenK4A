/*=============================================================================
  The colour MCU.

  Interface 2 of the colour device is a second command device, and everything
  about the camera that is not a video frame lives behind it: the
  synchronisation configuration (which sets the master/subordinate mode, the
  delay between depth and colour, and the streaming indicator), the two
  external sync jacks, and the IMU - which is not on the depth processor at
  all but on the colour MCU's own I2C bus.

  The commands are the SDK's own numbers, from src/color_mcu/colorcommands.h.
=============================================================================*/

#include "openk4a.h"

#pragma pack(push, 1)
typedef struct
{
    uint32_t mode;                             /* standalone, master, subordinate */
    uint32_t subordinate_delay_off_master_pts; /* 90 kHz ticks */
    int32_t depth_delay_off_color_pts;         /* 90 kHz ticks */
    uint8_t enable_privacy_led;
} openk4a_sync_config_t;
#pragma pack(pop)

#define OPENK4A_SYNC_MODE_STANDALONE 0
#define OPENK4A_SYNC_MODE_MASTER 1
#define OPENK4A_SYNC_MODE_SUBORDINATE 2

bool openk4a_color_mcu_open(openk4a_device_t *device)
{
    if (device->color_mcu_open)
    {
        return true;
    }
    if (!openk4a_usb_open(&device->usb_color, &OPENK4A_USB_COLOR, device->index))
    {
        return false;
    }
    device->color_mcu_open = true;
    return true;
}

void openk4a_color_mcu_close(openk4a_device_t *device)
{
    if (device->color_mcu_open)
    {
        openk4a_usb_close(&device->usb_color);
        device->color_mcu_open = false;
    }
}

bool openk4a_color_mcu_set_sys_cfg(openk4a_device_t *device, const k4a_device_configuration_t *config)
{
    if (!openk4a_color_mcu_open(device))
    {
        openk4a_log(OPENK4A_LOG_ERROR, "the colour MCU is not there; the camera cannot be configured");
        return false;
    }

    openk4a_sync_config_t sync;
    memset(&sync, 0, sizeof(sync));
    switch (config->wired_sync_mode)
    {
    case K4A_WIRED_SYNC_MODE_STANDALONE:
        sync.mode = OPENK4A_SYNC_MODE_STANDALONE;
        break;
    case K4A_WIRED_SYNC_MODE_MASTER:
        sync.mode = OPENK4A_SYNC_MODE_MASTER;
        break;
    case K4A_WIRED_SYNC_MODE_SUBORDINATE:
        sync.mode = OPENK4A_SYNC_MODE_SUBORDINATE;
        break;
    default:
        openk4a_log(OPENK4A_LOG_ERROR, "wired_sync_mode %d is not a mode", (int)config->wired_sync_mode);
        return false;
    }

    /* The device counts in 90 kHz ticks, which is what its own clock runs at;
     * the API counts in microseconds. */
    const int64_t subordinate = (int64_t)config->subordinate_delay_off_master_usec * 90 / 1000;
    const int64_t depth_delay = (int64_t)config->depth_delay_off_color_usec * 90 / 1000;
    sync.subordinate_delay_off_master_pts = (uint32_t)subordinate;
    sync.depth_delay_off_color_pts = (int32_t)depth_delay;
    sync.enable_privacy_led = config->disable_streaming_indicator ? 0 : 1;

    uint32_t status = 0;
    const bool sent = openk4a_usb_command(&device->usb_color,
                                      OPENK4A_COLOR_CMD_SET_SYS_CFG,
                                      &sync,
                                      sizeof(sync),
                                      NULL,
                                      0,
                                      NULL,
                                      0,
                                      NULL,
                                      &status);
    if (!sent)
    {
        openk4a_log(OPENK4A_LOG_ERROR,
                "the colour MCU refused the synchronisation settings (%zu bytes): status %08X",
                sizeof(sync), status);
    }
    return sent;
}

bool openk4a_color_mcu_get_jacks(openk4a_device_t *device, bool *in_jack, bool *out_jack)
{
    if (!openk4a_color_mcu_open(device))
    {
        return false;
    }
    uint8_t state = 0;
    size_t read = 0;
    if (!openk4a_usb_read_command(&device->usb_color, OPENK4A_COLOR_CMD_GET_JACK_STATE, &state, sizeof(state), &read) ||
        read != sizeof(state))
    {
        return false;
    }
    if (in_jack != NULL)
    {
        *in_jack = (state & 0x1) != 0;
    }
    if (out_jack != NULL)
    {
        *out_jack = (state & 0x2) != 0;
    }
    return true;
}

bool openk4a_color_mcu_imu_start(openk4a_device_t *device)
{
    if (!openk4a_color_mcu_open(device))
    {
        return false;
    }
    return openk4a_usb_write_command(&device->usb_color, OPENK4A_COLOR_CMD_IMU_STREAM_START, NULL, 0);
}

bool openk4a_color_mcu_imu_stop(openk4a_device_t *device)
{
    if (!device->color_mcu_open)
    {
        return true;
    }
    return openk4a_usb_write_command(&device->usb_color, OPENK4A_COLOR_CMD_IMU_STREAM_STOP, NULL, 0);
}

/*-----------------------------------------------------------------------------
  The public face.
---------------------------------------------------------------------------*/

k4a_result_t k4a_device_get_sync_jack(k4a_device_t device_handle, bool *sync_in_jack_connected, bool *sync_out_jack_connected)
{
    openk4a_device_t *device = (openk4a_device_t *)device_handle;
    if (device == NULL || (sync_in_jack_connected == NULL && sync_out_jack_connected == NULL))
    {
        return K4A_RESULT_FAILED;
    }
    if (!openk4a_color_mcu_get_jacks(device, sync_in_jack_connected, sync_out_jack_connected))
    {
        return K4A_RESULT_FAILED;
    }
    return K4A_RESULT_SUCCEEDED;
}
