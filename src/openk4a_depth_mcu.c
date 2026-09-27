/*=============================================================================
  The depth processor's command set.

  Everything the camera is asked to do goes through here: which sensor mode
  it runs, how fast, when to start and stop streaming, and the two documents
  that make the data meaningful - the calibration JSON (which a program reads
  for its intrinsics) and the IR sensor's calibration block (which the depth
  engine is created with, and which this tree expands into its own tables).

  The commands are the SDK's own numbers, from src/depth_mcu/depthcommands.h.
=============================================================================*/

#include "openk4a.h"

bool openk4a_depth_mcu_version(openk4a_device_t *device, openk4a_firmware_versions_t *version)
{
    size_t read = 0;
    memset(version, 0, sizeof(*version));
    if (!openk4a_usb_read_command(&device->usb_depth,
                              OPENK4A_DEPTH_CMD_COMPONENT_VERSION_GET,
                              version,
                              sizeof(*version),
                              &read))
    {
        return false;
    }
    return read >= sizeof(*version);
}

bool openk4a_depth_mcu_mode(openk4a_device_t *device, uint32_t sensor_mode)
{
    return openk4a_usb_write_command(&device->usb_depth, OPENK4A_DEPTH_CMD_MODE_SET, &sensor_mode, sizeof(sensor_mode));
}

bool openk4a_depth_mcu_fps(openk4a_device_t *device, uint32_t fps)
{
    return openk4a_usb_write_command(&device->usb_depth, OPENK4A_DEPTH_CMD_FPS_SET, &fps, sizeof(fps));
}

bool openk4a_depth_mcu_stream_start(openk4a_device_t *device)
{
    if (!openk4a_usb_write_command(&device->usb_depth, OPENK4A_DEPTH_CMD_START, NULL, 0))
    {
        openk4a_log(OPENK4A_LOG_ERROR, "the sensor would not start");
        return false;
    }
    if (!openk4a_usb_write_command(&device->usb_depth, OPENK4A_DEPTH_CMD_STREAM_START, NULL, 0))
    {
        openk4a_log(OPENK4A_LOG_ERROR, "the sensor would not stream");
        openk4a_usb_write_command(&device->usb_depth, OPENK4A_DEPTH_CMD_STOP, NULL, 0);
        return false;
    }
    return true;
}

bool openk4a_depth_mcu_stream_stop(openk4a_device_t *device)
{
    const bool stopped = openk4a_usb_write_command(&device->usb_depth, OPENK4A_DEPTH_CMD_STREAM_STOP, NULL, 0);
    openk4a_usb_write_command(&device->usb_depth, OPENK4A_DEPTH_CMD_STOP, NULL, 0);
    return stopped;
}

bool openk4a_depth_mcu_nv_data(openk4a_device_t *device, uint32_t tag, void *data, size_t size, size_t *read)
{
    return openk4a_usb_command(&device->usb_depth,
                           OPENK4A_DEPTH_CMD_NV_DATA_GET,
                           &tag,
                           sizeof(tag),
                           NULL,
                           0,
                           data,
                           size,
                           read,
                           NULL);
}

/* The depth engine is created with this: the IR sensor's own correction data
 * (NV tag 2), 506,952 bytes of it on this device. It is the device's data,
 * read over the same command set as everything else, and it is what OpenK4A
 * expands into per-mode depth tables. */
bool openk4a_depth_mcu_calibration_block(openk4a_device_t *device)
{
    const size_t capacity = 2 * 1024 * 1024;
    uint8_t *block = (uint8_t *)openk4a_alloc(capacity);
    if (block == NULL)
    {
        return false;
    }

    size_t size = 0;
    const bool read = openk4a_depth_mcu_nv_data(device, OPENK4A_NV_IR_SENSOR_CALIBRATION, block, capacity, &size) && size > 0;
    if (!read)
    {
        openk4a_log(OPENK4A_LOG_WARNING, "the IR sensor's calibration block did not read");
        openk4a_free(block);
        return false;
    }

    device->calibration_block = block;
    device->calibration_block_size = size;
    openk4a_log(OPENK4A_LOG_INFO, "the engine's own calibration block is %zu bytes", size);
    return true;
}

/* The calibration JSON: the same document the closed SDK parses for its
 * intrinsics, extrinsics and the inertial model. */
bool openk4a_depth_mcu_calibration_json(openk4a_device_t *device)
{
    const size_t capacity = 64 * 1024;
    char *json = (char *)openk4a_alloc(capacity);
    if (json == NULL)
    {
        return false;
    }

    size_t size = 0;
    if (!openk4a_usb_read_command(&device->usb_depth,
                              OPENK4A_DEPTH_CMD_READ_CALIBRATION_DATA,
                              json,
                              capacity - 1,
                              &size) ||
        size <= 16)
    {
        openk4a_log(OPENK4A_LOG_WARNING, "the calibration JSON did not read");
        openk4a_free(json);
        return false;
    }
    json[size] = '\0';
    (void)openk4a_cache_write("calibration.json", json, size);

    const bool parsed = openk4a_calibration_parse(json, size, &device->calibration);
    if (!parsed)
    {
        /* A document that will not parse is worth keeping: this is the
         * device's own text, and the first thing to look at is whether it
         * arrived whole. */
        char preview[161];
        const size_t take = size < sizeof(preview) - 1 ? size : sizeof(preview) - 1;
        memcpy(preview, json, take);
        preview[take] = '\0';
        openk4a_log(OPENK4A_LOG_WARNING, "the calibration JSON did not parse: %zu bytes, starting \"%s\"", size, preview);
    }
    openk4a_free(json);
    return parsed;
}

/*-----------------------------------------------------------------------------
  The public view of the firmware.
---------------------------------------------------------------------------*/

k4a_result_t k4a_device_get_version(k4a_device_t device_handle, k4a_hardware_version_t *version)
{
    openk4a_device_t *device = (openk4a_device_t *)device_handle;
    if (device == NULL || version == NULL)
    {
        return K4A_RESULT_FAILED;
    }

    openk4a_firmware_versions_t mcu;
    if (!openk4a_depth_mcu_version(device, &mcu))
    {
        return K4A_RESULT_FAILED;
    }

    memset(version, 0, sizeof(*version));
    version->rgb.major = mcu.rgb_major;
    version->rgb.minor = mcu.rgb_minor;
    version->rgb.iteration = mcu.rgb_build;
    version->depth.major = mcu.depth_major;
    version->depth.minor = mcu.depth_minor;
    version->depth.iteration = mcu.depth_build;
    version->audio.major = mcu.audio_major;
    version->audio.minor = mcu.audio_minor;
    version->audio.iteration = mcu.audio_build;
    version->depth_sensor.major = mcu.depth_sensor_major;
    version->depth_sensor.minor = mcu.depth_sensor_minor;

    switch (mcu.build_config)
    {
    case 0:
        version->firmware_build = K4A_FIRMWARE_BUILD_RELEASE;
        break;
    case 1:
        version->firmware_build = K4A_FIRMWARE_BUILD_DEBUG;
        break;
    default:
        openk4a_log(OPENK4A_LOG_WARNING, "the device reported firmware build %u", (unsigned)mcu.build_config);
        version->firmware_build = K4A_FIRMWARE_BUILD_DEBUG;
        break;
    }

    switch (mcu.signature_type)
    {
    case 0:
        version->firmware_signature = K4A_FIRMWARE_SIGNATURE_MSFT;
        break;
    case 1:
        version->firmware_signature = K4A_FIRMWARE_SIGNATURE_TEST;
        break;
    case 2:
        version->firmware_signature = K4A_FIRMWARE_SIGNATURE_UNSIGNED;
        break;
    default:
        openk4a_log(OPENK4A_LOG_WARNING, "the device reported signature type %u", (unsigned)mcu.signature_type);
        version->firmware_signature = K4A_FIRMWARE_SIGNATURE_UNSIGNED;
        break;
    }
    return K4A_RESULT_SUCCEEDED;
}
