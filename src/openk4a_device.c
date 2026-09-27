/*=============================================================================
  The device, and the public face of the SDK.

  Bringing the camera up is the SDK's own order, and the order matters:
  the sensor mode first, because the calibration block is only readable while
  the sensor is powered for it; then the block and the JSON; then the depth
  tables, which are built from both; then the frame rate; then start.

  Getting a frame is where the streaming design shows. Two read buffers are
  armed on the stream endpoint, and the next read is posted the moment the
  previous one has arrived, before the frame is decoded - so the transfer runs
  while the depth pass runs, and the read is never on the critical path
  between two frames. Overlapped I/O is not a thread, so the depth pass stays
  single-threaded.
=============================================================================*/

#include "openk4a.h"

/*=============================================================================
  Opening
=============================================================================*/

uint32_t k4a_device_get_installed_count(void)
{
    return openk4a_usb_count(&OPENK4A_USB_DEPTH);
}

k4a_result_t k4a_device_open(uint32_t index, k4a_device_t *device_handle)
{
    if (device_handle == NULL)
    {
        return K4A_RESULT_FAILED;
    }

    openk4a_device_t *device = (openk4a_device_t *)openk4a_alloc_zero(sizeof(openk4a_device_t));
    if (device == NULL)
    {
        return K4A_RESULT_FAILED;
    }
    device->index = index;
    device->decode_ir = true;
    device->decode_depth = true;

    if (!openk4a_usb_open(&device->usb_depth, &OPENK4A_USB_DEPTH, index))
    {
        openk4a_free(device);
        return K4A_RESULT_FAILED;
    }
    snprintf(device->serial, sizeof(device->serial), "%s", device->usb_depth.serial);

    /* The colour MCU is a second device, and the calibration block, the IMU
     * and the sync jacks all live behind it, so it is opened with the camera
     * rather than with the colour stream. */
    if (!openk4a_color_mcu_open(device))
    {
        openk4a_log(OPENK4A_LOG_ERROR, "the colour MCU would not open; the camera is unusable without it");
        openk4a_usb_close(&device->usb_depth);
        openk4a_free(device);
        return K4A_RESULT_FAILED;
    }

    /* The calibration JSON, which is what a program asks for by name and what
     * every depth mode's intrinsics are denormalized from. */
    if (!openk4a_depth_mcu_calibration_json(device))
    {
        openk4a_log(OPENK4A_LOG_ERROR, "the device's calibration could not be read");
        openk4a_color_mcu_close(device);
        openk4a_usb_close(&device->usb_depth);
        openk4a_free(device);
        return K4A_RESULT_FAILED;
    }

    device->start_time_nsec = openk4a_now_nsec();
    *device_handle = (k4a_device_t)device;
    return K4A_RESULT_SUCCEEDED;
}

void k4a_device_close(k4a_device_t device_handle)
{
    openk4a_device_t *device = (openk4a_device_t *)device_handle;
    if (device == NULL)
    {
        return;
    }

    k4a_device_stop_cameras(device_handle);
    k4a_device_stop_imu(device_handle);

    if (device->color != NULL)
    {
        openk4a_color_destroy(device->color);
        device->color = NULL;
    }
    if (device->imu != NULL)
    {
        openk4a_imu_destroy(device->imu);
        device->imu = NULL;
    }

    openk4a_depth_model_free(device->model);
    openk4a_calibration_free(&device->calibration);
    openk4a_free(device->calibration_block);
    openk4a_free(device->ir);
    openk4a_free(device->depth);
    openk4a_color_mcu_close(device);
    openk4a_usb_close(&device->usb_depth);
    openk4a_free(device);
}

k4a_buffer_result_t k4a_device_get_serialnum(k4a_device_t device_handle, char *serial_number, size_t *serial_number_size)
{
    openk4a_device_t *device = (openk4a_device_t *)device_handle;
    if (device == NULL || serial_number_size == NULL)
    {
        return K4A_BUFFER_RESULT_FAILED;
    }
    const size_t length = strlen(device->serial) + 1;
    if (serial_number == NULL || *serial_number_size < length)
    {
        *serial_number_size = length;
        return K4A_BUFFER_RESULT_TOO_SMALL;
    }
    memcpy(serial_number, device->serial, length);
    *serial_number_size = length;
    return K4A_BUFFER_RESULT_SUCCEEDED;
}

k4a_buffer_result_t k4a_device_get_raw_calibration(k4a_device_t device_handle, uint8_t *data, size_t *data_size)
{
    openk4a_device_t *device = (openk4a_device_t *)device_handle;
    if (device == NULL || data_size == NULL || !device->calibration.valid)
    {
        return K4A_BUFFER_RESULT_FAILED;
    }
    if (data == NULL || *data_size < device->calibration.json_size)
    {
        *data_size = device->calibration.json_size;
        return K4A_BUFFER_RESULT_TOO_SMALL;
    }
    memcpy(data, device->calibration.json, device->calibration.json_size);
    *data_size = device->calibration.json_size;
    return K4A_BUFFER_RESULT_SUCCEEDED;
}

k4a_result_t k4a_device_get_calibration(k4a_device_t device_handle,
                                        const k4a_depth_mode_t depth_mode,
                                        const k4a_color_resolution_t color_resolution,
                                        k4a_calibration_t *calibration)
{
    openk4a_device_t *device = (openk4a_device_t *)device_handle;
    if (device == NULL || calibration == NULL)
    {
        return K4A_RESULT_FAILED;
    }
    return openk4a_calibration_build(&device->calibration, depth_mode, color_resolution, calibration)
               ? K4A_RESULT_SUCCEEDED
               : K4A_RESULT_FAILED;
}

/*=============================================================================
  The depth stream
=============================================================================*/

bool openk4a_device_stream_start(openk4a_device_t *device, k4a_depth_mode_t depth_mode, uint32_t fps)
{
    const openk4a_mode_t *mode = openk4a_mode_of(depth_mode);
    if (mode == NULL)
    {
        openk4a_log(OPENK4A_LOG_ERROR, "depth mode %d is not one the camera has", (int)depth_mode);
        return false;
    }

    const size_t pixels = (size_t)mode->width * (size_t)mode->height;
    /* A stream that is being restarted in the same mode keeps the planes it
     * already has: a viewer that turns the colour camera on and off should not
     * be allocating two megabytes each time it does. */
    if (device->mode != mode || device->ir == NULL || device->depth == NULL || device->frame == NULL)
    {
        openk4a_free(device->ir);
        openk4a_free(device->depth);
        openk4a_free(device->frame);
        device->ir = (uint16_t *)openk4a_alloc_zero(pixels * sizeof(uint16_t));
        device->depth = (uint16_t *)openk4a_alloc_zero(pixels * sizeof(uint16_t));
        device->frame = (uint8_t *)openk4a_alloc(mode->frame_bytes);
        if (device->ir == NULL || device->depth == NULL || device->frame == NULL)
        {
            openk4a_log(OPENK4A_LOG_ERROR, "out of memory for the frame buffers");
            return false;
        }
    }
    device->mode = mode;
    device->frame_size = mode->frame_bytes;

    /* A session that was killed left the sensor streaming; the SDK makes the
     * same move for the same reason, by stopping before it starts. */
    if (!openk4a_depth_mcu_stream_stop(device))
    {
        openk4a_log(OPENK4A_LOG_INFO, "the sensor was not streaming before this start");
    }

    if (!openk4a_depth_mcu_mode(device, mode->sensor_mode))
    {
        openk4a_log(OPENK4A_LOG_ERROR, "the sensor would not take mode %u", (unsigned)mode->sensor_mode);
        return false;
    }

    /* The block is only readable while the sensor is powered for it, which is
     * why it comes after the mode and before the stream starts. */
    if (device->calibration_block == NULL)
    {
        (void)openk4a_depth_mcu_calibration_block(device);
    }

    char where[128] = "";
    if (device->model == NULL)
    {
        device->model = openk4a_depth_model_open(depth_mode,
                                             device->calibration_block,
                                             device->calibration_block_size,
                                             &device->calibration,
                                             where,
                                             sizeof(where));
    }
    else
    {
        snprintf(where, sizeof(where), "%s", openk4a_depth_model_where(device->model));
    }
    if (mode->has_depth)
    {
        if (device->model != NULL)
        {
            openk4a_log(OPENK4A_LOG_INFO, "depth tables: %s", where);
        }
        else
        {
            /* The depth pass needs the engine's own numbers for the mode, and
             * the engine is the only thing that has them for every mode: see
             * the table section of README.md for the one command that reads
             * them out and where to put them. */
            openk4a_log(OPENK4A_LOG_WARNING,
                    "no depth tables for %s, so its depth plane stays zero; see README.md for how to extract them "
                    "from the depth engine",
                    openk4a_mode_name(depth_mode));
        }
    }

    if (!openk4a_depth_mcu_fps(device, fps))
    {
        /* The wide-field modes run at 15 frames a second and no faster, so a
         * frame rate the sensor will not take is worth naming. */
        openk4a_log(OPENK4A_LOG_ERROR, "the sensor would not take %u frames a second in %s",
                (unsigned)fps, openk4a_mode_name(depth_mode));
        return false;
    }
    if (!openk4a_depth_mcu_stream_start(device))
    {
        return false;
    }

    /* Both read buffers are armed now: the first frame's transfer runs while
     * the caller is still getting ready. */
    for (int i = 0; i < 2; i++)
    {
        device->io[i].buffer = (uint8_t *)openk4a_alloc(mode->payload_bytes);
        device->io[i].size = mode->payload_bytes;
        if (device->io[i].buffer == NULL || !openk4a_usb_io_post(&device->usb_depth, &device->io[i]))
        {
            for (int j = 0; j < 2; j++)
            {
                openk4a_usb_io_close(&device->usb_depth, &device->io[j]);
            }
            return false;
        }
    }
    device->io_index = 0;
    device->io_armed = true;

    /* What a session that was killed left in the device's buffer would put
     * every frame after it out of step, so the first reads are thrown away
     * until one of them is a whole frame with a footer. */
    for (int attempt = 0; attempt < 4; attempt++)
    {
        openk4a_frame_info_t info;
        if (openk4a_usb_io_wait(&device->usb_depth, &device->io[device->io_index], 1000) != OPENK4A_OK)
        {
            break;
        }
        openk4a_io_t *io = &device->io[device->io_index];
        const bool whole = io->transferred >= mode->frame_bytes &&
                           openk4a_frame_info(io->buffer, io->transferred, &info) && info.exposure_ticks != 0;
        openk4a_usb_io_post(&device->usb_depth, io);
        device->io_index ^= 1;
        if (whole)
        {
            return true;
        }
        openk4a_log(OPENK4A_LOG_INFO, "dropped a stale frame left by the last session");
    }
    return true;
}

void openk4a_device_stream_stop(openk4a_device_t *device)
{
    if (!device->io_armed)
    {
        return;
    }
    for (int i = 0; i < 2; i++)
    {
        openk4a_usb_io_cancel(&device->usb_depth, &device->io[i]);
    }
    (void)openk4a_depth_mcu_stream_stop(device);
    for (int i = 0; i < 2; i++)
    {
        openk4a_usb_io_close(&device->usb_depth, &device->io[i]);
    }
    device->io_armed = false;
}

/* One frame: the transfer that has been running is waited for, the next one
 * is posted on the other buffer, and only then is the frame decoded. */
bool openk4a_device_frame(openk4a_device_t *device, openk4a_frame_info_t *info, int timeout_ms)
{
    if (!device->io_armed || device->mode == NULL)
    {
        return false;
    }

    openk4a_io_t *io = &device->io[device->io_index];
    const int waited = openk4a_usb_io_wait(&device->usb_depth, io, timeout_ms);
    if (waited == OPENK4A_TIMEOUT)
    {
        return false; /* the transfer is still running; the caller may ask again */
    }
    if (waited != OPENK4A_OK)
    {
        openk4a_log(OPENK4A_LOG_ERROR, "the depth stream read failed");
        (void)openk4a_usb_io_post(&device->usb_depth, io);
        return false;
    }

    const size_t received = io->transferred;
    const bool whole = received >= device->mode->frame_bytes;
    if (whole)
    {
        memcpy(device->frame, io->buffer, device->mode->frame_bytes);
    }

    /* The buffer is free the moment its bytes are copied out, so the next
     * read goes in before any of the decoding below runs. The other buffer is
     * already armed and waiting for its own frame: posting it again would put
     * two reads on the one buffer, and the device, with nowhere to put the
     * frame in between, drops it - which is how a camera running at thirty
     * frames a second is read at fifteen. */
    (void)openk4a_usb_io_post(&device->usb_depth, io);
    device->io_index ^= 1;

    if (!whole)
    {
        openk4a_log(OPENK4A_LOG_WARNING, "a frame came back short (%zu of %zu)", received, device->mode->frame_bytes);
        return false;
    }

    const size_t pixels = (size_t)device->mode->width * (size_t)device->mode->height;
    const bool want_ir = device->decode_ir;
    const bool want_depth = device->decode_depth && device->mode->has_depth && device->model != NULL;
    if (!want_ir)
    {
        memset(device->ir, 0, pixels * sizeof(uint16_t));
    }
    if (!want_depth)
    {
        memset(device->depth, 0, pixels * sizeof(uint16_t));
    }
    if (device->mode->sensor_mode == 3)
    {
        if (want_ir)
        {
            openk4a_ir_decode_passive(device->frame, device->frame_size, device->ir, pixels);
        }
    }
    else if (device->mode->nine_taps)
    {
        if (want_ir && want_depth)
        {
            /* The projection works the amplitudes out for the filter, and the
             * IR is the sum of those amplitudes: one pass over the frame
             * rather than two. */
            openk4a_depth_decode_full(device->model, device->frame, device->frame_size, device->depth, device->ir, NULL);
        }
        else if (want_depth)
        {
            openk4a_depth_decode(device->model, device->frame, device->frame_size, device->depth);
        }
        else if (want_ir)
        {
            /* The projection is what makes the IR amplitudes; the filter is
             * the part that costs, and a caller showing the IR does not want
             * it. Passing no depth plane asks for exactly that. */
            openk4a_depth_decode_full(device->model, device->frame, device->frame_size, NULL, device->ir, NULL);
        }
    }

    if (info != NULL)
    {
        openk4a_frame_info(device->frame, device->frame_size, info);
    }
    return true;
}

/*=============================================================================
  The public streaming face
=============================================================================*/

static uint32_t fps_of(k4a_fps_t fps)
{
    switch (fps)
    {
    case K4A_FRAMES_PER_SECOND_5:
        return 5;
    case K4A_FRAMES_PER_SECOND_15:
        return 15;
    default:
        return 30;
    }
}

k4a_result_t k4a_device_start_cameras(k4a_device_t device_handle, const k4a_device_configuration_t *config)
{
    openk4a_device_t *device = (openk4a_device_t *)device_handle;
    if (device == NULL || config == NULL)
    {
        return K4A_RESULT_FAILED;
    }
    if (device->cameras_started)
    {
        openk4a_log(OPENK4A_LOG_ERROR, "the cameras are already running");
        return K4A_RESULT_FAILED;
    }
    if (config->depth_mode == K4A_DEPTH_MODE_OFF && config->color_resolution == K4A_COLOR_RESOLUTION_OFF)
    {
        openk4a_log(OPENK4A_LOG_ERROR, "neither a depth mode nor a colour resolution was asked for");
        return K4A_RESULT_FAILED;
    }

    device->config = *config;

    /* The synchronisation configuration goes to the colour MCU first: it is
     * what sets the master/subordinate mode and the delay between the two
     * sensors, and both streams depend on it. */
    /* The synchronisation configuration carries the master/subordinate mode,
     * the delay between the two sensors and the streaming indicator. The
     * firmware on this device refuses the command whatever it is given - see
     * the note in README.md - and a single camera does not need it, so a
     * refusal is only fatal when the caller actually asked to be part of a
     * multi-camera rig. */
    if (!openk4a_color_mcu_set_sys_cfg(device, config))
    {
        if (config->wired_sync_mode != K4A_WIRED_SYNC_MODE_STANDALONE)
        {
            openk4a_log(OPENK4A_LOG_ERROR,
                    "this camera will not take the synchronisation configuration, so wired sync mode %d cannot be "
                    "honoured",
                    (int)config->wired_sync_mode);
            return K4A_RESULT_FAILED;
        }
        openk4a_log(OPENK4A_LOG_WARNING,
                "this camera refused the synchronisation configuration; carrying on, because a standalone camera "
                "does not need it");
    }

    if (config->depth_mode != K4A_DEPTH_MODE_OFF)
    {
        if (!openk4a_device_stream_start(device, config->depth_mode, fps_of(config->camera_fps)))
        {
            return K4A_RESULT_FAILED;
        }
        device->depth_stream_on = true;
        device->running_mode = config->depth_mode;
        device->running_fps = fps_of(config->camera_fps);
    }

    if (config->color_resolution != K4A_COLOR_RESOLUTION_OFF)
    {
        /* The colour camera is an open of its own, and it is started after
         * the depth stream so that nothing of it sits between the program
         * starting and the first depth frame. */
        if (device->color == NULL)
        {
            device->color = openk4a_color_create(&device->usb_color, device->index);
        }
        if (device->color == NULL || !openk4a_color_start(device->color, config))
        {
            openk4a_device_stream_stop(device);
            return K4A_RESULT_FAILED;
        }
    }

    device->cameras_started = true;
    return K4A_RESULT_SUCCEEDED;
}

void openk4a_device_set_decode(openk4a_device_t *device, bool ir, bool depth)
{
    if (device != NULL)
    {
        device->decode_ir = ir;
        device->decode_depth = depth;
    }
}

bool openk4a_device_set_depth_stream(openk4a_device_t *device, bool on)
{
    if (device == NULL || !device->cameras_started)
    {
        return false;
    }
    return openk4a_device_run_depth(device, device->config.depth_mode, fps_of(device->config.camera_fps), on);
}

bool openk4a_device_run_depth(openk4a_device_t *device, k4a_depth_mode_t mode, uint32_t fps, bool on)
{
    if (device == NULL || !device->cameras_started || mode == K4A_DEPTH_MODE_OFF)
    {
        return false;
    }
    if (!on)
    {
        if (device->depth_stream_on)
        {
            openk4a_device_stream_stop(device);
            device->depth_stream_on = false;
        }
        return true;
    }
    if (device->depth_stream_on && device->running_mode == mode && device->running_fps == fps)
    {
        return true;
    }
    if (device->depth_stream_on)
    {
        openk4a_device_stream_stop(device);
        device->depth_stream_on = false;
    }
    if (!openk4a_device_stream_start(device, mode, fps))
    {
        return false;
    }
    device->depth_stream_on = true;
    device->running_mode = mode;
    device->running_fps = fps;
    return true;
}

bool openk4a_device_set_color_stream(openk4a_device_t *device, bool on)
{
    if (device == NULL || !device->cameras_started ||
        device->config.color_resolution == K4A_COLOR_RESOLUTION_OFF)
    {
        return false;
    }
    if (on)
    {
        if (device->color == NULL)
        {
            device->color = openk4a_color_create(&device->usb_color, device->index);
        }
        return device->color != NULL && openk4a_color_start(device->color, &device->config);
    }
    if (device->color != NULL)
    {
        openk4a_color_stop(device->color);
    }
    return true;
}

void k4a_device_stop_cameras(k4a_device_t device_handle)
{
    openk4a_device_t *device = (openk4a_device_t *)device_handle;
    if (device == NULL || !device->cameras_started)
    {
        return;
    }
    if (device->color != NULL)
    {
        openk4a_color_stop(device->color);
    }
    openk4a_device_stream_stop(device);
    device->depth_stream_on = false;
    device->cameras_started = false;
}

k4a_wait_result_t k4a_device_get_capture(k4a_device_t device_handle, k4a_capture_t *capture_handle, int32_t timeout_in_ms)
{
    openk4a_device_t *device = (openk4a_device_t *)device_handle;
    if (device == NULL || capture_handle == NULL || !device->cameras_started)
    {
        return K4A_WAIT_RESULT_FAILED;
    }

    const bool want_depth = device->config.depth_mode != K4A_DEPTH_MODE_OFF && device->depth_stream_on;
    const bool want_color = device->config.color_resolution != K4A_COLOR_RESOLUTION_OFF;
    openk4a_frame_info_t info;
    uint64_t device_usec = 0;

    if (want_depth)
    {
        /* A caller that asks for zero wants to know whether a frame is
         * waiting, not to be given the next one. */
        const bool trace = getenv("OPENK4A_TRACE_CAPTURE") != NULL;
        const uint64_t started = openk4a_now_ms();
        if (!openk4a_device_frame(device, &info, timeout_in_ms < 0 ? -1 : timeout_in_ms))
        {
            if (trace)
            {
                openk4a_log(OPENK4A_LOG_INFO, "capture: no depth frame after %llu ms", (unsigned long long)(openk4a_now_ms() - started));
            }
            return timeout_in_ms == 0 ? K4A_WAIT_RESULT_TIMEOUT : K4A_WAIT_RESULT_TIMEOUT;
        }
        if (trace)
        {
            openk4a_log(OPENK4A_LOG_INFO, "capture: depth frame in %llu ms", (unsigned long long)(openk4a_now_ms() - started));
        }
        device_usec = info.exposure_ticks * 100ULL / 9ULL;
    }
    else
    {
        /* Colour only: there is no depth frame to wait for and none to put
         * the colour frame beside, so this waits for the colour camera's own
         * next frame rather than for a length of time. */
        const bool trace = getenv("OPENK4A_TRACE_CAPTURE") != NULL;
        const uint64_t started = openk4a_now_ms();
        if (device->color == NULL || !openk4a_color_wait(device->color, timeout_in_ms))
        {
            if (trace)
            {
                openk4a_log(OPENK4A_LOG_INFO, "capture: no colour frame after %llu ms",
                        (unsigned long long)(openk4a_now_ms() - started));
            }
            return K4A_WAIT_RESULT_TIMEOUT;
        }
    }

    k4a_capture_t capture = NULL;
    if (k4a_capture_create(&capture) != K4A_RESULT_SUCCEEDED)
    {
        return K4A_WAIT_RESULT_FAILED;
    }

    const uint64_t system_nsec = openk4a_now_nsec();
    if (want_depth)
    {
        const int width = device->mode->width;
        const int height = device->mode->height;
        const size_t bytes = (size_t)width * (size_t)height * sizeof(uint16_t);

        k4a_image_t ir = openk4a_image_alloc(K4A_IMAGE_FORMAT_IR16, width, height, width * 2);
        if (ir != NULL)
        {
            memcpy(k4a_image_get_buffer(ir), device->ir, bytes);
            k4a_image_set_device_timestamp_usec(ir, device_usec);
            k4a_image_set_system_timestamp_nsec(ir, system_nsec);
            k4a_capture_set_ir_image(capture, ir);
            k4a_image_release(ir);
        }

        if (device->mode->has_depth)
        {
            k4a_image_t depth = openk4a_image_alloc(K4A_IMAGE_FORMAT_DEPTH16, width, height, width * 2);
            if (depth != NULL)
            {
                memcpy(k4a_image_get_buffer(depth), device->depth, bytes);
                k4a_image_set_device_timestamp_usec(depth, device_usec);
                k4a_image_set_system_timestamp_nsec(depth, system_nsec);
                k4a_capture_set_depth_image(capture, depth);
                k4a_image_release(depth);
            }
        }
        k4a_capture_set_temperature_c(capture, info.sensor_temp_c);
    }

    bool have_something = want_depth;
    if (want_color && device->color != NULL)
    {
        openk4a_color_frame_t frame;
        if (getenv("OPENK4A_TRACE_CAPTURE") != NULL)
        {
            openk4a_log(OPENK4A_LOG_INFO, "capture: taking a colour frame");
        }
        const bool taken = want_depth ? openk4a_color_take(device->color, device_usec, &frame)
                                      : openk4a_color_take_latest(device->color, &frame);
        if (taken)
        {
            k4a_capture_set_color_image(capture, frame.image);
            openk4a_color_release(&frame);
            have_something = true;
        }
        if (getenv("OPENK4A_TRACE_CAPTURE") != NULL)
        {
            openk4a_log(OPENK4A_LOG_INFO, "capture: colour %s", have_something ? "attached" : "none yet");
        }
    }

    if (!have_something)
    {
        k4a_capture_release(capture);
        return K4A_WAIT_RESULT_TIMEOUT;
    }

    *capture_handle = capture;
    return K4A_WAIT_RESULT_SUCCEEDED;
}

/*=============================================================================
  The IMU
=============================================================================*/

k4a_result_t k4a_device_start_imu(k4a_device_t device_handle)
{
    openk4a_device_t *device = (openk4a_device_t *)device_handle;
    if (device == NULL)
    {
        return K4A_RESULT_FAILED;
    }
    if (device->imu == NULL)
    {
        device->imu = openk4a_imu_create(device);
        if (device->imu == NULL)
        {
            return K4A_RESULT_FAILED;
        }
    }
    if (!openk4a_imu_start(device->imu))
    {
        return K4A_RESULT_FAILED;
    }
    device->imu_running = true;
    return K4A_RESULT_SUCCEEDED;
}

void k4a_device_stop_imu(k4a_device_t device_handle)
{
    openk4a_device_t *device = (openk4a_device_t *)device_handle;
    if (device == NULL || device->imu == NULL || !device->imu_running)
    {
        return;
    }
    openk4a_imu_stop(device->imu);
    device->imu_running = false;
}

k4a_wait_result_t k4a_device_get_imu_sample(k4a_device_t device_handle, k4a_imu_sample_t *imu_sample, int32_t timeout_in_ms)
{
    openk4a_device_t *device = (openk4a_device_t *)device_handle;
    if (device == NULL || imu_sample == NULL || device->imu == NULL)
    {
        return K4A_WAIT_RESULT_FAILED;
    }
    if (!openk4a_imu_sample(device->imu, imu_sample, timeout_in_ms))
    {
        return timeout_in_ms == 0 ? K4A_WAIT_RESULT_TIMEOUT : K4A_WAIT_RESULT_TIMEOUT;
    }
    return K4A_WAIT_RESULT_SUCCEEDED;
}

/*=============================================================================
  Colour controls
=============================================================================*/

static openk4a_color_t *color_for_control(k4a_device_t device_handle)
{
    openk4a_device_t *device = (openk4a_device_t *)device_handle;
    if (device == NULL)
    {
        return NULL;
    }
    if (device->color == NULL)
    {
        /* The controls are the camera's, not the stream's, so they are there
         * before the cameras start. */
        device->color = openk4a_color_create(&device->usb_color, device->index);
    }
    return device->color;
}

k4a_result_t k4a_device_get_color_control_capabilities(k4a_device_t device_handle,
                                                       k4a_color_control_command_t command,
                                                       bool *supports_auto,
                                                       int32_t *min_value,
                                                       int32_t *max_value,
                                                       int32_t *step_value,
                                                       int32_t *default_value,
                                                       k4a_color_control_mode_t *default_mode)
{
    if (supports_auto == NULL || min_value == NULL || max_value == NULL || step_value == NULL ||
        default_value == NULL || default_mode == NULL)
    {
        return K4A_RESULT_FAILED;
    }
    openk4a_color_t *color = color_for_control(device_handle);
    if (color == NULL)
    {
        return K4A_RESULT_FAILED;
    }
    openk4a_color_control_capabilities_t capabilities;
    if (!openk4a_color_control_capabilities(color, command, &capabilities))
    {
        return K4A_RESULT_FAILED;
    }
    *supports_auto = capabilities.supports_auto;
    *min_value = capabilities.min_value;
    *max_value = capabilities.max_value;
    *step_value = capabilities.step_value;
    *default_value = capabilities.default_value;
    *default_mode = capabilities.default_mode;
    return K4A_RESULT_SUCCEEDED;
}

k4a_result_t k4a_device_get_color_control(k4a_device_t device_handle,
                                          k4a_color_control_command_t command,
                                          k4a_color_control_mode_t *mode,
                                          int32_t *value)
{
    if (mode == NULL || value == NULL)
    {
        return K4A_RESULT_FAILED;
    }
    openk4a_color_t *color = color_for_control(device_handle);
    if (color == NULL)
    {
        return K4A_RESULT_FAILED;
    }
    return openk4a_color_control_get(color, command, mode, value) ? K4A_RESULT_SUCCEEDED : K4A_RESULT_FAILED;
}

k4a_result_t k4a_device_set_color_control(k4a_device_t device_handle,
                                          k4a_color_control_command_t command,
                                          k4a_color_control_mode_t mode,
                                          int32_t value)
{
    openk4a_color_t *color = color_for_control(device_handle);
    if (color == NULL)
    {
        return K4A_RESULT_FAILED;
    }
    return openk4a_color_control_set(color, command, mode, value) ? K4A_RESULT_SUCCEEDED : K4A_RESULT_FAILED;
}
