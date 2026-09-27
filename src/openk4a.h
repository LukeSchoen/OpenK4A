/*=============================================================================
  OpenK4A - the Azure Kinect DK, in portable C, with no k4a.dll, no depth engine
  and no driver of its own.

  This header is the whole internal interface. The modules are:

    openk4a_util.c        bytes, files, time, memory, a small JSON reader
    openk4a_win.c         the Windows entry points this tree calls, resolved at
                      run time so that neither compiler needs an import library
    openk4a_usb.c         SetupAPI + WinUSB: find a camera, open it, talk to it
    openk4a_depth_mcu.c    the depth processor's own command set
    openk4a_color_mcu.c   the colour MCU's command set (system config, IMU, jacks)
    openk4a_frame.c       the raw frame: header, footer, the 5-to-8 packing
    openk4a_depth_model.c the depth pass and its per-mode tables
    openk4a_calibration.c the device's JSON, denormalized to a k4a calibration
    openk4a_image.c       k4a_image_t
    openk4a_capture.c     k4a_capture_t
    openk4a_allocator.c   k4a_set_allocator, and every allocation this tree makes
    openk4a_logging.c     k4a_set_debug_message_handler
    openk4a_transform.c   the transformation engine and the calibration maths
    openk4a_imu.c         the IMU stream and its per-unit rectifier
    openk4a_color.c       colour frames, undecoded MJPG, over Media Foundation
    openk4a_device.c      k4a_device_*, the public face, in one place

  The rules the code follows, because they are what keeps it small:

    - no dependency that is not on the machine already: Windows' own WinUSB
      for the depth processor and the colour MCU, Media Foundation for the
      colour camera, nothing else;
    - no header set the dev compiler does not carry: every Windows type and
      entry point this tree needs is named in openk4a_win.h;
    - the depth pass is single-threaded, with the read overlapped so the
      transfer never sits between two frames' arithmetic.
=============================================================================*/

#ifndef OPENK4A_H
#define OPENK4A_H

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include <k4a/k4a.h>

#include "openk4a_win.h"

/*=============================================================================
  1. Shared types
=============================================================================*/

/* Every fallible call in this tree returns one of these. It is the same shape
 * as k4a_result_t on purpose: the public layer passes it straight through. */
typedef int openk4a_result_t;

#define OPENK4A_OK 0
#define OPENK4A_FAILED 1
#define OPENK4A_TIMEOUT 2
#define OPENK4A_TOO_SMALL 3
#define OPENK4A_DONE 4      /* a loop asked for more and there is no more */

#define OPENK4A_OK_(expr) ((expr) == OPENK4A_OK)
#define OPENK4A_FAILED_(expr) ((expr) != OPENK4A_OK)

/* A float larger than any error this tree compares against, spelled out
 * rather than taken from float.h: a program that links this tree may be built
 * with a header set that does not carry it. */
#define OPENK4A_DISTANT 3.0e38f

/*=============================================================================
  2. util - bytes, files, time, memory, JSON
=============================================================================*/

uint32_t openk4a_le32(const uint8_t *bytes);
uint64_t openk4a_le64(const uint8_t *bytes);
float openk4a_le_float(const uint8_t *bytes);

/* The host clock the K4A system timestamps use, in nanoseconds. */
uint64_t openk4a_now_nsec(void);
uint64_t openk4a_now_ms(void);

/* A monotonic nanosecond clock off the performance counter, for timing a
 * step: openk4a_now_nsec is whole milliseconds by design - it is the clock the
 * system timestamps are on - and cannot resolve a frame's own internals. */
uint64_t openk4a_now_fine_nsec(void);

/* Reading a whole file, and the PGM form this tree writes and reads back. */
uint8_t *openk4a_file_read(const char *path, size_t *size);
bool openk4a_file_write(const char *path, const void *data, size_t size);
bool openk4a_write_pgm(const char *path, const uint16_t *image, int width, int height);
uint16_t *openk4a_read_pgm(const char *path, int *width, int *height);

/* The directory the running executable is in, without a trailing slash. */
bool openk4a_exe_dir(char *out, size_t out_size);

/* Writes a file of the device's own data beside the tree, under cache/. What
 * goes there is what the camera handed over - the calibration, the block -
 * so a run can be looked at afterwards without the camera. */
bool openk4a_cache_write(const char *name, const void *data, size_t size);

/* A small JSON reader: enough for the device's calibration document, which is
 * objects, arrays, numbers and strings. It parses into one arena and hands
 * back nodes; the whole document is freed at once. */
typedef struct openk4a_json openk4a_json_t;

openk4a_json_t *openk4a_json_parse(const char *text, size_t length);
void openk4a_json_free(openk4a_json_t *document);
/* A key of an object, or NULL. */
openk4a_json_t *openk4a_json_get(openk4a_json_t *object, const char *key);
/* The nth element of an array, or NULL. */
openk4a_json_t *openk4a_json_at(openk4a_json_t *array, size_t index);
size_t openk4a_json_count(openk4a_json_t *array);
bool openk4a_json_is_number(openk4a_json_t *node);
bool openk4a_json_is_string(openk4a_json_t *node);
double openk4a_json_number(openk4a_json_t *node);
const char *openk4a_json_string(openk4a_json_t *node);
/* An array of numbers into a float buffer; returns how many were read. */
size_t openk4a_json_floats(openk4a_json_t *array, float *out, size_t count);

/*=============================================================================
  3. logging
=============================================================================*/

/* The same order and the same numbers as k4a_log_level_t, so a message this
 * tree logs and a message a program's handler compares are the same value. */
typedef enum
{
    OPENK4A_LOG_CRITICAL = K4A_LOG_LEVEL_CRITICAL,
    OPENK4A_LOG_ERROR = K4A_LOG_LEVEL_ERROR,
    OPENK4A_LOG_WARNING = K4A_LOG_LEVEL_WARNING,
    OPENK4A_LOG_INFO = K4A_LOG_LEVEL_INFO,
    OPENK4A_LOG_TRACE = K4A_LOG_LEVEL_TRACE,
    OPENK4A_LOG_OFF = K4A_LOG_LEVEL_OFF
} openk4a_log_level_t;

void openk4a_log(openk4a_log_level_t level, const char *format, ...);
void openk4a_log_set_handler(k4a_logging_message_cb_t *handler, void *context);
void openk4a_log_set_level(openk4a_log_level_t level);

/*=============================================================================
  4. memory
=============================================================================*/

/* Everything this tree allocates goes through these, so that a program that
 * installs its own allocator with k4a_set_allocator owns all of it. */
void *openk4a_alloc(size_t size);
void *openk4a_alloc_zero(size_t size);
void openk4a_free(void *pointer);
void openk4a_allocator_set(k4a_memory_allocate_cb_t *allocate, k4a_memory_destroy_cb_t *free_fn);
bool openk4a_allocator_installed(void);

/*=============================================================================
  5. usb - SetupAPI and WinUSB
=============================================================================*/

#define OPENK4A_CMD_PACKET_TYPE 0x06022009u
#define OPENK4A_CMD_PACKET_TYPE_RESPONSE 0x0A6FE000u
#define OPENK4A_CMD_PACKET_DATA_BYTES 128
#define OPENK4A_CMD_TIMEOUT_MS 2000
/* The MCU ends a transfer with a zero-length packet, so a read that comes
 * back empty is read again. */
#define OPENK4A_READ_ATTEMPTS 16

/* Which endpoints a device's command set uses. The colour device's numbers
 * are its own: the SDK names them IN/OUT the other way round, so they are
 * spelled out here rather than inferred. */
typedef struct
{
    uint16_t pid;
    int interface_mi;      /* the composite interface, or -1 when there is none */
    uint8_t endpoint_out;  /* commands and data to the device */
    uint8_t endpoint_in;   /* responses and data from the device */
    uint8_t endpoint_stream;
} openk4a_usb_id_t;

extern const openk4a_usb_id_t OPENK4A_USB_DEPTH;  /* 045E:097C, depth processor */
extern const openk4a_usb_id_t OPENK4A_USB_COLOR;  /* 045E:097D MI_02, colour MCU */

typedef struct
{
    HANDLE event;
    OVERLAPPED overlapped;
    uint8_t *buffer;
    size_t size;
    DWORD transferred;
    bool pending;
    bool ready;
} openk4a_io_t;

typedef struct
{
    HANDLE file;
    void *winusb; /* WINUSB_INTERFACE_HANDLE */
    uint8_t endpoint_out;
    uint8_t endpoint_in;
    uint8_t endpoint_stream;
    uint32_t transaction_id;
    char path[512];
    char serial[128];
} openk4a_usb_t;

/* How many cameras of this kind are attached. */
uint32_t openk4a_usb_count(const openk4a_usb_id_t *id);

/* The path of the nth camera, or false. */
bool openk4a_usb_path(const openk4a_usb_id_t *id, uint32_t index, char *out, size_t out_size);

/* The serial number the SDK reports is the one in the device path. */
void openk4a_usb_serial_from_path(const char *path, char *serial, size_t serial_size);

/* Opens it. Every endpoint the command set needs is checked for. */
bool openk4a_usb_open(openk4a_usb_t *usb, const openk4a_usb_id_t *id, uint32_t index);
void openk4a_usb_close(openk4a_usb_t *usb);

/* One command transaction: the header and its data out, the payload in, then
 * the response that carries this command's own transaction id. */
bool openk4a_usb_command(openk4a_usb_t *usb,
                     uint32_t command,
                     const void *command_data,
                     size_t command_data_size,
                     const void *tx_data,
                     size_t tx_size,
                     void *rx_data,
                     size_t rx_size,
                     size_t *bytes_read,
                     uint32_t *status);

static inline bool openk4a_usb_read_command(openk4a_usb_t *usb, uint32_t command, void *data, size_t size, size_t *read)
{
    return openk4a_usb_command(usb, command, NULL, 0, NULL, 0, data, size, read, NULL);
}

static inline bool openk4a_usb_write_command(openk4a_usb_t *usb, uint32_t command, const void *data, size_t size)
{
    return openk4a_usb_command(usb, command, data, size, NULL, 0, NULL, 0, NULL, NULL);
}

bool openk4a_usb_set_timeout(openk4a_usb_t *usb, uint8_t endpoint, uint32_t milliseconds);

/* Overlapped stream reads. A frame is posted, and the next one can be posted
 * as soon as the first has arrived, so the transfer runs while the frame is
 * being decoded. */
bool openk4a_usb_io_post(openk4a_usb_t *usb, openk4a_io_t *io);
int openk4a_usb_io_wait(openk4a_usb_t *usb, openk4a_io_t *io, int timeout_ms); /* OPENK4A_OK, OPENK4A_TIMEOUT, OPENK4A_FAILED */
void openk4a_usb_io_cancel(openk4a_usb_t *usb, openk4a_io_t *io);
void openk4a_usb_io_close(openk4a_usb_t *usb, openk4a_io_t *io);

/*=============================================================================
  6. the raw frame
=============================================================================*/

#define OPENK4A_FRAME_HEADER_BYTES 256
#define OPENK4A_FRAME_FOOTER_BYTES 40
#define OPENK4A_PIXELS_PER_GROUP 5
#define OPENK4A_GROUP_BYTES 8
/* The multi-phase modes hand one output pixel nine taps: three frequencies of
 * three samples 120 degrees apart. */
#define OPENK4A_TAP_COUNT 9

typedef struct
{
    uint64_t exposure_ticks;  /* 90 kHz */
    uint64_t usb_sof_ticks;   /* 90 kHz */
    float sensor_temp_c;
    float laser_temp_c[2];
    uint8_t frame_number;
    bool valid;
} openk4a_frame_info_t;

/* The footer's own clock and temperatures, if this is a whole frame. */
bool openk4a_frame_info(const uint8_t *raw, size_t size, openk4a_frame_info_t *info);

/* A frame's twelve-bit fields, at the group/position an index names. */
uint32_t openk4a_frame_field(const uint8_t *raw, size_t size, size_t group, size_t position);

/* The five fields of one packed group, and how far apart the taps of one
 * pixel are - the depth pass walks the frame a group at a time so that the
 * eight bytes of a group are unpacked once rather than five times. */
void openk4a_frame_group(const uint8_t *raw, size_t size, size_t group, uint32_t fields[OPENK4A_PIXELS_PER_GROUP]);
size_t openk4a_frame_tap_groups(size_t pixels);

/* The companded magnitude of a code, and the same with the tags resolved.
 * One 4096-entry table, built once, replaces the shifts and the divide. */
uint16_t openk4a_code_value(uint32_t code);
void openk4a_code_table_init(void);

/*=============================================================================
  7. the modes
=============================================================================*/

typedef struct
{
    k4a_depth_mode_t mode;
    uint32_t sensor_mode;
    int width;
    int height;
    size_t frame_bytes;   /* what the MCU sends */
    size_t payload_bytes; /* rounded up to the USB3 packet size */
    bool has_depth;
    bool nine_taps;       /* the multi-phase layout: nine sub-images */
    int binning;          /* 2 when the mode halves a larger sub-image's grid */
    size_t tap_stride;    /* sub-image pixels between taps, 0 to derive it */
} openk4a_mode_t;

const openk4a_mode_t *openk4a_mode_of(k4a_depth_mode_t mode);
const openk4a_mode_t *openk4a_mode_by_name(const char *name);
const char *openk4a_mode_name(k4a_depth_mode_t mode);

/* The fastest rate this firmware will take for a mode, measured by asking the
 * device rather than by reading the SDK's table. The SDK has three rates in
 * its enum and refuses anything else before the device is asked; the device
 * takes a fourth - sixty, for the IR-only mode - and runs the wide binned mode
 * at thirty where the SDK asks for fifteen. See MiniKinect's probe raw. */
uint32_t openk4a_mode_max_fps(const openk4a_mode_t *mode);

/* All nine taps of one output pixel, as signed magnitudes: the depth pass and
 * the IR decode both want the companded value rather than the twelve-bit code,
 * and a mode that bins 2x2 wants the mean of the four sensor pixels' values -
 * not of their codes, because the companding is not linear. */
void openk4a_frame_taps(const uint8_t *raw,
                    size_t size,
                    const openk4a_mode_t *mode,
                    size_t pixel,
                    float values[OPENK4A_TAP_COUNT]);

/* The same for one twelve-bit code. */
float openk4a_frame_value(uint32_t field);

/* One sub-exposure on its own: the frame's tap `tap`, companded and signed,
 * as floats, sub_pixels of them. The nine taps of a frame were taken one
 * after another inside it - three frequencies of three phases - so asking for
 * one at a time is asking for the scene 3.7 ms apart rather than 33 ms apart.
 * This reads only that tap's span of the frame, so a caller showing them in
 * sequence does one ninth of the unpacking between one and the next. */
bool openk4a_frame_tap_plane(const uint8_t *raw,
                         size_t size,
                         const openk4a_mode_t *mode,
                         size_t tap,
                         float *values,
                         size_t count);

/* The pixels of one sub-image: the output grid, or the grid a binned mode
 * reads it from. */
size_t openk4a_mode_sub_pixels(const openk4a_mode_t *mode);

/*=============================================================================
  8. calibration

  The device's own document, parsed once when the camera is opened. The SDK
  keeps its per-unit inertial model in a private header; this is the same
  document's InertialSensors entry, which is what the IMU rectifier runs on:
  a bias polynomial and a mixing matrix per axis, both functions of the
  sensor's temperature, plus the second-order scaling the accelerometer
  wants. The fields are in the SDK's own order so the arithmetic in openk4a_imu.c
  can be read beside it.
=============================================================================*/

#define OPENK4A_IMU_MODEL_COEFFICIENTS 4

typedef struct
{
    k4a_calibration_extrinsics_t depth_to_imu;
    float noise[3 * 2];
    float temperature_in_c;
    float bias_temperature_model[3 * OPENK4A_IMU_MODEL_COEFFICIENTS];
    float mixing_matrix_temperature_model[9 * OPENK4A_IMU_MODEL_COEFFICIENTS];
    float second_order_scaling[9];
    float bias_uncertainty[3];
    float temperature_bounds[2];
    uint32_t model_type_mask;
} openk4a_imu_calibration_t;

typedef struct
{
    bool valid;
    k4a_calibration_camera_t depth;
    k4a_calibration_camera_t color;
    openk4a_imu_calibration_t gyro;
    openk4a_imu_calibration_t accel;
    char *json;
    size_t json_size;
} openk4a_calibration_t;

/*=============================================================================
  9. the depth model
=============================================================================*/

#define OPENK4A_DEPTH_MAX_PIXELS (1024 * 1024)

typedef struct openk4a_depth_model openk4a_depth_model_t;

/* The model for a device and mode: from the device's own calibration block
 * when that yields one, from the file in the search path when it is there,
 * and from the calibration compiled into this tree as the last resort. */
openk4a_depth_model_t *openk4a_depth_model_open(k4a_depth_mode_t mode,
                                        const uint8_t *calibration_block,
                                        size_t calibration_block_size,
                                        const openk4a_calibration_t *calibration,
                                        char *where,
                                        size_t where_size);
void openk4a_depth_model_free(openk4a_depth_model_t *model);
bool openk4a_depth_model_valid(const openk4a_depth_model_t *model);
const char *openk4a_depth_model_where(const openk4a_depth_model_t *model);

/* The depth pass for the model's own mode. The model is not const: the phase
 * correction is rebuilt when the frame's own temperature has moved far enough
 * from the one the tables were built at. */

/* The two halves of the pass, timed off the performance counter: the
 * projection, which turns nine taps into three unit vectors and their
 * amplitudes, and the neighbourhood filter, which is what is left of the
 * frame's time. */
typedef struct
{
    uint64_t projection_nsec;
    uint64_t filter_nsec;
    uint64_t total_nsec;
} openk4a_depth_times_t;

void openk4a_depth_decode(openk4a_depth_model_t *model, const uint8_t *raw, size_t size, uint16_t *depth);

/* The same, with the nine-tap IR plane as well. The projection works the
 * three amplitudes out anyway, so a caller that wants both planes pays one
 * pass over the frame rather than two. Either destination may be NULL. */
void openk4a_depth_decode_full(openk4a_depth_model_t *model,
                           const uint8_t *raw,
                           size_t size,
                           uint16_t *depth,
                           uint16_t *ir,
                           openk4a_depth_times_t *times);

/* The pass the fast one replaced, kept as the thing it is held to. */
void openk4a_depth_decode_reference(openk4a_depth_model_t *model,
                                const uint8_t *raw,
                                size_t size,
                                uint16_t *depth,
                                openk4a_depth_times_t *times);

void openk4a_depth_decode_timed(openk4a_depth_model_t *model,
                            const uint8_t *raw,
                            size_t size,
                            uint16_t *depth,
                            openk4a_depth_times_t *times);

/* IR: PASSIVE_IR's single 1024x1024 plane, and the nine-tap modes' sum. */
bool openk4a_ir_decode_passive(const uint8_t *raw, size_t size, uint16_t *image, size_t pixels);
bool openk4a_ir_decode_taps(const uint8_t *raw, size_t size, const openk4a_mode_t *mode, uint16_t *image);

/* The nine-tap IR is the sum of the three frequencies' amplitudes scaled by
 * this - the engine's own answer is a constant fraction of the plain sum,
 * over every frame and every range tried. It lives here because the depth
 * pass produces the same amplitudes and hands the plane over for free. */
#define OPENK4A_IR_TAP_SCALE 0.9697f

/* Parses the device's own calibration document. */
bool openk4a_calibration_parse(const char *json, size_t json_size, openk4a_calibration_t *calibration);
void openk4a_calibration_free(openk4a_calibration_t *calibration);

/* The camera, denormalized into the grid a mode produces. */
bool openk4a_calibration_for_depth_mode(const openk4a_calibration_t *calibration,
                                    k4a_depth_mode_t mode,
                                    k4a_calibration_camera_t *out);
bool openk4a_calibration_for_color_resolution(const openk4a_calibration_t *calibration,
                                          k4a_color_resolution_t resolution,
                                          k4a_calibration_camera_t *out);

/* The public k4a calibration for a pair of modes, extrinsics and all. */
bool openk4a_calibration_build(const openk4a_calibration_t *calibration,
                           k4a_depth_mode_t depth_mode,
                           k4a_color_resolution_t color_resolution,
                           k4a_calibration_t *out);

/* The grid a depth mode produces, for callers that only want the size. */
bool openk4a_calibration_grid(k4a_depth_mode_t mode, int *width, int *height);
bool openk4a_color_grid(k4a_color_resolution_t resolution, int *width, int *height);

/* The camera model itself, which the transformation engine builds on: a point
 * to a pixel and back, in a camera's own coordinates. */
bool openk4a_calibration_project(const k4a_calibration_camera_t *camera, const float point3d_mm[3], float point2d[2],
                             int *valid);
bool openk4a_calibration_unproject(const k4a_calibration_camera_t *camera, const float point2d[2], float depth_mm,
                               float point3d_mm[3], int *valid);

/*=============================================================================
  10. images and captures
=============================================================================*/

typedef void(openk4a_image_free_fn)(void *buffer, void *context);

k4a_image_t openk4a_image_create(k4a_image_format_t format,
                             int width,
                             int height,
                             int stride_bytes,
                             size_t size,
                             uint8_t *buffer,
                             openk4a_image_free_fn *free_fn,
                             void *free_context);
k4a_image_t openk4a_image_alloc(k4a_image_format_t format, int width, int height, int stride_bytes);
void openk4a_image_add_ref(k4a_image_t image);
void openk4a_image_dec_ref(k4a_image_t image);

/* The depth and colour grids a mode's images carry. */
int openk4a_format_stride(k4a_image_format_t format, int width);

/* The IMU's own image inside a capture, which is how the SDK's IMU path is
 * shaped: one sample is one capture with one image in it. */
void openk4a_capture_set_imu_image(k4a_capture_t capture_handle, k4a_image_t image_handle);
k4a_image_t openk4a_capture_get_imu_image(k4a_capture_t capture_handle);

/*=============================================================================
  11. the device
=============================================================================*/

typedef struct openk4a_color openk4a_color_t;
typedef struct openk4a_imu openk4a_imu_t;

typedef struct openk4a_device
{
    /* The two command devices. The depth processor is opened with the camera
     * and stays open; the colour MCU is opened with it too, because the
     * calibration block, the IMU and the sync jacks all live behind it. */
    openk4a_usb_t usb_depth;
    openk4a_usb_t usb_color;
    bool color_mcu_open;

    uint32_t index;
    char serial[128];

    openk4a_calibration_t calibration;
    uint8_t *calibration_block;
    size_t calibration_block_size;

    k4a_device_configuration_t config;
    const openk4a_mode_t *mode;
    bool cameras_started;
    bool depth_stream_on;
    k4a_depth_mode_t running_mode;
    uint32_t running_fps;

    /* The depth stream: two buffers, one being filled while the other is
     * decoded. */
    openk4a_io_t io[2];
    int io_index;
    bool io_armed;
    uint8_t *frame;        /* the frame being decoded */
    size_t frame_size;
    bool frame_valid;

    uint16_t *ir;
    uint16_t *depth;
    openk4a_depth_model_t *model;

    /* What a frame is worth decoding. A viewer that is showing the colour
     * camera has no use for either plane, and one that is showing the IR has
     * no use for the filter; both are on by default, because the SDK's own
     * capture carries both. */
    bool decode_ir;
    bool decode_depth;

    /* The IMU and the colour camera are each a second device open; neither is
     * on the depth stream's start-up path. */
    openk4a_color_t *color;
    openk4a_imu_t *imu;
    bool imu_running;

    uint64_t start_time_nsec;
    bool started;
} openk4a_device_t;

/* The command sets, one per device. */
#define OPENK4A_DEPTH_CMD_VERSION_GET 0x00000002u
#define OPENK4A_DEPTH_CMD_START 0x00000009u
#define OPENK4A_DEPTH_CMD_STOP 0x0000000Au
#define OPENK4A_DEPTH_CMD_NV_DATA_GET 0x00000022u
#define OPENK4A_DEPTH_CMD_MODE_SET 0x000000E1u
#define OPENK4A_DEPTH_CMD_POWER_OFF 0x000000EFu
#define OPENK4A_DEPTH_CMD_POWER_ON 0x000000F0u
#define OPENK4A_DEPTH_CMD_STREAM_START 0x000000F1u
#define OPENK4A_DEPTH_CMD_STREAM_STOP 0x000000F2u
#define OPENK4A_DEPTH_CMD_FPS_SET 0x00000103u
#define OPENK4A_DEPTH_CMD_READ_CALIBRATION_DATA 0x00000111u
#define OPENK4A_DEPTH_CMD_READ_PRODUCT_SN 0x00000115u
#define OPENK4A_DEPTH_CMD_COMPONENT_VERSION_GET 0x00000201u

#define OPENK4A_NV_IR_SENSOR_CALIBRATION 2u
#define OPENK4A_NV_ACCELEROMETER 10u
#define OPENK4A_NV_PRODUCT_TYPE 23u

#define OPENK4A_COLOR_CMD_RESET 0x80000000u
#define OPENK4A_COLOR_CMD_SET_SYS_CFG 0x80000001u
#define OPENK4A_COLOR_CMD_GET_SYS_CFG 0x80000002u
#define OPENK4A_COLOR_CMD_IMU_STREAM_START 0x80000003u
#define OPENK4A_COLOR_CMD_IMU_STREAM_STOP 0x80000004u
#define OPENK4A_COLOR_CMD_GET_JACK_STATE 0x80000006u

/* The device's own calibration block: the IR sensor's, which the closed depth
 * engine is created with, and which this tree expands into its own tables. */
bool openk4a_depth_mcu_calibration_block(openk4a_device_t *device);
bool openk4a_depth_mcu_calibration_json(openk4a_device_t *device);
/* What the depth processor says it is running. The layout is the firmware's
 * own, which is why it is packed rather than four-byte aligned. */
#pragma pack(push, 1)
typedef struct
{
    uint8_t rgb_major;
    uint8_t rgb_minor;
    uint16_t rgb_build;
    uint8_t depth_major;
    uint8_t depth_minor;
    uint16_t depth_build;
    uint8_t audio_major;
    uint8_t audio_minor;
    uint16_t audio_build;
    uint16_t depth_sensor_major;
    uint16_t depth_sensor_minor;
    uint8_t build_config;   /* 0 release, 1 debug */
    uint8_t signature_type; /* 0 Microsoft, 1 test, 2 unsigned */
} openk4a_firmware_versions_t;
#pragma pack(pop)

bool openk4a_depth_mcu_version(openk4a_device_t *device, openk4a_firmware_versions_t *version);
bool openk4a_depth_mcu_mode(openk4a_device_t *device, uint32_t sensor_mode);
bool openk4a_depth_mcu_fps(openk4a_device_t *device, uint32_t fps);
bool openk4a_depth_mcu_stream_start(openk4a_device_t *device);
bool openk4a_depth_mcu_stream_stop(openk4a_device_t *device);
bool openk4a_depth_mcu_nv_data(openk4a_device_t *device, uint32_t tag, void *data, size_t size, size_t *read);

/* Bringing the depth stream up: mode, calibration, tables, fps, start. */
bool openk4a_device_stream_start(openk4a_device_t *device, k4a_depth_mode_t mode, uint32_t fps);
void openk4a_device_stream_stop(openk4a_device_t *device);
/* Chooses which planes a frame decodes. Cheap to change between frames. */
void openk4a_device_set_decode(openk4a_device_t *device, bool ir, bool depth);
/* Starts or stops the colour camera on its own, leaving the depth stream
 * running. The two cameras share one cable, and the colour stream at thirty
 * frames a second is enough to cost the depth stream every other frame; a
 * viewer showing one of them at a time should ask for one of them at a time.
 * False when the configuration has no colour stream in it. */
bool openk4a_device_set_color_stream(openk4a_device_t *device, bool on);
/* The same for the depth processor. The two cameras share one cable, and the
 * depth stream is 5.3 MB a frame: a viewer showing the colour camera at thirty
 * frames a second, with the depth stream still running beside it, gets both at
 * fifteen. False when the configuration has no depth stream in it. */
bool openk4a_device_set_depth_stream(openk4a_device_t *device, bool on);
/* The same, saying which mode and which rate. A rate is a plain number
 * because the device takes more of them than the API's enum has: the IR-only
 * mode runs at sixty, which the SDK cannot ask for. Restarts the stream when
 * either has changed. */
bool openk4a_device_run_depth(openk4a_device_t *device, k4a_depth_mode_t mode, uint32_t fps, bool on);
/* One frame, decoded into device->ir and device->depth. */
bool openk4a_device_frame(openk4a_device_t *device, openk4a_frame_info_t *info, int timeout_ms);

/* The colour MCU: the synchronisation configuration, the jack state, and the
 * commands that start and stop the IMU. */
bool openk4a_color_mcu_open(openk4a_device_t *device);
void openk4a_color_mcu_close(openk4a_device_t *device);
bool openk4a_color_mcu_set_sys_cfg(openk4a_device_t *device, const k4a_device_configuration_t *config);
bool openk4a_color_mcu_get_jacks(openk4a_device_t *device, bool *in_jack, bool *out_jack);
bool openk4a_color_mcu_imu_start(openk4a_device_t *device);
bool openk4a_color_mcu_imu_stop(openk4a_device_t *device);

/*=============================================================================
  12. the colour camera
=============================================================================*/

typedef struct
{
    k4a_image_t image;         /* referenced; NULL when the queue was empty */
    uint64_t device_timestamp_usec;
} openk4a_color_frame_t;

/* What k4a_device_get_color_control_capabilities reports, held together so
 * the camera can answer it without the caller passing six pointers again. */
typedef struct
{
    bool supports_auto;
    int32_t min_value;
    int32_t max_value;
    int32_t step_value;
    int32_t default_value;
    k4a_color_control_mode_t default_mode;
    bool valid;
} openk4a_color_control_capabilities_t;

openk4a_color_t *openk4a_color_create(const openk4a_usb_t *color_mcu, uint32_t device_index);
void openk4a_color_destroy(openk4a_color_t *color);
bool openk4a_color_start(openk4a_color_t *color, const k4a_device_configuration_t *config);
void openk4a_color_stop(openk4a_color_t *color);
/* Takes the colour frame nearest a depth frame's device time, or NULL. */
bool openk4a_color_take(openk4a_color_t *color, uint64_t depth_device_usec, openk4a_color_frame_t *out);
/* The newest colour frame, and everything older thrown away. What a caller
 * that is showing the colour camera and has no depth frame to put beside it
 * wants. */
bool openk4a_color_take_latest(openk4a_color_t *color, openk4a_color_frame_t *out);
/* Waits for a colour frame to arrive. True when one is there. */
bool openk4a_color_wait(openk4a_color_t *color, int timeout_ms);
void openk4a_color_release(openk4a_color_frame_t *frame);

bool openk4a_color_control_capabilities(openk4a_color_t *color,
                                    k4a_color_control_command_t command,
                                    openk4a_color_control_capabilities_t *out);
bool openk4a_color_control_get(openk4a_color_t *color, k4a_color_control_command_t command, k4a_color_control_mode_t *mode, int32_t *value);
bool openk4a_color_control_set(openk4a_color_t *color, k4a_color_control_command_t command, k4a_color_control_mode_t mode, int32_t value);

/*=============================================================================
  13. the IMU
=============================================================================*/

openk4a_imu_t *openk4a_imu_create(openk4a_device_t *device);
void openk4a_imu_destroy(openk4a_imu_t *imu);
bool openk4a_imu_start(openk4a_imu_t *imu);
void openk4a_imu_stop(openk4a_imu_t *imu);
bool openk4a_imu_sample(openk4a_imu_t *imu, k4a_imu_sample_t *sample, int timeout_ms);

/*=============================================================================
  14. transformation
=============================================================================*/

/* Everything the transformation engine and the calibration maths need, with
 * no handle of its own: the public layer in openk4a_device.c wraps these. */
openk4a_result_t openk4a_calibration_math_3d_to_3d(const k4a_calibration_t *calibration,
                                           const float source_point3d_mm[3],
                                           k4a_calibration_type_t source_camera,
                                           k4a_calibration_type_t target_camera,
                                           float target_point3d_mm[3]);
openk4a_result_t openk4a_calibration_math_2d_to_3d(const k4a_calibration_t *calibration,
                                           const float source_point2d[2],
                                           float source_depth_mm,
                                           k4a_calibration_type_t source_camera,
                                           k4a_calibration_type_t target_camera,
                                           float target_point3d_mm[3],
                                           int *valid);
openk4a_result_t openk4a_calibration_math_3d_to_2d(const k4a_calibration_t *calibration,
                                           const float source_point3d_mm[3],
                                           k4a_calibration_type_t source_camera,
                                           k4a_calibration_type_t target_camera,
                                           float target_point2d[2],
                                           int *valid);
openk4a_result_t openk4a_calibration_math_2d_to_2d(const k4a_calibration_t *calibration,
                                           const float source_point2d[2],
                                           float source_depth_mm,
                                           k4a_calibration_type_t source_camera,
                                           k4a_calibration_type_t target_camera,
                                           float target_point2d[2],
                                           int *valid);
openk4a_result_t openk4a_calibration_math_color_2d_to_depth_2d(const k4a_calibration_t *calibration,
                                                       const float source_point2d[2],
                                                       k4a_image_t depth_image,
                                                       float target_point2d[2],
                                                       int *valid);

typedef struct openk4a_transformation openk4a_transformation_t;

openk4a_transformation_t *openk4a_transformation_create(const k4a_calibration_t *calibration);
void openk4a_transformation_destroy(openk4a_transformation_t *transformation);

openk4a_result_t openk4a_transformation_depth_to_color(openk4a_transformation_t *transformation,
                                               k4a_image_t depth_image,
                                               k4a_image_t transformed_depth_image);
openk4a_result_t openk4a_transformation_color_to_depth(openk4a_transformation_t *transformation,
                                               k4a_image_t depth_image,
                                               k4a_image_t color_image,
                                               k4a_image_t transformed_color_image);
openk4a_result_t openk4a_transformation_depth_to_point_cloud(openk4a_transformation_t *transformation,
                                                     k4a_image_t depth_image,
                                                     k4a_calibration_type_t camera,
                                                     k4a_image_t point_cloud_image);

#endif /* OPENK4A_H */
