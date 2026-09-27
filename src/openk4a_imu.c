/*=============================================================================
  The IMU.

  The inertial sensor is behind the colour MCU, not the depth processor: the
  SDK's own imu.c holds a colour MCU handle and reads the sensor over its
  I2C bus. So the stream is a second device's stream endpoint, and the
  samples are the MCU's own packets.

  What comes off the wire is not the measurement, it is the raw reading: a
  temperature, a sensitivity for each sensor, and a count of samples, each a
  three-axis signed reading with its own 90 kHz timestamp. Turning that into
  the sample the API hands back is two steps, and both are the device's own
  calibration:

    1. the reading times the sensitivity the packet carries, in the units the
       API uses - radians a second for the gyroscope, metres a second squared
       for the accelerometer, with the same 9.81 the calibration was made
       with;
    2. the per-unit rectifier: a bias and a mixing matrix per sensor, both
       polynomials in the sensor's temperature, and for the accelerometer a
       second-order scaling term. The model is refreshed when the temperature
       moves more than a quarter of a degree.

  The stream runs on a thread of its own, because a sensor that is read only
  when a program asks would lose its samples between the asks.
=============================================================================*/

#include "openk4a.h"

#include <math.h>

#define OPENK4A_IMU_PAYLOAD_BYTES 512
#define OPENK4A_IMU_QUEUE 512
#define OPENK4A_IMU_TEMPERATURE_DIVISOR 256.0f
#define OPENK4A_IMU_TEMPERATURE_CONSTANT 15.0f
#define OPENK4A_IMU_SCALE_NORMALIZATION 1000000.0f
#define OPENK4A_IMU_GRAVITATIONAL_CONSTANT 9.81f
#define OPENK4A_IMU_RADIANS_PER_DEGREE (3.14159265358979f / 180.0f)
#define OPENK4A_IMU_TICKS_PER_SECOND 90000.0f
#define OPENK4A_IMU_STREAM_TIMEOUT_MS 100

#pragma pack(push, 1)
/* The MCU sends the sample's own timestamp first, then the three signed
 * readings - eight bytes then six, so the struct is fourteen bytes packed.
 * Reading the three first puts the timestamp on the wrong bytes and every
 * value after it out by a sample. */
typedef struct
{
    uint64_t pts; /* 90 kHz ticks */
    int16_t rx;
    int16_t ry;
    int16_t rz;
} openk4a_xyz_t;

typedef struct
{
    uint32_t reporting_rate_in_us;
    uint16_t temperature_sensitivity;
    int16_t value;
} openk4a_imu_temperature_t;

typedef struct
{
    uint16_t sensitivity; /* micro degrees a second */
    uint32_t sample_rate_in_us;
    uint32_t sample_count;
} openk4a_imu_axis_t;

typedef struct
{
    openk4a_imu_temperature_t temperature;
    openk4a_imu_axis_t gyro;
    openk4a_imu_axis_t accel;
} openk4a_imu_payload_t;
#pragma pack(pop)

struct openk4a_imu
{
    openk4a_device_t *device;
    HANDLE thread;
    volatile bool running;

    CRITICAL_SECTION lock;
    k4a_imu_sample_t queue[OPENK4A_IMU_QUEUE];
    int head;
    int count;
    bool failed;

    /* The rectifier, refreshed whenever the temperature moves. */
    float bias_gyro[3];
    float bias_accel[3];
    float mixing_gyro[9];
    float mixing_accel[9];
    float applied_temperature;
};

/* The SDK's own three helpers, which the rectifier is written in terms of. */
static float eval_poly_3(float x, const float coefficients[4])
{
    return coefficients[0] + x * (coefficients[1] + x * (coefficients[2] + x * coefficients[3]));
}

static void affine_3(const float a[9], const float x[3], const float b[3], float out[3])
{
    for (int row = 0; row < 3; row++)
    {
        out[row] = a[row * 3] * x[0] + a[row * 3 + 1] * x[1] + a[row * 3 + 2] * x[2] + b[row];
    }
}

static void quadratic_3(const float a[9], const float b_matrix[9], const float x[3], const float b[3], float out[3])
{
    float affine[3];
    affine_3(a, x, b, affine);
    const float squared[3] = { x[0] * x[0], x[1] * x[1], x[2] * x[2] };
    affine_3(b_matrix, squared, affine, out);
}

static void refresh_rectifier(const openk4a_imu_calibration_t *calibration, float temperature, float *bias, float *mixing)
{
    for (int row = 0; row < 3; row++)
    {
        bias[row] = eval_poly_3(temperature, &calibration->bias_temperature_model[row * OPENK4A_IMU_MODEL_COEFFICIENTS]);
        for (int column = 0; column < 3; column++)
        {
            const int index = 3 * row + column;
            mixing[index] =
                eval_poly_3(temperature, &calibration->mixing_matrix_temperature_model[index * OPENK4A_IMU_MODEL_COEFFICIENTS]);
        }
    }
}

static void imu_update_temperature(openk4a_imu_t *imu, float temperature)
{
    const openk4a_calibration_t *calibration = &imu->device->calibration;
    imu->applied_temperature = temperature;
    refresh_rectifier(&calibration->gyro, temperature, imu->bias_gyro, imu->mixing_gyro);
    refresh_rectifier(&calibration->accel, temperature, imu->bias_accel, imu->mixing_accel);
}

/*=============================================================================
  The stream
=============================================================================*/

static void imu_push(openk4a_imu_t *imu, const k4a_imu_sample_t *sample)
{
    EnterCriticalSection(&imu->lock);
    if (imu->count == OPENK4A_IMU_QUEUE)
    {
        /* Full: the oldest goes, because a program that is behind wants the
         * newest motion rather than a backlog. */
        imu->head = (imu->head + 1) % OPENK4A_IMU_QUEUE;
        imu->count--;
    }
    imu->queue[(imu->head + imu->count) % OPENK4A_IMU_QUEUE] = *sample;
    imu->count++;
    LeaveCriticalSection(&imu->lock);
}

static void imu_parse(openk4a_imu_t *imu, const uint8_t *packet, size_t length)
{
    if (length < sizeof(openk4a_imu_payload_t))
    {
        return;
    }
    const openk4a_imu_payload_t *payload = (const openk4a_imu_payload_t *)packet;
    const openk4a_xyz_t *gyro = (const openk4a_xyz_t *)(packet + sizeof(openk4a_imu_payload_t));
    const openk4a_xyz_t *accel = gyro + payload->gyro.sample_count;

    const size_t needed = sizeof(openk4a_imu_payload_t) +
                          ((size_t)payload->gyro.sample_count + (size_t)payload->accel.sample_count) *
                              sizeof(openk4a_xyz_t);
    if (length < needed)
    {
        return;
    }

    const float temperature =
        (float)payload->temperature.value / OPENK4A_IMU_TEMPERATURE_DIVISOR + OPENK4A_IMU_TEMPERATURE_CONSTANT;
    const float gyro_scale =
        (float)payload->gyro.sensitivity * OPENK4A_IMU_RADIANS_PER_DEGREE / OPENK4A_IMU_SCALE_NORMALIZATION;
    const float accel_scale =
        (float)payload->accel.sensitivity * OPENK4A_IMU_GRAVITATIONAL_CONSTANT / OPENK4A_IMU_SCALE_NORMALIZATION;

    const uint32_t count = payload->gyro.sample_count < payload->accel.sample_count ? payload->gyro.sample_count
                                                                                    : payload->accel.sample_count;
    for (uint32_t i = 0; i < count; i++)
    {
        k4a_imu_sample_t sample;
        memset(&sample, 0, sizeof(sample));
        sample.temperature = temperature;
        sample.gyro_sample.xyz.x = (float)gyro[i].rx * gyro_scale;
        sample.gyro_sample.xyz.y = (float)gyro[i].ry * gyro_scale;
        sample.gyro_sample.xyz.z = (float)gyro[i].rz * gyro_scale;
        sample.gyro_timestamp_usec = (uint64_t)((float)gyro[i].pts * 1000000.0f / OPENK4A_IMU_TICKS_PER_SECOND);
        sample.acc_sample.xyz.x = (float)accel[i].rx * accel_scale;
        sample.acc_sample.xyz.y = (float)accel[i].ry * accel_scale;
        sample.acc_sample.xyz.z = (float)accel[i].rz * accel_scale;
        sample.acc_timestamp_usec = (uint64_t)((float)accel[i].pts * 1000000.0f / OPENK4A_IMU_TICKS_PER_SECOND);
        imu_push(imu, &sample);
    }
}

static DWORD WINAPI imu_thread(LPVOID parameter)
{
    openk4a_imu_t *imu = (openk4a_imu_t *)parameter;
    openk4a_usb_t *usb = &imu->device->usb_color;
    uint8_t packet[OPENK4A_IMU_PAYLOAD_BYTES];

    typedef BOOL (*read_pipe_t)(openk4a_winusb_handle_t, UCHAR, UCHAR *, DWORD, DWORD *, void *);
    read_pipe_t read_pipe = (read_pipe_t)openk4a_win.read_pipe;

    while (imu->running)
    {
        DWORD got = 0;
        if (read_pipe(usb->winusb, usb->endpoint_stream, packet, (DWORD)sizeof(packet), &got, NULL))
        {
            if (got > 0)
            {
                imu_parse(imu, packet, got);
            }
            continue;
        }
        const DWORD error = GetLastError();
        /* A transfer timeout is how a quiet sensor reports that it had
         * nothing; anything else means the stream is gone. */
        if (error == ERROR_SEM_TIMEOUT || error == ERROR_OPERATION_ABORTED || error == ERROR_IO_INCOMPLETE)
        {
            continue;
        }
        openk4a_log(OPENK4A_LOG_ERROR, "the IMU stream failed (%lu)", error);
        break;
    }

    EnterCriticalSection(&imu->lock);
    imu->failed = true;
    LeaveCriticalSection(&imu->lock);
    return 0;
}

/*=============================================================================
  The IMU's own interface
  ==========================================================================*/

openk4a_imu_t *openk4a_imu_create(openk4a_device_t *device)
{
    openk4a_imu_t *imu = (openk4a_imu_t *)openk4a_alloc_zero(sizeof(openk4a_imu_t));
    if (imu == NULL)
    {
        return NULL;
    }
    imu->device = device;
    imu->head = 0;
    imu->count = 0;
    InitializeCriticalSection(&imu->lock);

    /* The rectifier starts at the temperature the calibration was made at,
     * which is what the SDK does before the first sample arrives. */
    imu_update_temperature(imu, device->calibration.gyro.temperature_in_c);
    return imu;
}

void openk4a_imu_destroy(openk4a_imu_t *imu)
{
    if (imu == NULL)
    {
        return;
    }
    openk4a_imu_stop(imu);
    DeleteCriticalSection(&imu->lock);
    openk4a_free(imu);
}

bool openk4a_imu_start(openk4a_imu_t *imu)
{
    if (imu == NULL || imu->running)
    {
        return imu != NULL;
    }

    /* A quiet sensor makes a read time out, which is how the thread wakes up
     * often enough to notice it has been asked to stop. */
    (void)openk4a_usb_set_timeout(&imu->device->usb_color, imu->device->usb_color.endpoint_stream,
                              OPENK4A_IMU_STREAM_TIMEOUT_MS);

    if (!openk4a_color_mcu_imu_start(imu->device))
    {
        openk4a_log(OPENK4A_LOG_ERROR, "the colour MCU would not start the IMU");
        return false;
    }

    imu->running = true;
    imu->failed = false;
    imu->thread = CreateThread(NULL, 0, imu_thread, imu, 0, NULL);
    if (imu->thread == NULL)
    {
        imu->running = false;
        (void)openk4a_color_mcu_imu_stop(imu->device);
        openk4a_log(OPENK4A_LOG_ERROR, "the IMU stream thread would not start");
        return false;
    }
    return true;
}

void openk4a_imu_stop(openk4a_imu_t *imu)
{
    if (imu == NULL || !imu->running)
    {
        return;
    }
    imu->running = false;

    /* Wake the read up rather than waiting out its timeout. */
    if (openk4a_win.abort_pipe != NULL && imu->device->usb_color.winusb != NULL)
    {
        typedef BOOL (*abort_pipe_t)(openk4a_winusb_handle_t, UCHAR);
        ((abort_pipe_t)openk4a_win.abort_pipe)(imu->device->usb_color.winusb,
                                           imu->device->usb_color.endpoint_stream);
    }
    if (imu->thread != NULL)
    {
        WaitForSingleObject(imu->thread, 2000);
        CloseHandle(imu->thread);
        imu->thread = NULL;
    }
    (void)openk4a_color_mcu_imu_stop(imu->device);
    (void)openk4a_usb_set_timeout(&imu->device->usb_color, imu->device->usb_color.endpoint_stream, OPENK4A_CMD_TIMEOUT_MS);
}

bool openk4a_imu_sample(openk4a_imu_t *imu, k4a_imu_sample_t *sample, int timeout_ms)
{
    if (imu == NULL || sample == NULL)
    {
        return false;
    }

    const uint64_t deadline = timeout_ms < 0 ? UINT64_MAX : openk4a_now_ms() + (uint64_t)timeout_ms;
    for (;;)
    {
        EnterCriticalSection(&imu->lock);
        if (imu->count > 0)
        {
            *sample = imu->queue[imu->head];
            imu->head = (imu->head + 1) % OPENK4A_IMU_QUEUE;
            imu->count--;
            LeaveCriticalSection(&imu->lock);
            break;
        }
        const bool failed = imu->failed;
        LeaveCriticalSection(&imu->lock);
        if (failed || timeout_ms == 0 || (timeout_ms > 0 && openk4a_now_ms() >= deadline))
        {
            return false;
        }
        Sleep(1);
    }

    /* The rectifier is refreshed when the sensor's temperature has moved far
     * enough to matter, which is the SDK's own quarter of a degree. */
    const float temperature = sample->temperature;
    if (temperature > imu->applied_temperature + 0.25f || temperature < imu->applied_temperature - 0.25f)
    {
        imu_update_temperature(imu, temperature);
    }

    float gyro[3] = { sample->gyro_sample.xyz.x, sample->gyro_sample.xyz.y, sample->gyro_sample.xyz.z };
    float accel[3] = { sample->acc_sample.xyz.x, sample->acc_sample.xyz.y, sample->acc_sample.xyz.z };

    affine_3(imu->mixing_gyro, gyro, imu->bias_gyro, gyro);
    quadratic_3(imu->mixing_accel,
                imu->device->calibration.accel.second_order_scaling,
                accel,
                imu->bias_accel,
                accel);

    sample->gyro_sample.xyz.x = gyro[0];
    sample->gyro_sample.xyz.y = gyro[1];
    sample->gyro_sample.xyz.z = gyro[2];
    sample->acc_sample.xyz.x = accel[0];
    sample->acc_sample.xyz.y = accel[1];
    sample->acc_sample.xyz.z = accel[2];
    return true;
}
