/*=============================================================================
  The raw frame, and the IR images that come out of it.

  Every frame the depth MCU streams starts with a 256-byte header and ends
  with a 40-byte footer tagged MRMF; between them the pixels are packed five
  to eight bytes - the top eight bits of five pixels, then their low nibbles
  two to a byte, the sixth spare.

  The twelve bits a pixel gets are a companded magnitude: linear below 256
  and an eight-bit mantissa with a three-bit exponent above it. In a passive
  frame the top two bits are tags; in an active one bit 10 is a sign. The
  numbers here were read off Microsoft's closed depth engine field by field -
  tools/depthengine_open in the SDK tree says how, and tests/ holds the frames
  and its answers that prove it.

      field >= 0xC00                       0         (no data)
      0x800 <= field < 0xC00               value of the low ten bits
      0x400 <= field < 0x800               32767 when the code is 0, else 0
      otherwise                            value of the low ten bits
=============================================================================*/

#include "openk4a.h"

#include <math.h>

/* The five output modes and the raw frames behind them. The frame size says
 * which sensor mode produced it, which is the SDK's own table
 * (src/depth_mcu/depthcommands.h). */
static const openk4a_mode_t OPENK4A_MODES[] = {
    /* mode, sensor, width, height, frame, payload, has depth, nine taps,
     * binning, tap stride (0: pixels + the NFOV family's 160-pixel step) */
    { K4A_DEPTH_MODE_PASSIVE_IR, 3, 1024, 1024, 1678024, 1678336, false, false, 1, 0 },
    { K4A_DEPTH_MODE_NFOV_2X2BINNED, 4, 320, 288, 5310760, 5311488, true, true, 2, 0 },
    { K4A_DEPTH_MODE_NFOV_UNBINNED, 4, 640, 576, 5310760, 5311488, true, true, 1, 0 },
    { K4A_DEPTH_MODE_WFOV_2X2BINNED, 7, 512, 512, 3777232, 3777536, true, true, 1, 0 },
    { K4A_DEPTH_MODE_WFOV_UNBINNED, 5, 1024, 1024, 9438664, 9439232, true, true, 1, 0 },
};

const openk4a_mode_t *openk4a_mode_of(k4a_depth_mode_t mode)
{
    for (size_t i = 0; i < sizeof(OPENK4A_MODES) / sizeof(OPENK4A_MODES[0]); i++)
    {
        if (OPENK4A_MODES[i].mode == mode)
        {
            return &OPENK4A_MODES[i];
        }
    }
    return NULL;
}

const char *openk4a_mode_name(k4a_depth_mode_t mode)
{
    switch (mode)
    {
    case K4A_DEPTH_MODE_PASSIVE_IR:
        return "PASSIVE_IR";
    case K4A_DEPTH_MODE_NFOV_2X2BINNED:
        return "NFOV_2X2BINNED";
    case K4A_DEPTH_MODE_NFOV_UNBINNED:
        return "NFOV_UNBINNED";
    case K4A_DEPTH_MODE_WFOV_2X2BINNED:
        return "WFOV_2X2BINNED";
    case K4A_DEPTH_MODE_WFOV_UNBINNED:
        return "WFOV_UNBINNED";
    default:
        return "OFF";
    }
}

/* Measured on the device, not read from the SDK: each mode's rate was asked
 * for by hand through the depth MCU's own FPS command, and these are the
 * highest the firmware accepted and then actually delivered - the frame's own
 * counter advancing by one a frame at the rate quoted, on a full-size frame,
 * with nothing dropped. Passed over: 10, 20, 24, 25, 40, 45, 48, 50, 54, 60
 * (except mode 3), 66, 72, 75, 90, 100, 120, 144, 150, 180, 240 and 300, all
 * refused with status 2.
 *
 * The SDK cannot reach two of these: its enum is 5/15/30, and it rejects
 * anything else in software. See MiniKinect\README.md. */
uint32_t openk4a_mode_max_fps(const openk4a_mode_t *mode)
{
    if (mode == NULL)
    {
        return 0;
    }
    switch (mode->sensor_mode)
    {
    case 3:  /* PASSIVE_IR: one 1024x1024 exposure a frame, 101 MB/s at 60 */
        return 60;
    case 7:  /* WFOV_2X2BINNED: 113 MB/s at 30 */
        return 30;
    case 5:  /* WFOV_UNBINNED: 9.4 MB a frame, 142 MB/s at 15 */
        return 15;
    default: /* the NFOV pair: 5.3 MB a frame, 159 MB/s at 30 */
        return 30;
    }
}

static bool same_text(const char *a, const char *b)
{
    while (*a != '\0' && *b != '\0')
    {
        char ca = *a++;
        char cb = *b++;
        if (ca >= 'A' && ca <= 'Z')
        {
            ca = (char)(ca - 'A' + 'a');
        }
        if (cb >= 'A' && cb <= 'Z')
        {
            cb = (char)(cb - 'A' + 'a');
        }
        if (ca != cb)
        {
            return false;
        }
    }
    return *a == *b;
}

const openk4a_mode_t *openk4a_mode_by_name(const char *name)
{
    if (name == NULL)
    {
        return NULL;
    }
    for (size_t i = 0; i < sizeof(OPENK4A_MODES) / sizeof(OPENK4A_MODES[0]); i++)
    {
        if (same_text(name, openk4a_mode_name(OPENK4A_MODES[i].mode)))
        {
            return &OPENK4A_MODES[i];
        }
    }
    return NULL;
}

/*=============================================================================
  The header and the footer
=============================================================================*/

bool openk4a_frame_info(const uint8_t *raw, size_t size, openk4a_frame_info_t *info)
{
    memset(info, 0, sizeof(*info));
    if (raw == NULL || size < OPENK4A_FRAME_HEADER_BYTES + OPENK4A_FRAME_FOOTER_BYTES)
    {
        return false;
    }

    const uint8_t *footer = raw + size - OPENK4A_FRAME_FOOTER_BYTES;
    if (memcmp(footer, "MRMF", 4) != 0)
    {
        return false;
    }

    info->frame_number = raw[11];
    info->exposure_ticks = openk4a_le64(footer + 8);
    info->sensor_temp_c = openk4a_le_float(footer + 16);
    info->laser_temp_c[0] = openk4a_le_float(footer + 20);
    info->laser_temp_c[1] = openk4a_le_float(footer + 24);
    info->usb_sof_ticks = openk4a_le64(footer + 32);
    info->valid = true;
    return true;
}

uint32_t openk4a_frame_field(const uint8_t *raw, size_t size, size_t group, size_t position)
{
    const size_t byte = OPENK4A_FRAME_HEADER_BYTES + group * OPENK4A_GROUP_BYTES;
    if (byte + OPENK4A_GROUP_BYTES > size || position >= OPENK4A_PIXELS_PER_GROUP)
    {
        return 0;
    }

    const uint8_t *bytes = raw + byte;
    uint32_t low;
    switch (position)
    {
    case 0:
        low = bytes[5] & 0x0F;
        break;
    case 1:
        low = bytes[5] >> 4;
        break;
    case 2:
        low = bytes[6] & 0x0F;
        break;
    case 3:
        low = bytes[6] >> 4;
        break;
    default:
        low = bytes[7] & 0x0F;
        break;
    }
    return ((uint32_t)bytes[position] << 4) | low;
}

/*=============================================================================
  The companding
=============================================================================*/

/* The twelve bits are a code rather than a number: linear below 256, and
 * above it an eight-bit mantissa whose top bit is named rather than stored,
 * with a three-bit exponent. There are 4096 codes and a frame has ten million
 * taps, so the answer is a table - built once, on the first frame.
 */
static uint16_t openk4a_code_table[4096];
static bool openk4a_code_table_ready;

void openk4a_code_table_init(void)
{
    if (openk4a_code_table_ready)
    {
        return;
    }
    for (uint32_t code = 0; code < 4096; code++)
    {
        if (code < 256)
        {
            openk4a_code_table[code] = (uint16_t)code;
            continue;
        }
        const uint32_t exponent = code >> 7;
        const uint32_t mantissa = 128 + 16 * ((code >> 4) & 7) + (code & 15);
        uint32_t value = mantissa << (exponent - 1);
        if (value > 65535)
        {
            value = 65535;
        }
        openk4a_code_table[code] = (uint16_t)value;
    }
    openk4a_code_table_ready = true;
}

uint16_t openk4a_code_value(uint32_t code)
{
    if (!openk4a_code_table_ready)
    {
        openk4a_code_table_init();
    }
    return openk4a_code_table[code & 0x0FFF];
}

/* A passive frame's tags: 0x400 alone is the saturated value, 0xC00 and up is
 * no data, and bit 11 with bit 10 clear reads as the field without it. */
static uint16_t passive_value(uint32_t field)
{
    const uint32_t high = (field >> 4) & 0xFF;
    const uint32_t low = field & 0x0F;

    if (high >= 192)
    {
        return 0;
    }
    if (high >= 128)
    {
        return openk4a_code_value(((high - 128) << 4) | low);
    }
    if (high >= 64)
    {
        return (high == 64 && low == 0) ? 32767 : 0;
    }
    return openk4a_code_value(field);
}

/* An active frame's twelve bits are the same magnitude with bit 10 as its
 * sign - invisible in a one-sample group, which is why a single-plane sweep
 * looks passive, and the difference between adding and cancelling with two. */
static double active_value(uint32_t field)
{
    const double magnitude = (double)openk4a_code_value(field & 0x3FF);
    return (field & 0x400) ? -magnitude : magnitude;
}

float openk4a_frame_value(uint32_t field)
{
    return (float)active_value(field);
}

/* The engine's three-phase demodulation of one group; as a quadratic form it
 * is exactly (4/9)(a^2 + b^2 + c^2 - ab - ac - bc). */
static double active_amplitude(double a, double b, double c)
{
    const double squared = a * a + b * b + c * c - a * b - a * c - b * c;
    return (2.0 / 3.0) * sqrt(squared > 0.0 ? squared : 0.0);
}

/*=============================================================================
  The IR images
=============================================================================*/

bool openk4a_ir_decode_passive(const uint8_t *raw, size_t size, uint16_t *image, size_t pixels)
{
    if (raw == NULL || size < OPENK4A_FRAME_HEADER_BYTES + OPENK4A_GROUP_BYTES)
    {
        return false;
    }
    openk4a_code_table_init();

    const size_t groups = (size - OPENK4A_FRAME_HEADER_BYTES) / OPENK4A_GROUP_BYTES;
    size_t pixel = 0;
    for (size_t group = 0; group < groups && pixel < pixels; group++)
    {
        /* The five fields of a group are unpacked once and used five times;
         * doing it per field would read the same eight bytes five times. */
        const uint8_t *bytes = raw + OPENK4A_FRAME_HEADER_BYTES + group * OPENK4A_GROUP_BYTES;
        const uint32_t lows = (uint32_t)(bytes[5] & 0x0F) | ((uint32_t)(bytes[5] >> 4) << 4) |
                              ((uint32_t)(bytes[6] & 0x0F) << 8) | ((uint32_t)(bytes[6] >> 4) << 12) |
                              ((uint32_t)(bytes[7] & 0x0F) << 16);
        for (size_t position = 0; position < OPENK4A_PIXELS_PER_GROUP && pixel < pixels; position++)
        {
            const uint32_t field = ((uint32_t)bytes[position] << 4) | ((lows >> (4 * position)) & 0x0F);
            image[pixel++] = passive_value(field);
        }
    }
    return pixel > 0;
}

/* The multi-phase modes' IR: nine taps, three groups of three. The tap stride
 * is a whole number of groups, so every tap sits in the same position of a
 * group. The engine's own answer is 0.9697 of the plain sum of the three
 * amplitudes, the same factor over every frame and every range tried. */
#define OPENK4A_NFOV_TAP_PADDING 160
#define OPENK4A_TAP_SCALE OPENK4A_IR_TAP_SCALE

bool openk4a_ir_decode_taps(const uint8_t *raw, size_t size, const openk4a_mode_t *mode, uint16_t *image)
{
    if (raw == NULL || size < OPENK4A_FRAME_HEADER_BYTES + OPENK4A_GROUP_BYTES || mode == NULL)
    {
        return false;
    }
    openk4a_code_table_init();

    const size_t pixels = (size_t)mode->width * (size_t)mode->height;
    const size_t sub_pixels = openk4a_mode_sub_pixels(mode);
    const size_t tap_stride = mode->tap_stride != 0 ? mode->tap_stride : sub_pixels + OPENK4A_NFOV_TAP_PADDING;
    const size_t tap_groups = tap_stride / OPENK4A_PIXELS_PER_GROUP;
    const size_t groups = (size - OPENK4A_FRAME_HEADER_BYTES) / OPENK4A_GROUP_BYTES;
    if (sub_pixels / OPENK4A_PIXELS_PER_GROUP + (OPENK4A_TAP_COUNT - 1) * tap_groups >= groups)
    {
        return false;
    }

    for (size_t pixel = 0; pixel < pixels; pixel++)
    {
        float values[OPENK4A_TAP_COUNT];
        openk4a_frame_taps(raw, size, mode, pixel, values);
        double total = 0.0;
        for (size_t frequency = 0; frequency < OPENK4A_TAP_COUNT / 3; frequency++)
        {
            total += active_amplitude(values[frequency * 3],
                                      values[frequency * 3 + 1],
                                      values[frequency * 3 + 2]);
        }

        double value = total * OPENK4A_TAP_SCALE;
        if (value > 32767.0)
        {
            value = 32767.0;
        }
        image[pixel] = (uint16_t)(value + 0.5);
    }
    return true;
}

/*=============================================================================
  One frame's taps

  The depth pass wants the nine fields of one pixel together, so it asks for
  them as nine numbers rather than calling openk4a_frame_field nine times.
=============================================================================*/

size_t openk4a_mode_sub_pixels(const openk4a_mode_t *mode)
{
    return (size_t)mode->width * (size_t)mode->height * (size_t)mode->binning * (size_t)mode->binning;
}

void openk4a_frame_taps(const uint8_t *raw,
                    size_t size,
                    const openk4a_mode_t *mode,
                    size_t pixel,
                    float values[OPENK4A_TAP_COUNT])
{
    const size_t sub_width = (size_t)mode->width * (size_t)mode->binning;
    const size_t sub_pixels = openk4a_mode_sub_pixels(mode);
    const size_t tap_stride = mode->tap_stride != 0 ? mode->tap_stride : sub_pixels + OPENK4A_NFOV_TAP_PADDING;
    const size_t tap_groups = tap_stride / OPENK4A_PIXELS_PER_GROUP;
    const size_t row = pixel / (size_t)mode->width;
    const size_t column = pixel % (size_t)mode->width;
    for (size_t tap = 0; tap < OPENK4A_TAP_COUNT; tap++)
    {
        const size_t base_group = tap * tap_groups;
        if (mode->binning == 1)
        {
            const uint32_t field = openk4a_frame_field(raw,
                                                   size,
                                                   base_group + pixel / OPENK4A_PIXELS_PER_GROUP,
                                                   pixel % OPENK4A_PIXELS_PER_GROUP);
            values[tap] = (float)active_value(field);
            continue;
        }
        /* A 2x2 mode halves a larger sub-image: the light of four sensor
         * pixels is binned, so the tap the phase pipeline wants is the mean of
         * their values. */
        double total = 0.0;
        for (int dy = 0; dy < mode->binning; dy++)
        {
            for (int dx = 0; dx < mode->binning; dx++)
            {
                const size_t sub = (row * (size_t)mode->binning + (size_t)dy) * sub_width + column * (size_t)mode->binning + (size_t)dx;
                total += active_value(
                    openk4a_frame_field(raw, size, base_group + sub / OPENK4A_PIXELS_PER_GROUP, sub % OPENK4A_PIXELS_PER_GROUP));
            }
        }
        values[tap] = (float)(total / (double)(mode->binning * mode->binning));
    }
    return;
}

/* The five fields of one group, unpacked once. The five pixels of a group
 * share the same eight bytes, so unpacking per pixel reads them five times;
 * the depth pass walks the frame a group at a time instead. */
void openk4a_frame_group(const uint8_t *raw, size_t size, size_t group, uint32_t fields[OPENK4A_PIXELS_PER_GROUP])
{
    const size_t byte = OPENK4A_FRAME_HEADER_BYTES + group * OPENK4A_GROUP_BYTES;
    if (byte + OPENK4A_GROUP_BYTES > size)
    {
        for (size_t i = 0; i < OPENK4A_PIXELS_PER_GROUP; i++)
        {
            fields[i] = 0;
        }
        return;
    }
    const uint8_t *bytes = raw + byte;
    const uint32_t lows = (uint32_t)(bytes[5] & 0x0F) | ((uint32_t)(bytes[5] >> 4) << 4) |
                          ((uint32_t)(bytes[6] & 0x0F) << 8) | ((uint32_t)(bytes[6] >> 4) << 12) |
                          ((uint32_t)(bytes[7] & 0x0F) << 16);
    for (size_t i = 0; i < OPENK4A_PIXELS_PER_GROUP; i++)
    {
        fields[i] = ((uint32_t)bytes[i] << 4) | ((lows >> (4 * i)) & 0x0F);
    }
}

size_t openk4a_frame_tap_groups(size_t pixels)
{
    return (pixels + OPENK4A_NFOV_TAP_PADDING) / OPENK4A_PIXELS_PER_GROUP;
}
