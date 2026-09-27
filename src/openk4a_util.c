/*=============================================================================
  The small things: little-endian reads, the clock, files, the PGM form this
  tree writes its planes in, and a JSON reader for the device's calibration.

  The JSON reader is here rather than in a library because the tree has one
  document to read: the camera's own calibration, which is objects, arrays,
  numbers and strings and nothing else. It parses into one arena so the whole
  document is freed in one call.
=============================================================================*/

#include "openk4a.h"

#include <math.h>

/*-----------------------------------------------------------------------------
  bytes
---------------------------------------------------------------------------*/

uint32_t openk4a_le32(const uint8_t *bytes)
{
    return (uint32_t)bytes[0] | ((uint32_t)bytes[1] << 8) | ((uint32_t)bytes[2] << 16) |
           ((uint32_t)bytes[3] << 24);
}

uint64_t openk4a_le64(const uint8_t *bytes)
{
    return (uint64_t)openk4a_le32(bytes) | ((uint64_t)openk4a_le32(bytes + 4) << 32);
}

float openk4a_le_float(const uint8_t *bytes)
{
    const uint32_t bits = openk4a_le32(bytes);
    float value;
    memcpy(&value, &bits, sizeof(value));
    return value;
}

/*-----------------------------------------------------------------------------
  the clock
---------------------------------------------------------------------------*/

/* GetTickCount64 is the clock every timestamp in this tree is measured
 * against: the same one the SDK's system timestamps use, and the one a
 * program has when it times a frame itself. One compiler's headers declare
 * it and the other's do not, and an implicit declaration would hand back
 * only the low 32 bits - which wrap every seven weeks - so it is looked up
 * on first use instead. */
typedef ULONGLONG(WINAPI *tick_count64_t)(void);

static uint64_t host_ticks(void)
{
    static tick_count64_t counter;
    if (counter == NULL)
    {
        counter = (tick_count64_t)(void *)GetProcAddress(GetModuleHandleA("kernel32.dll"), "GetTickCount64");
        if (counter == NULL)
        {
            return (uint64_t)GetTickCount();
        }
    }
    return (uint64_t)counter();
}

uint64_t openk4a_now_ms(void)
{
    return host_ticks();
}

uint64_t openk4a_now_nsec(void)
{
    return host_ticks() * 1000000ULL;
}

/* The timestamps above move in whole milliseconds, which is the right
 * resolution for a clock a frame is stamped with and far too coarse to see
 * inside a frame: the depth pass's two halves both round to the same tick.
 * A step is measured with the performance counter instead. */
typedef BOOL(WINAPI *query_counter_t)(LARGE_INTEGER *);

static query_counter_t fine_counter;
static LARGE_INTEGER fine_origin;
static double fine_nsec_per_tick = 1.0e6;
static bool fine_ready;

static void fine_clock_init(void)
{
    LARGE_INTEGER frequency;
    fine_ready = true;
    fine_counter = (query_counter_t)(void *)GetProcAddress(GetModuleHandleA("kernel32.dll"),
                                                           "QueryPerformanceCounter");
    if (fine_counter == NULL || !fine_counter(&fine_origin) ||
        !QueryPerformanceFrequency(&frequency) || frequency.QuadPart == 0)
    {
        /* No counter: fall back to the millisecond clock so that a timing run
         * reports whole milliseconds rather than nothing at all. */
        fine_counter = NULL;
        fine_origin.QuadPart = 0;
        fine_nsec_per_tick = 1.0e6;
        return;
    }
    fine_nsec_per_tick = 1.0e9 / (double)frequency.QuadPart;
}

uint64_t openk4a_now_fine_nsec(void)
{
    if (!fine_ready)
    {
        fine_clock_init();
    }
    if (fine_counter == NULL)
    {
        return host_ticks() * 1000000ULL;
    }
    LARGE_INTEGER now;
    fine_counter(&now);
    return (uint64_t)((double)((uint64_t)now.QuadPart - (uint64_t)fine_origin.QuadPart) * fine_nsec_per_tick);
}

/*-----------------------------------------------------------------------------
  files
---------------------------------------------------------------------------*/

uint8_t *openk4a_file_read(const char *path, size_t *size)
{
    if (size != NULL)
    {
        *size = 0;
    }
    FILE *file = fopen(path, "rb");
    if (file == NULL)
    {
        return NULL;
    }
    if (fseek(file, 0, SEEK_END) != 0)
    {
        fclose(file);
        return NULL;
    }
    const long length = ftell(file);
    if (length < 0 || fseek(file, 0, SEEK_SET) != 0)
    {
        fclose(file);
        return NULL;
    }
    uint8_t *data = (uint8_t *)openk4a_alloc((size_t)length + 1);
    if (data == NULL)
    {
        fclose(file);
        return NULL;
    }
    const size_t read = fread(data, 1, (size_t)length, file);
    fclose(file);
    data[read] = 0;
    if (size != NULL)
    {
        *size = read;
    }
    return data;
}

bool openk4a_file_write(const char *path, const void *data, size_t size)
{
    FILE *file = fopen(path, "wb");
    if (file == NULL)
    {
        return false;
    }
    const bool written = fwrite(data, 1, size, file) == size;
    fclose(file);
    return written;
}

bool openk4a_write_pgm(const char *path, const uint16_t *image, int width, int height)
{
    FILE *file = fopen(path, "wb");
    if (file == NULL)
    {
        return false;
    }
    fprintf(file, "P5\n%d %d\n65535\n", width, height);
    /* Big endian, as the format says, and in one write per row rather than
     * one per pixel: a frame is three hundred thousand pixels. */
    const size_t pixels = (size_t)width * (size_t)height;
    uint8_t *row = (uint8_t *)openk4a_alloc((size_t)width * 2);
    if (row == NULL)
    {
        fclose(file);
        return false;
    }
    bool written = true;
    for (int y = 0; y < height && written; y++)
    {
        for (int x = 0; x < width; x++)
        {
            const uint16_t value = image[(size_t)y * (size_t)width + (size_t)x];
            row[x * 2] = (uint8_t)(value >> 8);
            row[x * 2 + 1] = (uint8_t)(value & 0xFF);
        }
        written = fwrite(row, 1, (size_t)width * 2, file) == (size_t)width * 2;
    }
    openk4a_free(row);
    fclose(file);
    return written && pixels > 0;
}

uint16_t *openk4a_read_pgm(const char *path, int *width, int *height)
{
    size_t size = 0;
    uint8_t *data = openk4a_file_read(path, &size);
    if (data == NULL)
    {
        return NULL;
    }
    if (size < 16 || data[0] != 'P' || data[1] != '5')
    {
        openk4a_free(data);
        return NULL;
    }
    size_t at = 2;
    int values[3] = { 0, 0, 0 };
    for (int v = 0; v < 3; v++)
    {
        while (at < size && (data[at] == ' ' || data[at] == '\n' || data[at] == '\r' || data[at] == '\t'))
        {
            at++;
        }
        while (at < size && data[at] >= '0' && data[at] <= '9')
        {
            values[v] = values[v] * 10 + (data[at] - '0');
            at++;
        }
    }
    at++;
    const int w = values[0];
    const int h = values[1];
    if (w <= 0 || h <= 0 || at + (size_t)w * (size_t)h * 2 > size)
    {
        openk4a_free(data);
        return NULL;
    }
    uint16_t *image = (uint16_t *)openk4a_alloc((size_t)w * (size_t)h * sizeof(uint16_t));
    if (image == NULL)
    {
        openk4a_free(data);
        return NULL;
    }
    for (size_t i = 0; i < (size_t)w * (size_t)h; i++)
    {
        image[i] = (uint16_t)(((uint16_t)data[at] << 8) | data[at + 1]);
        at += 2;
    }
    openk4a_free(data);
    if (width != NULL)
    {
        *width = w;
    }
    if (height != NULL)
    {
        *height = h;
    }
    return image;
}

bool openk4a_exe_dir(char *out, size_t out_size)
{
    if (GetModuleFileNameA(NULL, out, (DWORD)out_size) == 0)
    {
        out[0] = '\0';
        return false;
    }
    char *slash = strrchr(out, '\\');
    if (slash == NULL)
    {
        return false;
    }
    *slash = '\0';
    return true;
}

bool openk4a_cache_write(const char *name, const void *data, size_t size)
{
    char exe[512];
    if (!openk4a_exe_dir(exe, sizeof(exe)))
    {
        return false;
    }
    char directory[512];
    snprintf(directory, sizeof(directory), "%s\\..\\cache", exe);
    CreateDirectoryA(directory, NULL);
    char path[512];
    snprintf(path, sizeof(path), "%s\\%s", directory, name);
    return openk4a_file_write(path, data, size);
}

/*-----------------------------------------------------------------------------
  JSON

  Nodes come out of a bump arena whose blocks are freed together. Strings are
  decoded in place over the document, which is why the document is a copy.
---------------------------------------------------------------------------*/

typedef enum
{
    OPENK4A_JSON_NULL,
    OPENK4A_JSON_BOOL,
    OPENK4A_JSON_NUMBER,
    OPENK4A_JSON_STRING,
    OPENK4A_JSON_ARRAY,
    OPENK4A_JSON_OBJECT
} openk4a_json_type_t;

struct openk4a_json
{
    openk4a_json_type_t type;
    double number;
    bool boolean;
    char *string;
    openk4a_json_t **items; /* the elements, or the values when it is an object */
    char **keys;        /* an object's keys, NULL in an array */
    size_t count;
    struct openk4a_json_document *document; /* set on the root, so a node knows its arena */
};

typedef struct openk4a_json_block
{
    struct openk4a_json_block *next;
    size_t used;
    size_t size;
    uint8_t data[1];
} openk4a_json_block_t;

typedef struct
{
    char *text;
    size_t length;
    size_t at;
    openk4a_json_block_t *blocks;
    bool failed;
} openk4a_json_parser_t;

typedef struct openk4a_json_document
{
    openk4a_json_parser_t parser;
    openk4a_json_t root;
} openk4a_json_document_t;

#define OPENK4A_JSON_BLOCK_SIZE 8192

static void *json_alloc(openk4a_json_parser_t *parser, size_t size)
{
    size = (size + 15u) & ~(size_t)15u;
    if (parser->blocks == NULL || parser->blocks->used + size > parser->blocks->size)
    {
        const size_t block_size = size > OPENK4A_JSON_BLOCK_SIZE ? size : OPENK4A_JSON_BLOCK_SIZE;
        openk4a_json_block_t *block =
            (openk4a_json_block_t *)openk4a_alloc(sizeof(openk4a_json_block_t) + block_size);
        if (block == NULL)
        {
            parser->failed = true;
            return NULL;
        }
        block->next = parser->blocks;
        block->used = 0;
        block->size = block_size;
        parser->blocks = block;
    }
    void *result = parser->blocks->data + parser->blocks->used;
    parser->blocks->used += size;
    return result;
}

static void json_skip_whitespace(openk4a_json_parser_t *parser)
{
    while (parser->at < parser->length)
    {
        const char c = parser->text[parser->at];
        if (c == ' ' || c == '\t' || c == '\n' || c == '\r')
        {
            parser->at++;
        }
        else
        {
            break;
        }
    }
}

static openk4a_json_t *json_new(openk4a_json_parser_t *parser, openk4a_json_type_t type)
{
    openk4a_json_t *node = (openk4a_json_t *)json_alloc(parser, sizeof(openk4a_json_t));
    if (node == NULL)
    {
        return NULL;
    }
    memset(node, 0, sizeof(*node));
    node->type = type;
    return node;
}

static openk4a_json_t *json_parse_value(openk4a_json_parser_t *parser);

/* A string literal, decoded over the document itself: escapes are turned into
 * the bytes they mean, so no second buffer is needed. */
static char *json_parse_string(openk4a_json_parser_t *parser)
{
    if (parser->at >= parser->length || parser->text[parser->at] != '"')
    {
        parser->failed = true;
        return NULL;
    }
    parser->at++;
    char *out = parser->text + parser->at;
    char *write = out;
    while (parser->at < parser->length)
    {
        const char c = parser->text[parser->at++];
        if (c == '"')
        {
            *write = '\0';
            return out;
        }
        if (c != '\\')
        {
            *write++ = c;
            continue;
        }
        if (parser->at >= parser->length)
        {
            break;
        }
        const char escape = parser->text[parser->at++];
        switch (escape)
        {
        case '"':
        case '\\':
        case '/':
            *write++ = escape;
            break;
        case 'b':
            *write++ = '\b';
            break;
        case 'f':
            *write++ = '\f';
            break;
        case 'n':
            *write++ = '\n';
            break;
        case 'r':
            *write++ = '\r';
            break;
        case 't':
            *write++ = '\t';
            break;
        case 'u':
        {
            /* The calibration document is ASCII, so a code point above 127
             * cannot mean anything this tree reads; encode it as UTF-8 for
             * completeness and move on. */
            if (parser->at + 4 > parser->length)
            {
                parser->failed = true;
                return NULL;
            }
            unsigned int code = 0;
            for (int i = 0; i < 4; i++)
            {
                const char hex = parser->text[parser->at++];
                code <<= 4;
                if (hex >= '0' && hex <= '9')
                {
                    code |= (unsigned int)(hex - '0');
                }
                else if (hex >= 'a' && hex <= 'f')
                {
                    code |= (unsigned int)(hex - 'a' + 10);
                }
                else if (hex >= 'A' && hex <= 'F')
                {
                    code |= (unsigned int)(hex - 'A' + 10);
                }
                else
                {
                    parser->failed = true;
                    return NULL;
                }
            }
            if (code < 0x80)
            {
                *write++ = (char)code;
            }
            else if (code < 0x800)
            {
                *write++ = (char)(0xC0 | (code >> 6));
                *write++ = (char)(0x80 | (code & 0x3F));
            }
            else
            {
                *write++ = (char)(0xE0 | (code >> 12));
                *write++ = (char)(0x80 | ((code >> 6) & 0x3F));
                *write++ = (char)(0x80 | (code & 0x3F));
            }
            break;
        }
        default:
            parser->failed = true;
            return NULL;
        }
    }
    parser->failed = true;
    return NULL;
}

static bool json_parse_number(openk4a_json_parser_t *parser, double *out)
{
    const size_t start = parser->at;
    if (parser->at < parser->length &&
        (parser->text[parser->at] == '-' || parser->text[parser->at] == '+'))
    {
        parser->at++;
    }
    while (parser->at < parser->length)
    {
        const char c = parser->text[parser->at];
        if ((c >= '0' && c <= '9') || c == '.' || c == 'e' || c == 'E' || c == '+' || c == '-')
        {
            parser->at++;
        }
        else
        {
            break;
        }
    }
    if (parser->at == start)
    {
        parser->failed = true;
        return false;
    }
    char saved = parser->text[parser->at];
    parser->text[parser->at] = '\0';
    *out = strtod(parser->text + start, NULL);
    parser->text[parser->at] = saved;
    return true;
}

static bool json_parse_keyword(openk4a_json_parser_t *parser, const char *word)
{
    const size_t length = strlen(word);
    if (parser->at + length > parser->length || memcmp(parser->text + parser->at, word, length) != 0)
    {
        parser->failed = true;
        return false;
    }
    parser->at += length;
    return true;
}

static openk4a_json_t *json_parse_array(openk4a_json_parser_t *parser)
{
    parser->at++; /* '[' */
    openk4a_json_t *node = json_new(parser, OPENK4A_JSON_ARRAY);
    if (node == NULL)
    {
        return NULL;
    }
    size_t capacity = 0;
    openk4a_json_t **items = NULL;
    for (;;)
    {
        json_skip_whitespace(parser);
        if (parser->at < parser->length && parser->text[parser->at] == ']')
        {
            parser->at++;
            break;
        }
        if (node->count == capacity)
        {
            const size_t grown = capacity == 0 ? 16 : capacity * 2;
            openk4a_json_t **bigger = (openk4a_json_t **)json_alloc(parser, grown * sizeof(openk4a_json_t *));
            if (bigger == NULL)
            {
                return NULL;
            }
            if (items != NULL)
            {
                memcpy(bigger, items, node->count * sizeof(openk4a_json_t *));
            }
            items = bigger;
            capacity = grown;
        }
        openk4a_json_t *element = json_parse_value(parser);
        if (element == NULL)
        {
            return NULL;
        }
        items[node->count++] = element;
        json_skip_whitespace(parser);
        if (parser->at < parser->length && parser->text[parser->at] == ',')
        {
            parser->at++;
            continue;
        }
    }
    node->items = items;
    return node;
}

static openk4a_json_t *json_parse_object(openk4a_json_parser_t *parser)
{
    parser->at++; /* '{' */
    openk4a_json_t *node = json_new(parser, OPENK4A_JSON_OBJECT);
    if (node == NULL)
    {
        return NULL;
    }
    size_t capacity = 0;
    openk4a_json_t **items = NULL;
    char **keys = NULL;
    for (;;)
    {
        json_skip_whitespace(parser);
        if (parser->at < parser->length && parser->text[parser->at] == '}')
        {
            parser->at++;
            break;
        }
        if (node->count == capacity)
        {
            const size_t grown = capacity == 0 ? 16 : capacity * 2;
            openk4a_json_t **bigger_items = (openk4a_json_t **)json_alloc(parser, grown * sizeof(openk4a_json_t *));
            char **bigger_keys = (char **)json_alloc(parser, grown * sizeof(char *));
            if (bigger_items == NULL || bigger_keys == NULL)
            {
                return NULL;
            }
            if (items != NULL)
            {
                memcpy(bigger_items, items, node->count * sizeof(openk4a_json_t *));
                memcpy(bigger_keys, keys, node->count * sizeof(char *));
            }
            items = bigger_items;
            keys = bigger_keys;
            capacity = grown;
        }
        char *key = json_parse_string(parser);
        if (key == NULL)
        {
            return NULL;
        }
        json_skip_whitespace(parser);
        if (parser->at >= parser->length || parser->text[parser->at] != ':')
        {
            parser->failed = true;
            return NULL;
        }
        parser->at++;
        openk4a_json_t *value = json_parse_value(parser);
        if (value == NULL)
        {
            return NULL;
        }
        keys[node->count] = key;
        items[node->count] = value;
        node->count++;
        json_skip_whitespace(parser);
        if (parser->at < parser->length && parser->text[parser->at] == ',')
        {
            parser->at++;
            continue;
        }
    }
    node->items = items;
    node->keys = keys;
    return node;
}

static openk4a_json_t *json_parse_value(openk4a_json_parser_t *parser)
{
    json_skip_whitespace(parser);
    if (parser->at >= parser->length)
    {
        parser->failed = true;
        return NULL;
    }
    const char c = parser->text[parser->at];
    if (c == '{')
    {
        return json_parse_object(parser);
    }
    if (c == '[')
    {
        return json_parse_array(parser);
    }
    if (c == '"')
    {
        openk4a_json_t *node = json_new(parser, OPENK4A_JSON_STRING);
        if (node == NULL)
        {
            return NULL;
        }
        node->string = json_parse_string(parser);
        return node->string == NULL ? NULL : node;
    }
    if (c == 't' || c == 'f')
    {
        openk4a_json_t *node = json_new(parser, OPENK4A_JSON_BOOL);
        if (node == NULL)
        {
            return NULL;
        }
        if (c == 't')
        {
            node->boolean = json_parse_keyword(parser, "true");
        }
        else
        {
            node->boolean = false;
            (void)json_parse_keyword(parser, "false");
        }
        return node;
    }
    if (c == 'n')
    {
        openk4a_json_t *node = json_new(parser, OPENK4A_JSON_NULL);
        if (node == NULL || !json_parse_keyword(parser, "null"))
        {
            return NULL;
        }
        return node;
    }
    openk4a_json_t *node = json_new(parser, OPENK4A_JSON_NUMBER);
    if (node == NULL || !json_parse_number(parser, &node->number))
    {
        return NULL;
    }
    return node;
}

static void json_document_free(openk4a_json_document_t *document)
{
    if (document == NULL)
    {
        return;
    }
    openk4a_json_block_t *block = document->parser.blocks;
    while (block != NULL)
    {
        openk4a_json_block_t *next = block->next;
        openk4a_free(block);
        block = next;
    }
    openk4a_free(document->parser.text);
    openk4a_free(document);
}

openk4a_json_t *openk4a_json_parse(const char *text, size_t length)
{
    openk4a_json_document_t *document = (openk4a_json_document_t *)openk4a_alloc_zero(sizeof(openk4a_json_document_t));
    if (document == NULL)
    {
        return NULL;
    }
    document->parser.text = (char *)openk4a_alloc(length + 1);
    if (document->parser.text == NULL)
    {
        openk4a_free(document);
        return NULL;
    }
    memcpy(document->parser.text, text, length);
    document->parser.text[length] = '\0';
    document->parser.length = length;

    openk4a_json_t *parsed = json_parse_value(&document->parser);
    json_skip_whitespace(&document->parser);
    if (document->parser.failed || parsed == NULL)
    {
        json_document_free(document);
        return NULL;
    }
    document->root = *parsed;
    document->root.document = document;
    return &document->root;
}

void openk4a_json_free(openk4a_json_t *document)
{
    if (document == NULL)
    {
        return;
    }
    json_document_free(document->document);
}

openk4a_json_t *openk4a_json_get(openk4a_json_t *object, const char *key)
{
    if (object == NULL || object->type != OPENK4A_JSON_OBJECT)
    {
        return NULL;
    }
    for (size_t i = 0; i < object->count; i++)
    {
        if (strcmp(object->keys[i], key) == 0)
        {
            return object->items[i];
        }
    }
    return NULL;
}

openk4a_json_t *openk4a_json_at(openk4a_json_t *array, size_t index)
{
    if (array == NULL || array->type != OPENK4A_JSON_ARRAY || index >= array->count)
    {
        return NULL;
    }
    return array->items[index];
}

size_t openk4a_json_count(openk4a_json_t *array)
{
    if (array == NULL || (array->type != OPENK4A_JSON_ARRAY && array->type != OPENK4A_JSON_OBJECT))
    {
        return 0;
    }
    return array->count;
}

bool openk4a_json_is_number(openk4a_json_t *node)
{
    return node != NULL && node->type == OPENK4A_JSON_NUMBER;
}

bool openk4a_json_is_string(openk4a_json_t *node)
{
    return node != NULL && node->type == OPENK4A_JSON_STRING;
}

double openk4a_json_number(openk4a_json_t *node)
{
    return node != NULL && node->type == OPENK4A_JSON_NUMBER ? node->number : 0.0;
}

const char *openk4a_json_string(openk4a_json_t *node)
{
    return node != NULL && node->type == OPENK4A_JSON_STRING ? node->string : NULL;
}

size_t openk4a_json_floats(openk4a_json_t *array, float *out, size_t count)
{
    if (array == NULL || array->type != OPENK4A_JSON_ARRAY)
    {
        return 0;
    }
    size_t written = 0;
    for (size_t i = 0; i < array->count && written < count; i++)
    {
        if (array->items[i]->type == OPENK4A_JSON_NUMBER)
        {
            out[written++] = (float)array->items[i]->number;
        }
    }
    return written;
}
