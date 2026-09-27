/*=============================================================================
  Memory.

  k4a_set_allocator lets a program own every allocation the SDK makes, so
  every allocation this tree makes goes through these two functions - the
  images, the frames, the tables, the JSON arena, and the handles themselves.
  With no allocator installed they are malloc and free, which is what the
  tree uses when it is on its own.
=============================================================================*/

#include "openk4a.h"

static k4a_memory_allocate_cb_t *openk4a_allocate;
static k4a_memory_destroy_cb_t *openk4a_release;

void openk4a_allocator_set(k4a_memory_allocate_cb_t *allocate, k4a_memory_destroy_cb_t *free_fn)
{
    openk4a_allocate = allocate;
    openk4a_release = free_fn;
}

bool openk4a_allocator_installed(void)
{
    return openk4a_allocate != NULL;
}

/* A program's allocator hands back a context that its destructor wants again,
 * and openk4a_free only has the pointer, so the context is kept in a small header
 * the caller never sees. Sixteen bytes keeps the returned pointer aligned. */
#define OPENK4A_ALLOC_HEADER 16

void *openk4a_alloc(size_t size)
{
    if (openk4a_allocate != NULL)
    {
        void *context = NULL;
        uint8_t *base = openk4a_allocate((int)(size + OPENK4A_ALLOC_HEADER), &context);
        if (base == NULL)
        {
            return NULL;
        }
        memcpy(base, &context, sizeof(context));
        return base + OPENK4A_ALLOC_HEADER;
    }
    return malloc(size);
}

void *openk4a_alloc_zero(size_t size)
{
    void *block = openk4a_alloc(size);
    if (block != NULL)
    {
        memset(block, 0, size);
    }
    return block;
}

void openk4a_free(void *pointer)
{
    if (pointer == NULL)
    {
        return;
    }
    if (openk4a_release != NULL)
    {
        uint8_t *base = (uint8_t *)pointer - OPENK4A_ALLOC_HEADER;
        void *context = NULL;
        memcpy(&context, base, sizeof(context));
        openk4a_release(base, context);
        return;
    }
    free(pointer);
}

/*-----------------------------------------------------------------------------
  The public face.
---------------------------------------------------------------------------*/

k4a_result_t k4a_set_allocator(k4a_memory_allocate_cb_t allocate, k4a_memory_destroy_cb_t free_fn)
{
    /* The SDK's own rule: an allocator may only be installed before any other
     * call, and both halves must be given together. */
    if ((allocate == NULL) != (free_fn == NULL))
    {
        return K4A_RESULT_FAILED;
    }
    if (openk4a_allocator_installed())
    {
        openk4a_log(OPENK4A_LOG_ERROR, "an allocator is already installed");
        return K4A_RESULT_FAILED;
    }
    openk4a_allocator_set(allocate, free_fn);
    return K4A_RESULT_SUCCEEDED;
}
