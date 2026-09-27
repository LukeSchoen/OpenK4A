/*=============================================================================
  Logging.

  The SDK's k4a_set_debug_message_handler takes a callback and a level; this
  tree's own calls go to it when one is installed, and to stderr otherwise.
  Nothing is buffered and nothing is asynchronous: a message is a printf in
  the thread that caused it, which is what a program debugging a camera
  wants.
=============================================================================*/

#include "openk4a.h"

#include <stdarg.h>

static k4a_logging_message_cb_t *openk4a_log_handler;
static void *openk4a_log_context;
static openk4a_log_level_t openk4a_log_threshold = OPENK4A_LOG_INFO;

void openk4a_log_set_handler(k4a_logging_message_cb_t *handler, void *context)
{
    openk4a_log_handler = handler;
    openk4a_log_context = context;
}

void openk4a_log_set_level(openk4a_log_level_t level)
{
    openk4a_log_threshold = level;
}

static const char *log_level_name(openk4a_log_level_t level)
{
    switch (level)
    {
    case OPENK4A_LOG_CRITICAL:
        return "critical";
    case OPENK4A_LOG_WARNING:
        return "warning";
    case OPENK4A_LOG_INFO:
        return "info";
    case OPENK4A_LOG_TRACE:
        return "trace";
    default:
        return "error";
    }
}

void openk4a_log(openk4a_log_level_t level, const char *format, ...)
{
    /* A more severe level has the smaller number: the message is dropped when
     * it is quieter than the threshold and nobody is listening. */
    if (level > openk4a_log_threshold && openk4a_log_handler == NULL)
    {
        return;
    }

    char message[1024];
    va_list arguments;
    va_start(arguments, format);
    vsnprintf(message, sizeof(message), format, arguments);
    va_end(arguments);

    if (openk4a_log_handler != NULL)
    {
        openk4a_log_handler(openk4a_log_context, (k4a_log_level_t)level, "openk4a", __LINE__, message);
        return;
    }

    fprintf(stderr, "openk4a: %s: %s\n", log_level_name(level), message);
}

/*-----------------------------------------------------------------------------
  The public face.
---------------------------------------------------------------------------*/

k4a_result_t k4a_set_debug_message_handler(k4a_logging_message_cb_t *message_cb,
                                           void *message_cb_context,
                                           k4a_log_level_t min_level)
{
    if (min_level < K4A_LOG_LEVEL_CRITICAL || min_level > K4A_LOG_LEVEL_OFF)
    {
        return K4A_RESULT_FAILED;
    }
    if (message_cb == NULL)
    {
        openk4a_log_set_handler(NULL, NULL);
        openk4a_log_set_level((openk4a_log_level_t)min_level);
        return K4A_RESULT_SUCCEEDED;
    }
    openk4a_log_set_handler(message_cb, message_cb_context);
    openk4a_log_set_level((openk4a_log_level_t)min_level);
    return K4A_RESULT_SUCCEEDED;
}
