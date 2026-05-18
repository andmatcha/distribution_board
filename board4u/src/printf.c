#include "main.h"
#include <errno.h>
#include <unistd.h>

int _write(int file, char *ptr, int len)
{
    if ((file != STDOUT_FILENO) && (file != STDERR_FILENO)) {
        errno = EBADF;
        return -1;
    }

#if defined(DEBUG_LOG_ENABLED) && (DEBUG_LOG_ENABLED)
    for (int index = 0; index < len; index++) {
        ITM_SendChar((uint32_t)ptr[index]);
    }
#else
    (void)ptr;
#endif

    return len;
}
