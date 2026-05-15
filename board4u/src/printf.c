#include "main.h"
#include <errno.h>
#include <stdint.h>
#include <unistd.h>

extern UART_HandleTypeDef huart2;

int _write(int file, char *ptr, int len)
{
    if (file == STDOUT_FILENO || file == STDERR_FILENO) {
        const uint8_t *data = (const uint8_t *)ptr;
        int remaining = len;

        while (remaining > 0) {
            uint16_t chunk = (remaining > UINT16_MAX) ? UINT16_MAX : (uint16_t)remaining;

            if (HAL_UART_Transmit(&huart2, (uint8_t *)data, chunk, HAL_MAX_DELAY) != HAL_OK) {
                errno = EIO;
                return -1;
            }

            data += chunk;
            remaining -= chunk;
        }

        return len;
    }

    errno = EBADF;
    return -1;
}
