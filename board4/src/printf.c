#include "debug_log.h"

#if DEBUG_LOG_ENABLED

#include "stm32f1xx.h"

#include <sys/errno.h>
#include <unistd.h>

#define ITM_WRITE_SPIN_LIMIT 10000U

static inline int itm_write_with_timeout(char ch)
{
    if ((ITM->TCR & ITM_TCR_ITMENA_Msk) == 0U) {
        return 1;
    }

    if ((ITM->TER & 1U) == 0U) {
        return 1;
    }

    for (uint32_t spin_count = 0U; ITM->PORT[0].u32 == 0U; spin_count++) {
        if (spin_count >= ITM_WRITE_SPIN_LIMIT) {
            return 0;
        }
        __NOP();
    }

    ITM->PORT[0].u8 = (uint8_t)ch;
    return 1;
}

int _write(int file, char *ptr, int len)
{
    if (file == STDOUT_FILENO || file == STDERR_FILENO) {
        for (int i = 0; i < len; i++) {
            if (!itm_write_with_timeout(ptr[i])) {
                break;
            }
        }

        return len;
    }

    errno = EBADF;
    return -1;
}

#endif
