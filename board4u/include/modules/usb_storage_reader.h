#ifndef MODULES_USB_STORAGE_READER_H
#define MODULES_USB_STORAGE_READER_H

#include <stdint.h>

void usb_storage_reader_init(void);
void usb_storage_reader_poll(void);
uint8_t usb_storage_reader_read_next_position(int32_t *latitude_e7, int32_t *longitude_e7);

#endif /* MODULES_USB_STORAGE_READER_H */
