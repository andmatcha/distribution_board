#ifndef MODULES_USB_STORAGE_READER_H
#define MODULES_USB_STORAGE_READER_H

#include <stddef.h>
#include <stdint.h>

void usb_storage_reader_init(void);
void usb_storage_reader_poll(void);
uint8_t usb_storage_reader_read_data(uint8_t *buffer, size_t buffer_size, size_t *bytes_read);

#endif /* MODULES_USB_STORAGE_READER_H */
