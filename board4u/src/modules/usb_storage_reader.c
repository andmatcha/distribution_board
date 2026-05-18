#include "modules/usb_storage_reader.h"

#include <stddef.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>

#include "fatfs.h"
#include "usb_host.h"

#define USB_STORAGE_READER_PATH_BUFFER_SIZE 256U
#define USB_STORAGE_READER_MAX_SEARCH_DEPTH 4U

extern ApplicationTypeDef Appli_state;

static uint8_t storage_ready_reported;

static const char *fatfs_result_name(FRESULT result)
{
    switch (result) {
    case FR_OK:
        return "FR_OK";
    case FR_DISK_ERR:
        return "FR_DISK_ERR";
    case FR_INT_ERR:
        return "FR_INT_ERR";
    case FR_NOT_READY:
        return "FR_NOT_READY";
    case FR_NO_FILE:
        return "FR_NO_FILE";
    case FR_NO_PATH:
        return "FR_NO_PATH";
    case FR_INVALID_NAME:
        return "FR_INVALID_NAME";
    case FR_DENIED:
        return "FR_DENIED";
    case FR_EXIST:
        return "FR_EXIST";
    case FR_INVALID_OBJECT:
        return "FR_INVALID_OBJECT";
    case FR_WRITE_PROTECTED:
        return "FR_WRITE_PROTECTED";
    case FR_INVALID_DRIVE:
        return "FR_INVALID_DRIVE";
    case FR_NOT_ENABLED:
        return "FR_NOT_ENABLED";
    case FR_NO_FILESYSTEM:
        return "FR_NO_FILESYSTEM";
    case FR_MKFS_ABORTED:
        return "FR_MKFS_ABORTED";
    case FR_TIMEOUT:
        return "FR_TIMEOUT";
    case FR_LOCKED:
        return "FR_LOCKED";
    case FR_NOT_ENOUGH_CORE:
        return "FR_NOT_ENOUGH_CORE";
    case FR_TOO_MANY_OPEN_FILES:
        return "FR_TOO_MANY_OPEN_FILES";
    case FR_INVALID_PARAMETER:
        return "FR_INVALID_PARAMETER";
    default:
        return "FR_UNKNOWN";
    }
}

static const char *fatfs_filesystem_type_name(BYTE fs_type)
{
    switch (fs_type) {
    case FS_FAT12:
        return "FAT12";
    case FS_FAT16:
        return "FAT16";
    case FS_FAT32:
        return "FAT32";
#if _FS_EXFAT
    case FS_EXFAT:
        return "exFAT";
#endif
    default:
        return "unknown";
    }
}

static void print_fatfs_error(const char *operation, FRESULT result)
{
    printf("[usb-storage] %s failed: FRESULT=%d (%s)\r\n",
           operation,
           (int)result,
           fatfs_result_name(result));
}

static uint8_t is_current_or_parent_directory(const char *name)
{
    if ((name[0] == '.') && (name[1] == '\0')) {
        return 1U;
    }

    if ((name[0] == '.') && (name[1] == '.') && (name[2] == '\0')) {
        return 1U;
    }

    return 0U;
}

static char to_lower_ascii(char c)
{
    if ((c >= 'A') && (c <= 'Z')) {
        return (char)(c - 'A' + 'a');
    }

    return c;
}

static uint8_t strings_equal_ignore_ascii_case(const char *left, const char *right)
{
    while ((*left != '\0') && (*right != '\0')) {
        if (to_lower_ascii(*left) != to_lower_ascii(*right)) {
            return 0U;
        }

        left++;
        right++;
    }

    return (uint8_t)((*left == '\0') && (*right == '\0'));
}

static uint8_t has_text_file_extension(const char *name)
{
    const char *dot = NULL;

    while (*name != '\0') {
        if (*name == '.') {
            dot = name;
        }

        name++;
    }

    if (dot == NULL) {
        return 0U;
    }

    return strings_equal_ignore_ascii_case(dot, ".txt");
}

static FRESULT join_path(char *path, size_t path_size, const char *directory, const char *name)
{
    const size_t directory_length = strlen(directory);
    const char *name_suffix = name;
    const char *separator = "";
    int written;

    if (directory_length == 0U) {
        return FR_INVALID_NAME;
    }

    if ((directory[directory_length - 1U] == '/') && (name[0] == '/')) {
        name_suffix = name + 1;
    } else if ((directory[directory_length - 1U] != '/') && (name[0] != '/')) {
        separator = "/";
    }

    written = snprintf(path, path_size, "%s%s%s", directory, separator, name_suffix);
    if ((written < 0) || ((size_t)written >= path_size)) {
        return FR_INVALID_NAME;
    }

    return FR_OK;
}

static uint8_t file_matches_filter(const char *name, uint8_t accept_any_extension)
{
    if (accept_any_extension != 0U) {
        return 1U;
    }

    return has_text_file_extension(name);
}

static FRESULT read_regular_file(const char *file_path,
                                 uint8_t *buffer,
                                 size_t buffer_size,
                                 size_t *bytes_read)
{
    FIL file;
    FRESULT result;
    const size_t start_offset = *bytes_read;

    result = f_open(&file, file_path, FA_READ);
    if (result != FR_OK) {
        print_fatfs_error("open", result);
        return FR_OK;
    }

    while (*bytes_read < buffer_size) {
        UINT bytes_read_this_time = 0U;
        const size_t remaining = buffer_size - *bytes_read;

        result = f_read(&file,
                        &buffer[*bytes_read],
                        (UINT)remaining,
                        &bytes_read_this_time);
        if (result != FR_OK) {
            print_fatfs_error("read", result);
            (void)f_close(&file);
            return result;
        }

        if (bytes_read_this_time == 0U) {
            break;
        }

        *bytes_read += bytes_read_this_time;
    }

    (void)f_close(&file);

    printf("[usb-storage] loaded %lu bytes from %s\r\n",
           (unsigned long)(*bytes_read - start_offset),
           file_path);
    return FR_OK;
}

static FRESULT read_matching_files_recursive(const char *directory_path,
                                             unsigned int depth,
                                             uint8_t accept_any_extension,
                                             uint8_t *buffer,
                                             size_t buffer_size,
                                             size_t *bytes_read,
                                             uint32_t *matched_file_count)
{
    DIR directory;
    FILINFO file_info;
    FRESULT result;

    if ((depth > USB_STORAGE_READER_MAX_SEARCH_DEPTH) || (*bytes_read >= buffer_size)) {
        return FR_OK;
    }

    result = f_opendir(&directory, directory_path);
    if (result != FR_OK) {
        return result;
    }

    for (;;) {
        char entry_path[USB_STORAGE_READER_PATH_BUFFER_SIZE];

        result = f_readdir(&directory, &file_info);
        if (result != FR_OK) {
            break;
        }

        if (file_info.fname[0] == '\0') {
            break;
        }

        if (is_current_or_parent_directory(file_info.fname) != 0U) {
            continue;
        }

        result = join_path(entry_path, sizeof(entry_path), directory_path, file_info.fname);
        if (result != FR_OK) {
            break;
        }

        if ((file_info.fattrib & AM_DIR) != 0U) {
            result = read_matching_files_recursive(entry_path,
                                                   depth + 1U,
                                                   accept_any_extension,
                                                   buffer,
                                                   buffer_size,
                                                   bytes_read,
                                                   matched_file_count);
            if (result != FR_OK) {
                break;
            }

            if (*bytes_read >= buffer_size) {
                break;
            }

            continue;
        }

        if (file_matches_filter(file_info.fname, accept_any_extension) == 0U) {
            continue;
        }

        (*matched_file_count)++;
        result = read_regular_file(entry_path, buffer, buffer_size, bytes_read);
        if (result != FR_OK) {
            break;
        }

        if (*bytes_read >= buffer_size) {
            break;
        }
    }

    (void)f_closedir(&directory);
    return result;
}

void usb_storage_reader_init(void)
{
    storage_ready_reported = 0U;
    printf("[usb-storage] waiting for USB mass storage...\r\n");
}

void usb_storage_reader_poll(void)
{
    if (Appli_state == APPLICATION_READY) {
        if (storage_ready_reported == 0U) {
            storage_ready_reported = 1U;
            printf("[usb-storage] USB mass storage ready\r\n");
        }
        return;
    }

    if ((storage_ready_reported != 0U) && (Appli_state == APPLICATION_DISCONNECT)) {
        storage_ready_reported = 0U;
        printf("[usb-storage] USB mass storage disconnected\r\n");
    }
}

uint8_t usb_storage_reader_read_data(uint8_t *buffer, size_t buffer_size, size_t *bytes_read)
{
    FRESULT result;
    uint32_t matched_file_count = 0U;

    if ((buffer == NULL) || (bytes_read == NULL) || (buffer_size == 0U)) {
        return 0U;
    }

    *bytes_read = 0U;

    if (Appli_state != APPLICATION_READY) {
        printf("[usb-storage] read request ignored; USB mass storage is not ready\r\n");
        return 0U;
    }

    if (retUSBH != 0U) {
        printf("[usb-storage] FATFS USBH driver is not linked: retUSBH=%u\r\n",
               (unsigned int)retUSBH);
        return 0U;
    }

    result = f_mount(&USBHFatFS, USBHPath, 1U);
    if (result != FR_OK) {
        print_fatfs_error("mount", result);
        return 0U;
    }

    printf("[usb-storage] mounted %s filesystem at %s\r\n",
           fatfs_filesystem_type_name(USBHFatFS.fs_type),
           USBHPath);

    result = read_matching_files_recursive(USBHPath,
                                           0U,
                                           0U,
                                           buffer,
                                           buffer_size,
                                           bytes_read,
                                           &matched_file_count);
    if (result != FR_OK) {
        print_fatfs_error("scan", result);
        (void)f_mount(NULL, USBHPath, 0U);
        return 0U;
    }

    if (matched_file_count == 0U) {
        printf("[usb-storage] no .txt files found under %s; accepting all file extensions\r\n",
               USBHPath);
        result = read_matching_files_recursive(USBHPath,
                                               0U,
                                               1U,
                                               buffer,
                                               buffer_size,
                                               bytes_read,
                                               &matched_file_count);
        if (result != FR_OK) {
            print_fatfs_error("scan", result);
            (void)f_mount(NULL, USBHPath, 0U);
            return 0U;
        }
    }

    (void)f_mount(NULL, USBHPath, 0U);

    if (matched_file_count == 0U) {
        printf("[usb-storage] no files found under %s\r\n", USBHPath);
        return 0U;
    }

    if (*bytes_read == 0U) {
        printf("[usb-storage] no file data loaded from %lu matched file(s)\r\n",
               (unsigned long)matched_file_count);
        return 0U;
    }

    if (*bytes_read >= buffer_size) {
        printf("[usb-storage] loaded maximum %lu bytes from %lu matched file(s)\r\n",
               (unsigned long)*bytes_read,
               (unsigned long)matched_file_count);
    } else {
        printf("[usb-storage] loaded %lu bytes from %lu matched file(s)\r\n",
               (unsigned long)*bytes_read,
               (unsigned long)matched_file_count);
    }

    return 1U;
}
