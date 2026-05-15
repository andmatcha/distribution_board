#include "modules/usb_storage_reader.h"

#include <stddef.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>

#include "fatfs.h"
#include "modules/position_parser.h"
#include "usb_host.h"

#define USB_STORAGE_READER_BUFFER_SIZE 1024U
#define USB_STORAGE_READER_PATH_BUFFER_SIZE 256U
#define USB_STORAGE_READER_MAX_SEARCH_DEPTH 4U

extern ApplicationTypeDef Appli_state;

static uint8_t storage_ready_reported;
static uint32_t next_text_file_index;
static char read_buffer[USB_STORAGE_READER_BUFFER_SIZE];

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

static FRESULT copy_path(char *destination, size_t destination_size, const char *source)
{
    int written = snprintf(destination, destination_size, "%s", source);

    if ((written < 0) || ((size_t)written >= destination_size)) {
        return FR_INVALID_NAME;
    }

    return FR_OK;
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

static FRESULT count_text_files_recursive(const char *directory_path,
                                          unsigned int depth,
                                          uint32_t *file_count)
{
    DIR directory;
    FILINFO file_info;
    FRESULT result;

    if (depth > USB_STORAGE_READER_MAX_SEARCH_DEPTH) {
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
            result = count_text_files_recursive(entry_path, depth + 1U, file_count);
            if (result != FR_OK) {
                break;
            }
            continue;
        }

        if (has_text_file_extension(file_info.fname) != 0U) {
            (*file_count)++;
        }
    }

    (void)f_closedir(&directory);
    return result;
}

static FRESULT find_text_file_by_index_recursive(const char *directory_path,
                                                 unsigned int depth,
                                                 uint32_t target_index,
                                                 uint32_t *seen_count,
                                                 char *found_path,
                                                 size_t found_path_size)
{
    DIR directory;
    FILINFO file_info;
    FRESULT result;

    if (depth > USB_STORAGE_READER_MAX_SEARCH_DEPTH) {
        return FR_NO_FILE;
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
            result = FR_NO_FILE;
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
            result = find_text_file_by_index_recursive(entry_path,
                                                       depth + 1U,
                                                       target_index,
                                                       seen_count,
                                                       found_path,
                                                       found_path_size);
            if (result == FR_OK) {
                break;
            }

            if (result != FR_NO_FILE) {
                break;
            }

            continue;
        }

        if (has_text_file_extension(file_info.fname) == 0U) {
            continue;
        }

        if (*seen_count == target_index) {
            result = copy_path(found_path, found_path_size, entry_path);
            break;
        }

        (*seen_count)++;
    }

    (void)f_closedir(&directory);
    return result;
}

static uint8_t parse_position_file(const char *file_path, int32_t *latitude_e7, int32_t *longitude_e7)
{
    FIL file;
    UINT bytes_read = 0U;
    FRESULT result;

    result = f_open(&file, file_path, FA_READ);
    if (result != FR_OK) {
        print_fatfs_error("open", result);
        return 0U;
    }

    result = f_read(&file, read_buffer, sizeof(read_buffer) - 1U, &bytes_read);
    (void)f_close(&file);

    if (result != FR_OK) {
        print_fatfs_error("read", result);
        return 0U;
    }

    read_buffer[bytes_read] = '\0';

    if (position_parser_parse(read_buffer, latitude_e7, longitude_e7) == 0U) {
        printf("[usb-storage] no supported coordinate format in %s\r\n", file_path);
        return 0U;
    }

    printf("[usb-storage] parsed %s: lat=%ld lon=%ld\r\n",
           file_path,
           (long)*latitude_e7,
           (long)*longitude_e7);
    return 1U;
}

void usb_storage_reader_init(void)
{
    storage_ready_reported = 0U;
    next_text_file_index = 0U;
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
        next_text_file_index = 0U;
        printf("[usb-storage] USB mass storage disconnected\r\n");
    }
}

uint8_t usb_storage_reader_read_next_position(int32_t *latitude_e7, int32_t *longitude_e7)
{
    FRESULT result;
    uint32_t file_count = 0U;
    uint32_t offset;
    uint32_t start_index;

    if ((latitude_e7 == NULL) || (longitude_e7 == NULL)) {
        return 0U;
    }

    if (Appli_state != APPLICATION_READY) {
        printf("[usb-storage] coordinate request ignored; USB mass storage is not ready\r\n");
        return 0U;
    }

    result = f_mount(&USBHFatFS, USBHPath, 1U);
    if (result != FR_OK) {
        print_fatfs_error("mount", result);
        return 0U;
    }

    result = count_text_files_recursive(USBHPath, 0U, &file_count);
    if (result != FR_OK) {
        print_fatfs_error("scan", result);
        (void)f_mount(NULL, USBHPath, 0U);
        return 0U;
    }

    if (file_count == 0U) {
        printf("[usb-storage] no .txt files found under %s\r\n", USBHPath);
        (void)f_mount(NULL, USBHPath, 0U);
        return 0U;
    }

    start_index = next_text_file_index % file_count;

    for (offset = 0U; offset < file_count; offset++) {
        char file_path[USB_STORAGE_READER_PATH_BUFFER_SIZE];
        uint32_t seen_count = 0U;
        uint32_t target_index = start_index + offset;

        if (target_index >= file_count) {
            target_index -= file_count;
        }

        result = find_text_file_by_index_recursive(USBHPath,
                                                   0U,
                                                   target_index,
                                                   &seen_count,
                                                   file_path,
                                                   sizeof(file_path));
        if (result != FR_OK) {
            if (result != FR_NO_FILE) {
                print_fatfs_error("find text file", result);
                break;
            }

            continue;
        }

        if (parse_position_file(file_path, latitude_e7, longitude_e7) != 0U) {
            next_text_file_index = target_index + 1U;
            if (next_text_file_index >= file_count) {
                next_text_file_index = 0U;
            }

            (void)f_mount(NULL, USBHPath, 0U);
            return 1U;
        }
    }

    next_text_file_index = start_index + 1U;
    if (next_text_file_index >= file_count) {
        next_text_file_index = 0U;
    }

    (void)f_mount(NULL, USBHPath, 0U);
    return 0U;
}
