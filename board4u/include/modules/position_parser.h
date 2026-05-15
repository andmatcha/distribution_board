#ifndef MODULES_POSITION_PARSER_H
#define MODULES_POSITION_PARSER_H

#include <stdint.h>

uint8_t position_parser_parse(const char *text, int32_t *latitude_e7, int32_t *longitude_e7);

#endif /* MODULES_POSITION_PARSER_H */
