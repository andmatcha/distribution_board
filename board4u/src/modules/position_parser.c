#include "modules/position_parser.h"

#include <stddef.h>
#include <stdlib.h>
#include <string.h>

#define POSITION_PARSER_SCALE 10000000.0
#define LATITUDE_LIMIT_E7 900000000L
#define LONGITUDE_LIMIT_E7 1800000000L

typedef enum {
    COORDINATE_TYPE_UNKNOWN = 0,
    COORDINATE_TYPE_LATITUDE,
    COORDINATE_TYPE_LONGITUDE
} coordinate_type_t;

typedef struct {
    const char *start;
    const char *end;
    coordinate_type_t type;
    int32_t value_e7;
} parsed_coordinate_t;

static char to_lower_ascii(char c)
{
    if ((c >= 'A') && (c <= 'Z')) {
        return (char)(c - 'A' + 'a');
    }

    return c;
}

static uint8_t is_ascii_letter(char c)
{
    return (uint8_t)(((c >= 'A') && (c <= 'Z')) || ((c >= 'a') && (c <= 'z')));
}

static uint8_t is_digit(char c)
{
    return (uint8_t)((c >= '0') && (c <= '9'));
}

static uint8_t is_space_ascii(char c)
{
    return (uint8_t)((c == ' ') || (c == '\t') || (c == '\r') || (c == '\n'));
}

static const char *skip_spaces(const char *text)
{
    while (is_space_ascii(*text) != 0U) {
        text++;
    }

    return text;
}

static uint8_t is_number_start(const char *text)
{
    if (is_digit(text[0]) != 0U) {
        return 1U;
    }

    if ((text[0] == '.') && (is_digit(text[1]) != 0U)) {
        return 1U;
    }

    if (((text[0] == '+') || (text[0] == '-')) &&
        ((is_digit(text[1]) != 0U) ||
         ((text[1] == '.') && (is_digit(text[2]) != 0U)))) {
        return 1U;
    }

    return 0U;
}

static uint8_t is_hemisphere(char c)
{
    c = to_lower_ascii(c);
    return (uint8_t)((c == 'n') || (c == 's') || (c == 'e') || (c == 'w'));
}

static coordinate_type_t coordinate_type_from_hemisphere(char hemisphere)
{
    hemisphere = to_lower_ascii(hemisphere);

    if ((hemisphere == 'n') || (hemisphere == 's')) {
        return COORDINATE_TYPE_LATITUDE;
    }

    if ((hemisphere == 'e') || (hemisphere == 'w')) {
        return COORDINATE_TYPE_LONGITUDE;
    }

    return COORDINATE_TYPE_UNKNOWN;
}

static const char *skip_component_gap(const char *text)
{
    while (*text != '\0') {
        if ((*text == ',') || (*text == ';') || (*text == '\r') || (*text == '\n')) {
            break;
        }

        if ((is_number_start(text) != 0U) || (is_hemisphere(*text) != 0U) ||
            (is_ascii_letter(*text) != 0U)) {
            break;
        }

        text++;
    }

    return text;
}

static const char *skip_to_coordinate_start(const char *text)
{
    while (*text != '\0') {
        if (is_number_start(text) != 0U) {
            break;
        }

        if (is_hemisphere(*text) != 0U) {
            const char *after_hemisphere = skip_spaces(text + 1);

            if (is_number_start(after_hemisphere) != 0U) {
                break;
            }
        }

        text++;
    }

    return text;
}

static double abs_double(double value)
{
    return (value < 0.0) ? -value : value;
}

static uint8_t type_matches_expected(coordinate_type_t type, coordinate_type_t expected)
{
    return (uint8_t)((expected == COORDINATE_TYPE_UNKNOWN) ||
                     (type == COORDINATE_TYPE_UNKNOWN) ||
                     (type == expected));
}

static uint8_t value_within_type_limit(double value, coordinate_type_t type)
{
    double limit = 180.0;

    if (type == COORDINATE_TYPE_LATITUDE) {
        limit = 90.0;
    }

    return (uint8_t)(value <= limit);
}

static uint8_t build_coordinate(const double *components,
                                unsigned int component_count,
                                char hemisphere,
                                coordinate_type_t expected,
                                coordinate_type_t detected_type,
                                int32_t *value_e7,
                                coordinate_type_t *resolved_type)
{
    double value;
    double scaled;
    int negative;
    coordinate_type_t type = detected_type;

    if (component_count == 0U) {
        return 0U;
    }

    if (type == COORDINATE_TYPE_UNKNOWN) {
        type = expected;
    }

    if (type_matches_expected(detected_type, expected) == 0U) {
        return 0U;
    }

    if ((component_count > 1U) &&
        (detected_type == COORDINATE_TYPE_UNKNOWN) &&
        (expected == COORDINATE_TYPE_UNKNOWN) &&
        (hemisphere == '\0')) {
        return 0U;
    }

    if ((component_count > 1U) && ((components[1] < 0.0) || (components[1] >= 60.0))) {
        return 0U;
    }

    if ((component_count > 2U) && ((components[2] < 0.0) || (components[2] >= 60.0))) {
        return 0U;
    }

    negative = (components[0] < 0.0) ? 1 : 0;
    value = abs_double(components[0]);

    if (component_count > 1U) {
        value += components[1] / 60.0;
    }

    if (component_count > 2U) {
        value += components[2] / 3600.0;
    }

    if (hemisphere != '\0') {
        const char lower_hemisphere = to_lower_ascii(hemisphere);

        if ((lower_hemisphere == 's') || (lower_hemisphere == 'w')) {
            negative = 1;
        } else {
            negative = 0;
        }
    }

    if (value_within_type_limit(value, type) == 0U) {
        return 0U;
    }

    if (negative != 0) {
        value = -value;
    }

    scaled = value * POSITION_PARSER_SCALE;
    if ((scaled > 2147483647.0) || (scaled < -2147483648.0)) {
        return 0U;
    }

    *value_e7 = (int32_t)((scaled >= 0.0) ? (scaled + 0.5) : (scaled - 0.5));
    *resolved_type = type;
    return 1U;
}

static uint8_t parse_coordinate_at(const char *start,
                                   coordinate_type_t expected,
                                   parsed_coordinate_t *coordinate)
{
    const char *text = skip_spaces(start);
    const char *component_start;
    char *component_end;
    coordinate_type_t detected_type = COORDINATE_TYPE_UNKNOWN;
    double components[3];
    unsigned int component_count = 0U;
    char hemisphere = '\0';
    uint8_t has_prefix_hemisphere = 0U;

    if (is_hemisphere(*text) != 0U) {
        hemisphere = *text;
        detected_type = coordinate_type_from_hemisphere(hemisphere);
        has_prefix_hemisphere = 1U;
        text = skip_spaces(text + 1);
    }

    if (is_number_start(text) == 0U) {
        return 0U;
    }

    component_start = text;

    for (;;) {
        components[component_count] = strtod(text, &component_end);
        if (component_end == text) {
            return 0U;
        }

        component_count++;
        text = skip_component_gap(component_end);

        if (is_hemisphere(*text) != 0U) {
            const coordinate_type_t suffix_type = coordinate_type_from_hemisphere(*text);
            const char *after_suffix = skip_spaces(text + 1);

            if ((has_prefix_hemisphere != 0U) &&
                (suffix_type != detected_type) &&
                (is_number_start(after_suffix) != 0U)) {
                break;
            }

            hemisphere = *text;
            detected_type = suffix_type;
            text++;
            break;
        }

        if ((component_count >= 3U) ||
            (is_number_start(text) == 0U) ||
            (text[0] == '+') ||
            (text[0] == '-')) {
            break;
        }
    }

    if (build_coordinate(components,
                         component_count,
                         hemisphere,
                         expected,
                         detected_type,
                         &coordinate->value_e7,
                         &coordinate->type) == 0U) {
        return 0U;
    }

    coordinate->start = component_start;
    coordinate->end = text;
    return 1U;
}

static uint8_t parse_coordinate_from(const char *text,
                                     coordinate_type_t expected,
                                     parsed_coordinate_t *coordinate)
{
    const char *candidate = skip_to_coordinate_start(text);

    while (*candidate != '\0') {
        if (parse_coordinate_at(candidate, expected, coordinate) != 0U) {
            return 1U;
        }

        candidate = skip_to_coordinate_start(candidate + 1);
    }

    return 0U;
}

static uint8_t keyword_matches_at(const char *text_start, const char *text, const char *keyword)
{
    const char *cursor = text;

    if ((text > text_start) && (is_ascii_letter(text[-1]) != 0U)) {
        return 0U;
    }

    while (*keyword != '\0') {
        if (to_lower_ascii(*cursor) != to_lower_ascii(*keyword)) {
            return 0U;
        }

        cursor++;
        keyword++;
    }

    if (is_ascii_letter(*cursor) != 0U) {
        return 0U;
    }

    return 1U;
}

static const char *find_keyword(const char *text_start, const char *search_start, const char *keyword)
{
    while (*search_start != '\0') {
        if (keyword_matches_at(text_start, search_start, keyword) != 0U) {
            return search_start;
        }

        search_start++;
    }

    return NULL;
}

static uint8_t parse_labeled_coordinate(const char *text,
                                        const char *const *keywords,
                                        size_t keyword_count,
                                        coordinate_type_t expected,
                                        int32_t *value_e7)
{
    size_t keyword_index;

    for (keyword_index = 0U; keyword_index < keyword_count; keyword_index++) {
        const char *match = text;
        const size_t keyword_length = strlen(keywords[keyword_index]);

        while ((match = find_keyword(text, match, keywords[keyword_index])) != NULL) {
            parsed_coordinate_t coordinate;

            if (parse_coordinate_from(match + keyword_length, expected, &coordinate) != 0U) {
                *value_e7 = coordinate.value_e7;
                return 1U;
            }

            match++;
        }
    }

    return 0U;
}

static int32_t abs_i32(int32_t value)
{
    return (value < 0) ? -value : value;
}

static uint8_t coordinate_can_be_latitude(const parsed_coordinate_t *coordinate)
{
    return (uint8_t)(((coordinate->type == COORDINATE_TYPE_LATITUDE) ||
                      (coordinate->type == COORDINATE_TYPE_UNKNOWN)) &&
                     (abs_i32(coordinate->value_e7) <= LATITUDE_LIMIT_E7));
}

static uint8_t coordinate_can_be_longitude(const parsed_coordinate_t *coordinate)
{
    return (uint8_t)(((coordinate->type == COORDINATE_TYPE_LONGITUDE) ||
                      (coordinate->type == COORDINATE_TYPE_UNKNOWN)) &&
                     (abs_i32(coordinate->value_e7) <= LONGITUDE_LIMIT_E7));
}

static uint8_t resolve_coordinate_pair(const parsed_coordinate_t *first,
                                       const parsed_coordinate_t *second,
                                       int32_t *latitude_e7,
                                       int32_t *longitude_e7)
{
    if ((first->type == COORDINATE_TYPE_LATITUDE) &&
        (coordinate_can_be_longitude(second) != 0U)) {
        *latitude_e7 = first->value_e7;
        *longitude_e7 = second->value_e7;
        return 1U;
    }

    if ((first->type == COORDINATE_TYPE_LONGITUDE) &&
        (coordinate_can_be_latitude(second) != 0U)) {
        *latitude_e7 = second->value_e7;
        *longitude_e7 = first->value_e7;
        return 1U;
    }

    if ((second->type == COORDINATE_TYPE_LATITUDE) &&
        (coordinate_can_be_longitude(first) != 0U)) {
        *latitude_e7 = second->value_e7;
        *longitude_e7 = first->value_e7;
        return 1U;
    }

    if ((second->type == COORDINATE_TYPE_LONGITUDE) &&
        (coordinate_can_be_latitude(first) != 0U)) {
        *latitude_e7 = first->value_e7;
        *longitude_e7 = second->value_e7;
        return 1U;
    }

    if ((coordinate_can_be_latitude(first) != 0U) &&
        (coordinate_can_be_longitude(second) != 0U)) {
        *latitude_e7 = first->value_e7;
        *longitude_e7 = second->value_e7;
        return 1U;
    }

    return 0U;
}

static uint8_t parse_coordinate_pair(const char *text, int32_t *latitude_e7, int32_t *longitude_e7)
{
    const char *cursor = text;

    while (*cursor != '\0') {
        parsed_coordinate_t first;
        parsed_coordinate_t second;

        if (parse_coordinate_from(cursor, COORDINATE_TYPE_UNKNOWN, &first) == 0U) {
            return 0U;
        }

        if (parse_coordinate_from(first.end, COORDINATE_TYPE_UNKNOWN, &second) != 0U) {
            if (resolve_coordinate_pair(&first, &second, latitude_e7, longitude_e7) != 0U) {
                return 1U;
            }
        }

        cursor = first.start + 1;
    }

    return 0U;
}

uint8_t position_parser_parse(const char *text, int32_t *latitude_e7, int32_t *longitude_e7)
{
    static const char *const latitude_keywords[] = {
        "latitude",
        "lat"
    };
    static const char *const longitude_keywords[] = {
        "longitude",
        "lon",
        "lng"
    };
    int32_t parsed_latitude = 0;
    int32_t parsed_longitude = 0;
    uint8_t has_latitude;
    uint8_t has_longitude;

    if ((text == NULL) || (latitude_e7 == NULL) || (longitude_e7 == NULL)) {
        return 0U;
    }

    has_latitude = parse_labeled_coordinate(text,
                                            latitude_keywords,
                                            sizeof(latitude_keywords) / sizeof(latitude_keywords[0]),
                                            COORDINATE_TYPE_LATITUDE,
                                            &parsed_latitude);
    has_longitude = parse_labeled_coordinate(text,
                                             longitude_keywords,
                                             sizeof(longitude_keywords) / sizeof(longitude_keywords[0]),
                                             COORDINATE_TYPE_LONGITUDE,
                                             &parsed_longitude);

    if ((has_latitude != 0U) && (has_longitude != 0U)) {
        *latitude_e7 = parsed_latitude;
        *longitude_e7 = parsed_longitude;
        return 1U;
    }

    return parse_coordinate_pair(text, latitude_e7, longitude_e7);
}
