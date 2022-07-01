#ifndef _READLINE_H_
#define _READLINE_H_

#include <stdio.h>

typedef struct {
    char *buffer;
    size_t len;
    size_t cursor;
} readline_t;

void readline_push(readline_t *rl, const char *buffer, size_t len);

char *readline_pop(readline_t *rl);

#endif // _READLINE_H_

#ifdef READLINE_IMPLEMENTATION

void
readline_push(readline_t *rl, const char *buffer, size_t len) {
    size_t new_len = rl->len - rl->cursor + len;
    char *new_buffer = (char *)malloc(new_len);
    if (rl->len - rl->cursor > 0) {
        memcpy(new_buffer, rl->buffer + rl->cursor, rl->len - rl->cursor);
    }
    memcpy(new_buffer + rl->len - rl->cursor, buffer, len);
    if (rl->buffer) {
        free(rl->buffer);
    }

    rl->buffer = new_buffer;
    rl->len = new_len;
    rl->cursor = 0;
}

char *
readline_pop(readline_t *rl) {
    char *line;
    size_t len;
    size_t cur = rl->cursor;

    while (cur < rl->len && rl->buffer[cur++] != '\n')
        ;

    len = cur - rl->cursor;
    if (len == 0) {
        return 0;
    }

    line = (char *)malloc(len);
    if (0 == line) {
        return 0;
    }

    memcpy(line, rl->buffer + rl->cursor, len);
    rl->cursor = cur;
    return line;
}

#endif // READLINE_IMPLEMENTATION