/*
 * matrix.h -- matrix utils.
 *
 * Copyright (c) zhoukk <izhoukk@gmail.com>
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in all
 * copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 */

#ifndef _MATRIX_H_
#define _MATRIX_H_

typedef struct {
    int rows;
    int cols;

    double **data;
} matrix_t;

matrix_t matrix_new(int rows, int cols);

void matrix_free(matrix_t m);

void matrix_set(matrix_t m, ...);

void matrix_identity(matrix_t m);

void matrix_copy(matrix_t s, matrix_t d);

void matrix_print(matrix_t m);

void matrix_add(matrix_t a, matrix_t b, matrix_t c);

void matrix_sub(matrix_t a, matrix_t b, matrix_t c);

void matrix_sub_identity(matrix_t m);

void matrix_mul(matrix_t a, matrix_t b, matrix_t c);

void matrix_mul_transpose(matrix_t a, matrix_t b, matrix_t c);

void matrix_transpose(matrix_t s, matrix_t d);

void matrix_scale(matrix_t m, double s);

void matrix_swap_rows(matrix_t m, int r1, int r2);

void matrix_scale_row(matrix_t m, int r, double s);

void matrix_shear_row(matrix_t m, int r1, int r2, double s);

int matrix_equal(matrix_t a, matrix_t b, double t);

int matrix_destructive_invert(matrix_t s, matrix_t d);

#endif // _MATRIX_H_

#ifdef MATRIX_IMPLEMENTATION

#include <assert.h>
#include <stdarg.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

matrix_t
matrix_new(int rows, int cols) {
    matrix_t m;

    m.rows = rows;
    m.cols = cols;
    m.data = (double **)malloc(sizeof(double *) * m.rows);
    assert(m.data);
    for (int i = 0; i < m.rows; i++) {
        m.data[i] = (double *)malloc(sizeof(double) * m.cols);
        assert(m.data[i]);
        for (int j = 0; j < m.cols; j++) {
            m.data[i][j] = 0.0;
        }
    }

    return m;
}

void
matrix_free(matrix_t m) {
    assert(m.data);
    for (int i = 0; i < m.rows; i++) {
        assert(m.data[i]);
        free(m.data[i]);
    }
    free(m.data);
}

void
matrix_set(matrix_t m, ...) {
    va_list ap;

    va_start(ap, m);
    for (int i = 0; i < m.rows; i++) {
        for (int j = 0; j < m.cols; j++) {
            m.data[i][j] = va_arg(ap, double);
        }
    }
    va_end(ap);
}

void
matrix_identity(matrix_t m) {
    assert(m.rows == m.cols);

    for (int i = 0; i < m.rows; i++) {
        for (int j = 0; j < m.cols; j++) {
            if (i == j) {
                m.data[i][j] = 1.0;
            } else {
                m.data[i][j] = 0.0;
            }
        }
    }
}

void
matrix_copy(matrix_t s, matrix_t d) {
    assert(s.rows == d.rows);
    assert(s.cols == d.cols);

    for (int i = 0; i < s.rows; i++) {
        for (int j = 0; j < s.cols; j++) {
            d.data[i][j] = s.data[i][j];
        }
    }
}

void
matrix_print(matrix_t m) {
    for (int i = 0; i < m.rows; i++) {
        for (int j = 0; j < m.cols; j++) {
            if (j > 0) {
                printf(" ");
            }
            printf("%6.2f", m.data[i][j]);
        }
        printf("\n");
    }
}

void
matrix_add(matrix_t a, matrix_t b, matrix_t c) {
    assert(a.rows == b.rows);
    assert(a.rows == c.rows);
    assert(a.cols == b.cols);
    assert(a.cols == c.cols);

    for (int i = 0; i < a.rows; i++) {
        for (int j = 0; j < a.cols; j++) {
            c.data[i][j] = a.data[i][j] + b.data[i][j];
        }
    }
}

void
matrix_sub(matrix_t a, matrix_t b, matrix_t c) {
    assert(a.rows == b.rows);
    assert(a.rows == c.rows);
    assert(a.cols == b.cols);
    assert(a.cols == c.cols);

    for (int i = 0; i < a.rows; i++) {
        for (int j = 0; j < a.cols; j++) {
            c.data[i][j] = a.data[i][j] - b.data[i][j];
        }
    }
}

void
matrix_sub_identity(matrix_t m) {
    assert(m.cols == m.cols);

    for (int i = 0; i < m.rows; i++) {
        for (int j = 0; j < m.cols; j++) {
            if (i == j) {
                m.data[i][j] = 1.0 - m.data[i][j];
            } else {
                m.data[i][j] = 0.0 - m.data[i][j];
            }
        }
    }
}

void
matrix_mul(matrix_t a, matrix_t b, matrix_t c) {
    assert(a.cols == b.rows);
    assert(a.rows == c.rows);
    assert(b.cols == c.cols);

    for (int i = 0; i < c.rows; i++) {
        for (int j = 0; j < c.cols; j++) {
            c.data[i][j] = 0.0;
            for (int k = 0; k < a.cols; k++) {
                c.data[i][j] += a.data[i][k] * b.data[k][j];
            }
        }
    }
}

void
matrix_mul_transpose(matrix_t a, matrix_t b, matrix_t c) {
    assert(a.cols == b.cols);
    assert(a.rows == c.rows);
    assert(b.rows == c.cols);

    for (int i = 0; i < c.rows; i++) {
        for (int j = 0; j < c.cols; j++) {
            c.data[i][j] = 0.0;
            for (int k = 0; k < a.cols; k++) {
                c.data[i][j] += a.data[i][k] * b.data[j][k];
            }
        }
    }
}

void
matrix_transpose(matrix_t s, matrix_t d) {
    assert(s.rows == d.cols);
    assert(s.cols == d.rows);

    for (int i = 0; i < s.rows; i++) {
        for (int j = 0; j < s.cols; j++) {
            d.data[j][i] = s.data[i][j];
        }
    }
}

void
matrix_scale(matrix_t m, double s) {
    assert(s != 0.0);

    for (int i = 0; i < m.rows; i++) {
        for (int j = 0; j < m.cols; j++) {
            m.data[i][j] *= s;
        }
    }
}

void
matrix_swap_rows(matrix_t m, int r1, int r2) {
    assert(r1 != r2);

    double *tmp = m.data[r1];
    m.data[r1] = m.data[r2];
    m.data[r2] = tmp;
}

void
matrix_scale_row(matrix_t m, int r, double s) {
    assert(s != 0.0);

    for (int i = 0; i < m.cols; i++) {
        m.data[r][i] *= s;
    }
}

void
matrix_shear_row(matrix_t m, int r1, int r2, double s) {
    assert(r1 != r2);
    for (int i = 0; i < m.cols; i++) {
        m.data[r1][i] += s * m.data[r2][i];
    }
}

int
matrix_equal(matrix_t a, matrix_t b, double e) {
    assert(a.rows == b.rows);
    assert(a.cols == b.cols);

    for (int i = 0; i < a.rows; i++) {
        for (int j = 0; j < a.cols; j++) {
            if (fabs(a.data[i][j] - b.data[i][j]) > e) {
                return 0;
            }
        }
    }

    return 1;
}

int
matrix_destructive_invert(matrix_t s, matrix_t d) {
    assert(s.rows == s.cols);
    assert(s.rows == d.rows);
    assert(s.rows == d.cols);

    matrix_identity(d);

    for (int i = 0; i < s.rows; i++) {
        if (s.data[i][i] == 0.0) {
            int r;
            for (r = i + 1; r < s.rows; r++) {
                if (s.data[r][i] != 0.0) {
                    break;
                }
            }
            if (r == s.rows) {
                return 0;
            }
            matrix_swap_rows(s, i, r);
            matrix_swap_rows(d, i, r);
        }

        const double scalar = 1.0 / s.data[i][i];
        matrix_scale_row(s, i, scalar);
        matrix_scale_row(d, i, scalar);

        for (int j = 0; j < s.rows; j++) {
            if (i == j) {
                continue;
            }
            double shear_needed = -s.data[j][i];
            matrix_shear_row(s, j, i, shear_needed);
            matrix_shear_row(d, j, i, shear_needed);
        }
    }

    return 1;
}

#endif // MATRIX_IMPLEMENTATION