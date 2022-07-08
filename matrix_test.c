#define MATRIX_IMPLEMENTATION
#include "matrix.h"

int
main(int argc, char *argv[]) {

    matrix_t m = matrix_new(3, 2);

    matrix_set(m, 1., 2., 3., 4., 5., 6.);

    matrix_print(m);

    return 0;
}