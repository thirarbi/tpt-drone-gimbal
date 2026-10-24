// Test to prove the file compiles without actual math yet!
#include <stdio.h>
#include <assert.h>
#include "rot.h"

int main(void) {
    mat3_t m = {0};
    assert(m.m[0][0] == 0.0f);

    printf("test_rot: OK\n");
    return 0;
}