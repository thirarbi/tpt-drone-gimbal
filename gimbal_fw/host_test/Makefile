CC = gcc
CFLAGS = -std=c11 -O2 -Wall -Wextra -Werror -Wdouble-promotion -Wconversion
MATH_DIR = ../components/gimbal_math

CFLAGS += -I$(MATH_DIR)

test_rot: test_rot.c $(MATH_DIR)/rot.c
		$(CC) $(CFLAGS) $^ -o $@ -lm

.PHONY: test
test: test_rot
		./test_rot