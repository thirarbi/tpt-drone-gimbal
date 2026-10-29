CC = gcc
CFLAGS = -std=c11 -O2 -Wall -Wextra -Werror -Wdouble-promotion -Wconversion
MATH_DIR = ../components/gimbal_math
CFLAGS += -I$(MATH_DIR)

TESTS = test_rot test_kinematics test_mahony

.PHONY: test clean
test: $(TESTS)
	@for t in $(TESTS); do ./$$t || exit 1; done

test_rot: test_rot.c $(MATH_DIR)/rot.c
	$(CC) $(CFLAGS) $^ -o $@ -lm

test_kinematics: test_kinematics.c $(MATH_DIR)/rot.c $(MATH_DIR)/kinematics.c
	$(CC) $(CFLAGS) $^ -o $@ -lm

test_mahony: test_mahony.c $(MATH_DIR)/rot.c $(MATH_DIR)/mahony.c
	$(CC) $(CFLAGS) $^ -o $@ -lm

clean:
	rm -f $(TESTS)
