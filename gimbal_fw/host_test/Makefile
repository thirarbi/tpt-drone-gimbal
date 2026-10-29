CC = gcc
CFLAGS = -std=c11 -O2 -Wall -Wextra -Werror -Wdouble-promotion -Wconversion
MATH_DIR = ../components/gimbal_math
DXL_DIR = ../components/xl320
PROTO_DIR = ../components/proto
CFLAGS += -I$(MATH_DIR) -I$(DXL_DIR) -I$(PROTO_DIR)

TESTS = test_rot test_kinematics test_mahony test_dxl_packet test_stab test_proto

.PHONY: test clean
test: $(TESTS)
	@for t in $(TESTS); do ./$$t || exit 1; done

test_rot: test_rot.c $(MATH_DIR)/rot.c
	$(CC) $(CFLAGS) $^ -o $@ -lm

test_kinematics: test_kinematics.c $(MATH_DIR)/rot.c $(MATH_DIR)/kinematics.c
	$(CC) $(CFLAGS) $^ -o $@ -lm

test_mahony: test_mahony.c $(MATH_DIR)/rot.c $(MATH_DIR)/mahony.c
	$(CC) $(CFLAGS) $^ -o $@ -lm

test_dxl_packet: test_dxl_packet.c $(DXL_DIR)/dxl_packet.c
	$(CC) $(CFLAGS) $^ -o $@ -lm

test_stab: test_stab.c $(MATH_DIR)/rot.c $(MATH_DIR)/kinematics.c $(MATH_DIR)/stab.c
	$(CC) $(CFLAGS) $^ -o $@ -lm

test_proto: test_proto.c $(PROTO_DIR)/proto.c
	$(CC) $(CFLAGS) $^ -o $@

clean:
	rm -f $(TESTS)
