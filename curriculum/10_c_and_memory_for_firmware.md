# Module 10: C and Memory Management for `gimbal_fw`

A **topic map**, not a tutorial. It lists everything in C, memory and embedded
concurrency that `october_demo_plan.md` assumes you know. For each topic it says
**why this project needs it**, **where it shows up** (task and file), and gives
**self-check questions**. Answers are left out on purpose: if you can't answer a
question, that topic is where you start.

Assumes zero C. Skip anything you already know.

---

## How to use this

1. Go through the **self-check questions** for each topic. Mark each topic
   🟢 (can answer all of them), 🟡 (shaky) or 🔴 (no idea).
2. Look at the **task → topic matrix** (§E) to see which 🔴 topics block the next
   task on your list. Learn those first.
3. Learn by **compiling small experiments on the laptop** (§D.1). Run each one with
   `-fsanitize=address,undefined`. Most memory bugs that take an afternoon to find
   on the ESP32 show up instantly on the host this way.

Tiers:

- **Core**: you can't write any of the firmware without it.
- **Needed**: required by specific tasks (see the matrix).
- **Depth**: makes you faster and lets you defend design choices in front of the
  lecturer, but isn't required to get things working.

---

## Part A: The C language

### A.1 How a C program is built (Core)

**What:** preprocessor → compiler → assembler → linker. Translation units (one
`.c` = one unit). Header (`.h`) = declarations, source (`.c`) = definitions.
Include guards / `#pragma once`. Declaration vs definition. "Undefined reference"
(linker error) vs "implicit declaration" (compiler error).

**Why here:** the whole firmware is split into components (`gimbal_math`,
`mpu6050`, `xl320`, ...), and `gimbal_math` must compile **both** with ESP-IDF and
with plain `gcc` (T2). If you don't understand what a header is for, the component
boundaries look like magic.

**Where:** every file; `host_test/Makefile`; each component's `CMakeLists.txt`.

**Self-check:**
- What does `#include "rot.h"` literally do?
- Why put a function *prototype* in the `.h` and the *body* in the `.c`? What
  happens if you put the body in the `.h` and include it from two `.c` files?
- A compile error says *implicit declaration of function*; a link error says
  *undefined reference*. What's missing in each case?
- What does `static` mean in front of a **function** at file scope? Why do the
  plan's ISR and `ctrl_task` use it?
- What is `extern` for?

### A.2 Types, and conversions between them (Core)

**What:** `char/short/int/long` and why their sizes aren't fixed;
`<stdint.h>` (`uint8_t`, `int16_t`, `uint16_t`, `int64_t`); `size_t`; `bool`
(`<stdbool.h>`); signed vs unsigned; **integer promotion**; **usual arithmetic
conversions**; explicit casts; `float` vs `double`; literal suffixes (`1.0f`,
`1023u`).

**Why here:** most "silent" firmware bugs in this project are conversion bugs.
- The MPU6050 gives you two `uint8_t`s that must become one **signed** `int16_t`.
- `tick - zero_tick` with both `uint16_t` → what if `tick < zero_tick`? (Look at
  why §4.3 casts to `int` first.)
- `float` vs `double`: on the ESP32, `double` is emulated in software (§4.4).
  `0.5` is a `double` literal; `0.5f` is a `float`.
- The plan compiles host tests with `-Wconversion -Wdouble-promotion -Werror`, so
  every implicit conversion becomes a compile error you'll have to understand.

**Where:** `mpu6050.c` (T4), `xl320.c` tick ↔ rad (T8, T10), `rot.c`,
`mahony.c`, everything under `-Wconversion`.

**Self-check:**
- What is `(uint8_t)200 + (uint8_t)100`? What type is the result?
- `uint16_t a = 100, b = 200;` What is `a - b`? Now what is `(int)a - (int)b`?
- Why is `int16_t x = 0xFFFF;` problematic, while `uint16_t x = 0xFFFF;` is fine?
- What does `-Wdouble-promotion` warn about in `float y = sinf(x) * 0.5;`?
- Signed overflow vs unsigned overflow: one wraps, one is *undefined behaviour*.
  Which is which?

### A.3 Operators, especially bitwise (Core)

**What:** `& | ^ ~ << >>`, masks, testing / setting / clearing a bit, combining
two bytes into a 16-bit value, splitting a 16-bit value into two bytes. Shifts on
signed values. Operator precedence (`a & b == c` is a classic trap).

**Why here:** register configuration (MPU6050), both CRCs (bitwise loops), the
Dynamixel error byte (bit 7 = Alert), Present Load (bits 0–9 magnitude, bit 10
direction), LED bits, little-endian packet fields.

**Where:** `mpu6050.c`, `dxl_packet.c` (`crc16_dxl`), `proto.c` (`crc16_ccitt`),
`xl320.c`.

**Self-check:**
- Given `uint8_t hi, lo;` from a big-endian register pair, write the expression
  for the signed 16-bit value. Where does the cast go, and why does it matter?
- How do you test whether bit 7 of `err` is set? Clear bit 3? Extract bits 0–9?
- What is `1 << 31` when `int` is 32 bits? What about `1u << 31`?
- Why does `if (flags & MASK == MASK)` not do what it looks like?
- Trace a bitwise CRC by hand for one byte. What does "poly", "init" and
  "reflected" each change?

### A.4 Functions and control flow (Core)

**What:** `if/for/while/switch`, `break/continue`, function prototypes,
**pass-by-value** (C has nothing else), return values as error codes, `goto`
for cleanup (it is accepted style in C for exactly this).

**Why here:** C has no exceptions. Every driver call in the plan returns `0`,
a negative `XL320_ERR_*`, or `esp_err_t`. Byte-wise parsers (`dxl_parser_feed`,
`proto_parser_feed`) are `switch` statements on a state.

**Self-check:**
- If C only passes by value, how does `xl320_read_u16(id, addr, &v)` return a
  value through `v`?
- Why does the plan want `gimbal_ik` to return an `int` *and* write results into
  an array, instead of returning the array?
- When is `switch` fall-through a bug, and when is it intended?

### A.5 Arrays (Core)

**What:** fixed-size arrays, 2-D arrays (`float m[3][3]`), indexing, **array
decay** (an array becomes a pointer when passed to a function), why a function
can't know the length of an array it receives, `sizeof` on an array vs on a
pointer, out-of-bounds access.

**Why here:** `float q_rad[3]`, `mat3_t` (3×3), packet buffers `uint8_t out[]`,
`const uint8_t ids[]` + `int n` in `xl320_sync_write_goal`.

**Self-check:**
- In `void gimbal_fk(const float q_rad[3], mat3_t *R)`, what type does `q_rad`
  *actually* have inside the function? Does the `3` mean anything to the compiler?
- Why do almost all buffer APIs take a pointer **and** a length?
- `uint8_t buf[64]; f(buf);` Inside `f`, what is `sizeof(buf)`?
- What happens when you write `buf[64] = 0`? (Hint: nothing visible, *yet*.)
- Row-major: in `m[2][1]`, which index is the row?

### A.6 Pointers (Core, the most important topic)

**What:** address-of `&`, dereference `*`, pointer types, `NULL`, pointer
arithmetic, `const` with pointers (`const T *p` vs `T *const p`), output
parameters, `void *`, pointer to struct (`->`), pointer to pointer, **function
pointers** (callbacks).

**Why here:** everywhere.
- Output params: `mpu6050_read(mpu6050_sample_t *out)`.
- State objects: `mahony_update(mahony_t *f, ...)`, `dxl_parser_feed(dxl_parser_t *p, ...)`.
- `void *`: FreeRTOS task argument, `proto_encode(..., const void *payload, ...)`.
- Function pointers: task functions passed to `xTaskCreatePinnedToCore`, the ISR
  passed to `gpio_isr_handler_add`, the per-frame callback in `proto_parser_feed`.
- `const` correctness: `const float q_rad[3]` promises the function won't modify
  the input. The compiler holds you to it.

**Self-check:**
- Difference between `int *p; *p = 5;` and `int x; int *p = &x; *p = 5;`?
- `const uint8_t *p` vs `uint8_t *const p`: which one can you reassign, which can
  you write through?
- `p->field` is shorthand for what?
- If `p` is a `uint16_t *`, how many bytes does `p + 1` move forward?
- Write the type of "pointer to a function taking `void *` and returning `void`".
  Where in the plan does that exact type appear?
- Why must a `void *` be cast (or assigned to a typed pointer) before you can
  dereference it?

### A.7 Structs, enums, typedef (Core)

**What:** `struct` definition, `typedef`, member access, designated initialisers
(`{ .kp = 1.0f, .ki = 0.02f }`), nesting, passing structs by pointer, returning
small structs by value, `enum` for named states/modes.

**Why here:** the plan's API is *built* from structs: `mat3_t`, `mahony_t`,
`stab_in_t`, `stab_out_t`, `calib_t`, `dxl_parser_t`, `msg_set_target_t`.
Modes (`PARK/HOLD/NADIR`) and parser states are `enum`s.

**Self-check:**
- When do you pass a struct by value vs by pointer? What does each cost?
- What does `mat3_t R = {0};` guarantee about *all* fields?
- Why `typedef struct { ... } mahony_t;` rather than writing `struct mahony` everywhere?
- What is the value of the first enumerator if you don't assign one?

### A.8 Struct layout: padding, alignment, packing (Needed)

**What:** alignment requirements, padding bytes the compiler inserts,
`sizeof(struct)` ≠ sum of fields, `offsetof`, `__attribute__((packed))` and its
cost (unaligned access), `_Static_assert` to lock a layout.

**Why here:** §4.5's `msg_set_target_t` is packed so its bytes match Python's
`struct.Struct('<B3f')` exactly. `calib_t` is saved to NVS as a raw blob. The
`ctypes` sim (T7) must mirror C structs byte for byte. A layout mismatch is
**silent**: pitch and roll just swap (§T13).

**Where:** `proto.h`, `calib.h`, `stab.h` (for ctypes), `gimbal_proto.py`.

**Self-check:**
- `struct { uint8_t a; float b; }`: what is `sizeof` without `packed`? With?
- Why can reading `b` from a packed struct be slower or even fault on some CPUs?
- How would a `_Static_assert(sizeof(msg_set_target_t) == 13, "...")` protect you?
- If you add a field to `calib_t` and flash new firmware, what happens when
  `calib_load()` reads the *old* blob from NVS? How would you detect it?

### A.9 Serialisation: bytes ↔ values (Needed)

**What:** endianness (big vs little), the ESP32 being little-endian, building
multi-byte values with shifts vs `memcpy`, **strict aliasing** (why
`*(float *)&buf[4]` is undefined behaviour), `memcpy` as the safe type-pun.

**Why here:** three byte orders meet in this project:
MPU6050 registers are **big-endian**; Dynamixel and your Pi protocol are
**little-endian**; the ESP32 and x86 laptop are little-endian.

**Where:** `mpu6050.c` (T4), `dxl_packet.c` (T8), `proto.c` (T13).

**Self-check:**
- How is `0x1234` laid out in memory on a little-endian CPU?
- Why does decoding the MPU6050 with shifts work on any CPU, while
  `memcpy(&x, buf, 2)` gives the wrong answer here?
- Why is `float f = *(float *)(buf + 1);` dangerous, *two* ways (aliasing,
  alignment)? What do you write instead?

### A.10 Preprocessor (Core)

**What:** `#define` constants and macros, why macro arguments need parentheses,
`#if/#ifdef` for conditional compilation, `assert` (`<assert.h>`) and `NDEBUG`,
`_Static_assert`. `#define` vs `static const` vs `enum` for constants.

**Why here:** `XL320_RAD_PER_TICK`, `board.h` pin map, `assert(tick <= 1023)`
before `dxl_build` (§4.3), host vs target builds.

**Self-check:**
- Why is `#define SQ(x) x*x` wrong for `SQ(a+1)`? Fix it. What's still wrong with
  `SQ(i++)`?
- What does `assert` compile to when `NDEBUG` is defined? So should an
  `assert` ever be your *only* safety check on hardware?
- Why does `XL320_RAD_PER_TICK` in §4.3 end its literals with `f`?

### A.11 Floating point and `<math.h>` (Needed)

**What:** `sinf/cosf/atan2f/asinf/sqrtf/fminf/fmaxf/fabsf`, linking with `-lm`,
NaN and Inf (how they appear, how they spread), comparing floats with a
tolerance, single-precision limits (~7 significant digits), `M_PI` not being
standard C.

**Why here:** kinematics (T3b, done), Mahony (T5), `stab_step` (T7/T11). The plan
already warns about `asinf(1.0000001f) = NaN`.

**Self-check:**
- Name three operations that produce NaN. Why is `x == x` false for NaN?
- If one NaN enters the Mahony quaternion, what happens to every later output?
  Where would you check for it?
- Why compare angles with `wrap_pi(a - b)` and a tolerance, not `a == b`?
- What does `float` precision mean for integrating a 1 kHz gyro over 10 minutes?

### A.12 Program structure: "objects" in C (Needed)

**What:** state struct + functions that take a pointer to it (`mahony_init`,
`mahony_update`); pure functions (output depends only on input); `static` file
scope as "private"; opaque handles (`typedef struct foo *foo_handle_t`).

**Why here:** the plan's main design decision is *keep the control law a pure
function* (§3.1 `stab.h`) so it can be host-tested and called from Python.
ESP-IDF itself hands you opaque handles (`i2c_master_dev_handle_t`).

**Self-check:**
- What makes a function "pure"? Why can't a pure `stab_step` read a global?
- Why can you have two independent Mahony filters with this pattern but not with
  file-level globals?
- What is an opaque handle, and why can't you `sizeof` the struct behind it?

### A.13 Byte-wise state machines (Needed)

**What:** `enum` state + `switch` + index into a buffer, consuming one byte at a
time, resync after a bad byte or CRC, never blocking.

**Why here:** `dxl_parser_feed` (T8) and `proto_parser_feed` (T13). Byte-wise
parsing is what lets you host-test the parser with a recorded "echo + reply"
stream.

**Self-check:**
- Why parse byte by byte instead of "read a whole packet"? What does UART give
  you, in what chunks?
- The length field says 300 bytes but your buffer is 64. What must the parser do?
- After a CRC failure, where should the parser restart searching for the header?

---

## Part B: Memory

### B.1 Storage duration and lifetime (Core)

**What:** the three kinds of storage:
- **automatic** (locals, on the stack, gone when the function returns),
- **static** (globals, and `static` locals, live for the whole program),
- **allocated** (`malloc/free`, on the heap, live until you free them).

Scope (where a *name* is visible) vs lifetime (how long the *memory* exists).
Initialisation rules: statics are zero-initialised, automatic locals are **not**.

**Why here:** almost every embedded memory bug is a lifetime bug: returning a
pointer to a local, giving a task a pointer to a local of `app_main` (which
**returns**, see the cheat sheet), reading an uninitialised local.

**Self-check:**
- What is wrong with `float *make_q(void) { float q[3] = {0}; return q; }`?
- `app_main` creates a task and passes `&cfg` where `cfg` is a local of
  `app_main`. Then `app_main` returns. What happens?
- What is the value of an uninitialised local `int x;`? And of an uninitialised
  global?
- What does `static` mean on a **local variable**? (Different from A.1!)
- Why does §4.4 declare `static TaskHandle_t s_ctrl;` at file scope?

### B.2 The stack (Core)

**What:** call frames, what lives in them (locals, return addresses, saved
registers), stack growth, **stack overflow**, and how large arrays as locals eat
it. In FreeRTOS **every task has its own fixed-size stack**.

**Why here:** `xTaskCreatePinnedToCore(ctrl_task, "ctrl", 4096, ...)`: that
`4096` is the task's whole stack. In ESP-IDF it's in **bytes** (vanilla FreeRTOS
uses *words*; this difference shows up in tutorials). `printf` with floats alone
needs a lot of stack.

**Self-check:**
- A task with a 2048-byte stack declares `uint8_t buf[2048];` as a local. What
  happens?
- How can you measure how much stack a task actually used?
  (Look up `uxTaskGetStackHighWaterMark`.)
- What does ESP-IDF print when it detects a stack overflow, and which `menuconfig`
  option controls the detection?

### B.3 The heap, and why firmware avoids it at runtime (Needed)

**What:** `malloc/calloc/realloc/free`, leaks, double free, use-after-free,
fragmentation, checking for `NULL`. The embedded rule: **allocate at init, never
in the loop**.

**Why here:** your 1 kHz loop must have bounded timing (§2.4 budget ≤ 400 µs).
`malloc` takes a variable amount of time and can fail after hours of
fragmentation. ESP-IDF drivers allocate internally during `*_install`/`*_new_*`
calls, which is why those happen once at boot.

**Self-check:**
- List four distinct heap bugs. Which of them does AddressSanitizer catch on the
  laptop?
- Why is "it ran fine for 10 minutes" weak evidence that there's no leak?
- Which of the plan's structs could you just declare `static` instead of
  allocating them?
- What does `heap_caps_get_free_size()` tell you, and when would you log it?

### B.4 The ESP32 memory map (Depth → Needed for ISRs)

**What:** flash (code + `const` data, executed through a cache), IRAM (code that
must run when flash is unavailable), DRAM (data), RTC memory, PSRAM (WROVER only).
Where `const` arrays end up. Why `IRAM_ATTR` exists.

**Why here:** the plan's ISR is `IRAM_ATTR` (§4.4). During flash writes (NVS
saves!) the flash cache is disabled; an ISR in flash crashes then. WROVER's PSRAM
also takes GPIO 16/17 (§2.5).

**Self-check:**
- Why must an ISR (and everything it calls) be in IRAM?
- What happens if a non-IRAM ISR fires while `calib_save()` writes to NVS?
- Where does `static const uint16_t table[256]` go? Where does it go without `const`?

### B.5 Undefined behaviour (Core, so you recognise it)

**What:** the list that bites embedded C: out-of-bounds access, uninitialised
reads, signed overflow, shifting by ≥ type width, dereferencing `NULL` or freed
memory, strict-aliasing violations, unaligned access, modifying a variable twice
without a sequence point (`i = i++`).

**Why here:** UB often *works* on the laptop and fails on the ESP32 (or the
reverse), or works at `-O0` and breaks at `-O2` (§4.7 builds with
`OPTIMIZATION_PERF`). That's exactly the "C and Python disagree only on target"
scenario in T10.

**Self-check:**
- Why can the compiler legally delete an `if (x + 1 < x)` overflow check on a
  signed `int`?
- Why is "it works on my laptop" not evidence that a function has no UB?
- What do `-fsanitize=undefined` and `-fsanitize=address` each detect?

---

## Part C: Concurrency, interrupts, FreeRTOS

### C.1 Interrupts and ISR rules (Needed: T4, T9)

**What:** what an interrupt is, ISR context vs task context, keep ISRs tiny,
only `...FromISR` FreeRTOS calls in ISRs, `portYIELD_FROM_ISR`, no blocking, no
`printf`, no float on the ESP32 (§4.4), `volatile`.

**Why here:** the MPU6050 data-ready pin drives the whole 1 kHz loop through an
ISR that only notifies a task (§4.4).

**Self-check:**
- Why can't an ISR call `vTaskDelay` or `xQueueSend` (without `FromISR`)?
- What does `portYIELD_FROM_ISR(woken)` achieve? What happens without it?
- What does `volatile` promise, and what does it **not** promise (hint:
  atomicity, ordering across cores)?
- Why is the ISR's job only "notify", with the I²C read in the task?

### C.2 Tasks and the scheduler (Core for T9+)

**What:** tasks as independent `for(;;)` loops, priorities, preemption, blocking
vs busy-waiting, the idle task and the **task watchdog**, the tick (`CONFIG_FREERTOS_HZ`),
the ESP32's two cores and pinning (`xTaskCreatePinnedToCore`).

**Why here:** the architecture is four tasks on two cores (§2.1). "Busy-wait loop:
not allowed" (cheat sheet).

**Self-check:**
- What does it mean for a task to "block"? What runs while it's blocked?
- Why does a busy-wait loop reset the chip after a few seconds?
- `vTaskDelay(pdMS_TO_TICKS(1))` with a 100 Hz tick: how long does it really wait?
  Why does the plan set 1000 Hz?
- Why put `ctrl_task` on core 1 and the bus/comms tasks on core 0?

### C.3 Sharing data between tasks (Core for T9+)

**What:** race conditions, torn reads (a 3-float array half-updated), critical
sections, mutexes, **queues as copy-in/copy-out**, the 1-slot "mailbox"
(`xQueueOverwrite` + `xQueuePeek`), task notifications, **single ownership** of a
resource.

**Why here:** `q_cmd` flows ctrl → servo, `q_meas` flows servo → ctrl (§2.1).
§4.4 says `servo_task` *owns* UART2 alone, "so no mutex". Understanding why that
works is the point of this topic.

**Self-check:**
- Two tasks share a global `float q[3]`; one writes, one reads. Describe a torn
  read concretely.
- A FreeRTOS queue **copies** the item. Why does that remove the lifetime problem
  from B.1?
- What happens if you put a *pointer* to a local into a queue instead of the data?
- Why does "one task owns the bus" remove the need for a mutex?
- Mailbox vs normal queue: when do you want the *latest* value instead of *every*
  value?
- Task notification vs binary semaphore: why does the plan use the notification?

### C.4 Timing and measuring (Needed)

**What:** `esp_timer_get_time()` (µs, `int64_t`), computing deltas, min/avg/max
stats, why you don't log inside a hard loop (push to a ring buffer instead).

**Why here:** every "Done when" in T4/T8/T9 is a measured number.

**Self-check:**
- Why is the timestamp `int64_t` and not `uint32_t`? When would a 32-bit µs
  counter wrap?
- Why does `ESP_LOGI` inside `ctrl_task` ruin the very loop time you're measuring?
- What's a ring buffer, and what happens when the writer is faster than the reader?

---

## Part D: Tooling and the ESP-IDF way

### D.1 Host builds, warnings, sanitizers, debugger (Core, do this first)

**What:** compiling with `gcc` by hand, reading the flags in §4.7 (`-std=c11 -Wall
-Wextra -Werror -Wdouble-promotion -Wconversion`), a minimal `Makefile`,
`-g -O0` vs `-O2`, `-fsanitize=address,undefined`, `gdb` (break, step, print,
backtrace), optional `valgrind`.

**Why here:** this is your C learning lab. Every experiment for Parts A and B can
run here in seconds, and the plan's host tests use the same setup.

**Self-check:**
- What does each of the §4.7 flags enable or turn into an error?
- You get a segfault on the host. Which two tools tell you the exact line?
- Read a `Makefile` rule (`target: deps` + recipe). What do `$@` and `$<` mean?

### D.2 ESP-IDF conventions (Needed: T4 onward)

**What:** `esp_err_t` and `ESP_ERROR_CHECK` (aborts!) vs handling the error;
`ESP_LOGx` and tags; handles and the `*_new_*` / `*_del_*` lifecycle
(`i2c_new_master_bus`, `i2c_master_bus_add_device`); driver install functions;
components and `REQUIRES` in CMake; `menuconfig` / `sdkconfig`; panic output and
backtraces (`idf.py monitor` decodes addresses for you).

**Why here:** everything on target. The plan warns about the legacy vs new I²C
API: knowing the handle-based pattern helps you spot the legacy one.

**Self-check:**
- When is `ESP_ERROR_CHECK` the right choice, and when would it turn a recoverable
  servo timeout into a reboot?
- In the new I²C API, what's the difference between the *bus* handle and the
  *device* handle?
- A "Guru Meditation Error: LoadProhibited" with an address near 0: what kind of
  bug is it most likely?
- Where does a component declare that it depends on `driver` or `esp_timer`?

### D.3 NVS (Needed: T10, T16)

**What:** key-value flash storage; namespaces; blobs; `nvs_commit`; first-boot
"not found" handling; blob versioning.

**Why here:** `calib_t` lives in NVS (§3.1, §4.6 step 1).

**Self-check:**
- What does `nvs_get_blob` return when the key doesn't exist? What should boot do
  then (§4.6)?
- Why add a version or size field to a stored struct?

### D.4 Shared libraries and `ctypes` (Depth: T7 only, cuttable)

**What:** `-shared -fPIC`, symbol visibility, calling a C function from Python,
mirroring structs in `ctypes.Structure`, pointer arguments (`byref`).

**Self-check:**
- Why does `ctypes` need you to declare `argtypes`/`restype`? What happens if you
  pass a Python float where C expects a `float *`?
- If `stab_in_t` gains a field in C but not in Python, what goes wrong, and does
  it crash or silently produce garbage?

---

## E. Task → topic matrix

Status from the plan: T2, T3a, T3b, T3c are done. Remaining work needs:

| Task | Core topics | Needed topics |
|---|---|---|
| T4 MPU6050 + 1 kHz loop + logger | A.1–A.7, A.10, B.1, B.2, D.1 | A.3 (endianness), A.9, C.1, C.2, C.3 (notify), C.4, B.4 (IRAM ISR), D.2 |
| T5 Mahony (Python → C) | A.2, A.5–A.7 | A.11, A.12, D.1 (reading a CSV log in a host test) |
| T7 ctypes sim (stretch) | A.6, A.7 | A.8, A.12, D.4 |
| T8 XL-320 driver | A.3, A.5, A.6 | A.9, A.10 (`assert`), A.13, C.4, D.2 |
| T9 Control task + boot | B.1, B.2 | B.3, C.1–C.4, D.2 |
| T10 3 servos + NVS | A.7 | A.8, B.4, D.3 |
| T11/T12 Stabilisation + tuning | A.11, A.12 | C.4 (logging without breaking timing) |
| T13 Protocol both sides | A.3, A.6, A.7 | A.8, A.9, A.13, C.3, D.2 |
| T14 Link robustness | C.2, C.3 | C.4 |
| T16–T20 | everything above, applied | panic/backtrace reading (D.2) |

**Minimum path to unblock T4** (the next C task):
A.1 → A.2 → A.3 → A.5 → A.6 → A.7 → B.1 → B.2 → C.1 → C.2 → C.3 (notifications only).

---

## F. Sources

Read alongside the ESP-IDF docs you already have open.

**C language**
- *Beej's Guide to C Programming* (free, online). Easiest on-ramp, good on pointers.
- *Modern C*, Jens Gustedt (free PDF from the author). C11/C17, rigorous about
  types, UB and storage duration (Part B).
- *Effective C*, Robert Seacord. Short, focused on writing *correct* C
  (conversions, UB, memory).
- *The C Programming Language*, Kernighan & Ritchie. Classic and compact; the
  bitwise and pointer chapters are still the best short treatment.
- cppreference.com, **C** section (not C++): the reference for `<stdint.h>`,
  `<math.h>`, conversions, storage duration.
- SEI CERT C Coding Standard (free online): a catalogue of rules with "why" for
  most of Part B.5.

**Embedded / RTOS**
- *Mastering the FreeRTOS Real Time Kernel* (free PDF, freertos.org): tasks,
  queues, notifications, ISR rules. Read the chapters on queues and task
  notifications before T9.
- ESP-IDF Programming Guide (v6.1): *FreeRTOS (IDF)* differences page (stack in
  bytes, SMP, core pinning), *Memory Types* (IRAM/DRAM), *Fatal Errors* (reading
  panics), *Error Handling* (`esp_err_t`).
- *Making Embedded Systems*, Elecia White: design-level view (state machines,
  interrupts, memory). Depth reading.

**Tools**
- Compiler Explorer (godbolt.org): paste a tiny function, see what `float` vs
  `double` or `-O0` vs `-O2` actually produces. Very good for A.2 and B.5.
- GCC manual, *Warning Options* and *Instrumentation Options* (sanitizers).
