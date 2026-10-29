#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include <assert.h>
#include "mahony.h"

// 3 Test-cases the Mahony Filter implementation must pass
// Match Golden, Converge from a Tilt, Ignore bad Accelerometer
// Add visualization later?

#define DT 0.001f

static void test_golden(const char *path) {
    FILE *f = fopen(path, "r");
    assert(f && "golden_mahony.csv not found (run tools/gen_golden_mahony.py)");

    char line[512];
    mahony_t m;
    mahony_init(&m, 1.0f, 0.02f);
    int n = 0;
    float worst = 0.0f;
    
    while (fgets(line, sizeof line, f)) {
        if (line[0] == '#') continue; // Gets rid of comment
        float g[3], a[3], q[4];
        int got = sscanf(line, "%f,%f,%f,%f,%f,%f,%f,%f,%f,%f",
                         &g[0], &g[1], &g[2], 
                         &a[0], &a[1], &a[2], 
                         &q[0], &q[1], &q[2], &q[3]);
    
        assert(got == 10);
        mahony_update(&m, g, a, DT);
        float dot = m.q.w*q[0] + m.q.x*q[1] + m.q.y*q[2] + m.q.z*q[3];
        float s = dot < 0.0f ? -1.0f : 1.0f;      // q and -q are the same rotation
        float d[4] = {m.q.w - s*q[0], m.q.x - s*q[1], m.q.y - s*q[2], m.q.z - s*q[3]};
        for (int i = 0; i < 4; i++) if (fabsf(d[i]) > worst) worst = fabsf(d[i]);
        n++;
    }
    fclose(f);
    printf("golden: %d rows, worst component error %.2e\n", n, (double)worst);
    assert(n > 0 && worst < 1e-4f);
}

static void test_static_converges(void) {
    // Camera pitched 30 deg about +y: R_wc[2] = (-sin th, 0, cos th); accel = -R_wc[2]
    float th = 0.5235988f;
    float a[3] = {sinf(th), 0.0f, -cosf(th)};
    float g[3] = {0, 0, 0};

    mahony_t m;
    mahony_init(&m, 1.0f, 0.0f);
    
    for (int i = 0; i < 10000; i++){ 
        mahony_update(&m, g, a, DT);
    }

    mat3_t R;
    mahony_get_R(&m, &R);
    assert(fabsf(R.m[2][0] - (-sinf(th))) < 1e-3f);
}

static void test_gate_skips_accel(void) {
    float g[3] = {0, 0, 0};
    float a[3] = {0, 0, -2.0f}; // 2 g: way outside the gate

    mahony_t m;
    mahony_init(&m, 1.0f, 0.02f);
    for (int i = 0; i < 100; i++) {
        mahony_update(&m, g, a, DT);
    }

    assert(m.q.w == 1.0f && m.q.x == 0.0f && m.q.y == 0.0f && m.q.z == 0.0f);
}

static void test_integral_clamped(void) {
    // Level camera, but gyro reports a constant 0.2 rad/s about x (far above i_max).
    // The integral must saturate at -i_max instead of winding up.
    float g[3] = {0.2f, 0, 0}, a[3] = {0, 0, -1.0f};
    mahony_t m;
    mahony_init(&m, 1.0f, 0.2f);
    for (int i = 0; i < 20000; i++) mahony_update(&m, g, a, DT);
    assert(fabsf(m.integ[0]) <= m.i_max);

    // If really saturated
    assert(fabsf(m.integ[0]) > 0.99f * m.i_max);
}

int main(void) {
    test_golden("golden_mahony.csv");
    test_static_converges();
    test_gate_skips_accel();
    test_integral_clamped();
    printf("test_mahony: OK\n");
    return 0;
}