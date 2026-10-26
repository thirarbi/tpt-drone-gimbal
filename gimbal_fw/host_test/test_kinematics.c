#include <stdio.h>
#include <math.h>
#include <string.h>
#include "kinematics.h"

// Tolerances
#define TOL_Q 1e-4f // Compares joint angles
#define TOL_R 1e-5f // Matrix entries

int main(void) {
    // cwd = host_test (make -C)
    FILE *f = fopen("golden_kinematics.csv", "r");
    if (!f) { perror("golden_kinematics.csv"); return 1; }

    char line[512]; // 512bytes buffer to hold one line of text form csv
    int n = 0;
    int bad = 0;

    // Check the forward kinematics against python's C, then
    // check C inverse kinematics against python's Q
    while (fgets(line, sizeof line, f)) {
        if (line[0] == '#') continue; // Skips comment
        float q[3], Rg[9]; // 3 Joint angles and 9 golden Rotation matrix
        if (sscanf(line, "%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f",
                   &q[0], &q[1], &q[2],
                   &Rg[0], &Rg[1], &Rg[2],
                   &Rg[3], &Rg[4], &Rg[5],
                   &Rg[6], &Rg[7], &Rg[8]) != 12) { 
                    bad++; continue; 
                }
        mat3_t R;
        gimbal_fk(q, &R);
        for (int i = 0; i < 9; i++) {
            if (fabsf(R.m[i / 3][i % 3] - Rg[i]) > TOL_R) {
                bad++; printf("FK row %d [%d]\n", n, i);
            }
        }
        
        mat3_t Rin;
        memcpy(Rin.m, Rg, sizeof Rg); // Golden R -> IK
        float q2[3];
        gimbal_ik(&Rin, q2);
        for (int i = 0; i<3 ; i++) {
            if (fabsf(wrap_pi(q2[i] - q[i])) > TOL_Q) {
                bad++; printf("IK row %d [%d]\n", n, i);
            }
        }
        n++;
    }
    fclose(f);
    printf("test_kinematics: %d rows, %d failures\n", n, bad);
    return bad != 0;
}

