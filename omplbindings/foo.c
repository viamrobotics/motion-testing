#include "bindings.h"
#include <stdio.h>
#include <stdint.h>

// gcc -pthread foo.c ./bindings -lnlopt -o foo; ./foo

int main(int argc, char **argv) {
    GoString scene = {"scene2", 6};
    Init(scene);
    GoFloat64 jointData[] = {0, -1, -0.5, 0.5, 0, 1};
    GoSlice joints = {jointData, 6, 6};

    struct pose* p;
    p = ComputePositions(joints);
    printf("X: %f, Y: %f, Z: %f\n", p->X, p->Y, p->Z);

    int valid = ValidState(joints);
    printf("valid? %d\n", valid);

    double *res = StartPos();
    for (int i = 0; i < 6; i++){
        printf("%f\n", res[i]);
    }

    p->X += 100;
    p->Y += 100;
    double *ik_joints = ComputePose(p);
    for (int i = 0; i < 6; i++){
        printf("%f\n", ik_joints[i]);
    }

    struct limits* lim;
    lim = Limits();
    for (int i = 0; i < 6; ++i)
    {
        printf("Min: %f , Max: %f\n", lim[i].Min, lim[i].Max);
    }

    return 0;
}
