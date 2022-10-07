#include "bindings.h"
#include <stdio.h>

// gcc -pthread foo.c bindings.a -o foo; ./foo

int main(int argc, char **argv) {
    GoString kinFile = {"/home/peter/Documents/viam/rdk/components/arm/xarm/xarm7_kinematics.json", 72};
    Init(kinFile);
    GoFloat64 jointData[] = {0, 0, 1, 1, 0, 0, 1};
    GoSlice joints = {jointData, 7, 7};
    GoString armName = {"arm", 3};
    
    struct pose* p;
    
     p = ComputePositions(armName, joints);
    
    printf("X: %f, Y: %f, Z: %f\n", p->X, p->Y, p->Z);
    
    return 0;
}
