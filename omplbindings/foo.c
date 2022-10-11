#include "bindings.h"
#include <stdio.h>

// gcc -pthread foo.c bindings -lnlopt -o foo; ./foo

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
    
    return 0;
}
