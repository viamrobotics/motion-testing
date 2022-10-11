#include "bindings.h"
#include <stdio.h>

// gcc -pthread foo.c bindings -lnlopt -o foo; ./foo

int main(int argc, char **argv) {
    GoString scene = {"scene2", 6};
    Init(scene);
    GoFloat64 jointData[] = {0, 0, 1, 1, 0, 0, 1};
    GoSlice joints = {jointData, 7, 7};
    
    struct pose* p;
    
     p = ComputePositions(joints);
    
    printf("X: %f, Y: %f, Z: %f\n", p->X, p->Y, p->Z);
    
    return 0;
}
