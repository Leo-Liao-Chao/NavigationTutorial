#include <stdio.h>
#include "myutil.h"

using namespace std;

void GetCudaDeviceCount( int &num_gpus )
{
    cudaGetDeviceCount( &num_gpus );

    for ( int i = 0; i < num_gpus; ++ i )
    {
        cudaDeviceProp dprop;
        cudaGetDeviceProperties(&dprop, i);
        printf("   %d: %s\n", i, dprop.name);
    }
}