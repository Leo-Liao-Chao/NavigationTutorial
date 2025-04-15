#include <omp.h>
#include <stdio.h>
#include "myutil.h"

int main(int argc, char *argv[])
{
    int num_gpus = 0; 

    GetCudaDeviceCount( num_gpus );

    printf("number of host CPUs:\t%d\n", omp_get_num_procs());
    printf("number of CUDA devices:\t%d\n", num_gpus);

    #pragma omp parallel
    {
        printf("Hello Cuda + OpenMP!\n");
    }

    return 0;
}