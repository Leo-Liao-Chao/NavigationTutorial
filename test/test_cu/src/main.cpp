#include <omp.h>
#include <iostream>

extern "C" void cudaFunction();

int main() {
    // 使用 OpenMP
    #pragma omp parallel
    {
        std::cout << "Hello from thread " << omp_get_thread_num() << std::endl;
    }

    // 调用 CUDA 函数
    cudaFunction();

    return 0;
}