#include <iostream>
#include <omp.h>

int main() {
    double start_time = omp_get_wtime();
    std::cout << "Hello from main function" << std::endl;
    double end_time = omp_get_wtime();
    std::cout << "Time taken: " << (end_time - start_time) << " seconds" << std::endl;
    return 0;
}