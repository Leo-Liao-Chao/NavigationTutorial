#include <iostream>

extern "C" void cudaFunction() {
    std::cout << "Hello from CUDA" << std::endl;
}