#include <iostream>
#include "lm2.hpp"

int main() {
    lm2::vec3 v1 { 1, 1, 1 };
    lm2::vec3 v2 { 5, 5, 5 };

    std::cout << v1 * v2 << "\n";

    std::cout << lm2::degrees2radians(90.0f) << "\n";

    return 0;
}