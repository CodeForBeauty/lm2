#include <iostream>
#include "lm2.hpp"

int main() {
    std::cout << lm2::degrees2radians(90.0f) << "\n";

    lm2::Vector<int, 3> t{ 1 };

    std::cout << t[2] << "\n";

    lm2::Vector<int, 4> t1 = t.cast<4>();

    std::cout << t1[0];

    return 0;
}
