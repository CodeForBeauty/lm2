#include <sstream>
#include <iomanip>

#include "testlib.hpp"
#include "lm2.hpp"

using namespace lm2;
using namespace std;

constexpr float float_epsilon = 0.000001f;

template<typename T, size_t N>
std::string to_string(const Vector<T, N>& vec) {
    std::ostringstream stream;
    stream << std::fixed << std::setprecision(7);
    
    for (size_t i = 0; i < N; i++) {
        stream << vec[i];
        if (i < N - 1) {
            stream << ", ";
        }
    }
    
    return stream.str();
}

template<typename T, size_t NRow, size_t NCol>
std::string to_string(const Matrix<T, NRow, NCol>& mat) {
    std::ostringstream stream;
    stream << std::fixed << std::setprecision(7);

    for (size_t i = 0; i < NRow; i++) {
        for (size_t j = 0; j < NCol; j++) {
            stream << mat(i, j);
            if (j < NCol - 1) {
                stream << ", ";
            }
        }
        if (i < NRow - 1) {
            stream << "\n";
        }
    }

    return stream.str();
}

TEST_CASE(GaussElimSolver) {
    Matrix<float, 3, 3> rotMat = eulerRotation3DMatrix<float>({ 45.0f, 0.0f, 25.0f });

    Vector<float, 3> startPos{ 2.0f, 3.0f, 4.0f };

    auto solution = gaussElim(rotMat, rotMat * startPos);

    ASSERT_EQUAL_FLOAT(startPos, solution.result, equal, float_epsilon);
}

TEST_CASE(PLUSolver) {
    Matrix<float, 3, 3> rotMat = eulerRotation3DMatrix<float>({ 45.0f, 0.0f, 25.0f });

    Vector<float, 3> startPos{ 2.0f, 3.0f, 4.0f };

    auto solver = plu(rotMat);

    ASSERT_EQUAL_FLOAT(startPos, solver.solve(rotMat * startPos), equal, float_epsilon);
}

int main() {
    return RUN_TESTS();
}
