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

template<typename T, size_t NRow, size_t NCol>
void assertMatrixEqual(const Matrix<T, NRow, NCol>& a, const Matrix<T, NRow, NCol>& b, T epsil) {
    for (size_t i = 0; i < NRow; i++) {
        for (size_t j = 0; j < NCol; j++) {
            ASSERT_EQUAL_FLOAT(a(i, j), b(i, j), equalScalar, epsil);
        }
    }
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

TEST_CASE(MatrixInverse) {
    Matrix<float, 3, 3> rotMat = eulerRotation3DMatrix<float>({ 45.0f, 0.0f, 25.0f });

    Vector<float, 3> startPos{ 2.0f, 3.0f, 4.0f };

    auto invRot = inverse(rotMat);

    ASSERT_EQUAL_FLOAT(startPos, rotMat * invRot * startPos, equal, float_epsilon);

    auto identity = identityMatrix<float, 3>();

    auto mulRes = rotMat * invRot;

    assertMatrixEqual(identity, mulRes, float_epsilon);
}

TEST_CASE(OrthographicProjection) {
    auto proj = orthographicProjection(-2.0f, 2.0f, -3.0f, 3.0f, 1.0f, 10.0f);

    Vector<float, 4> corner{2.0f, 3.0f, -1.0f, 1.0f};
    Vector<float, 4> ndc = proj * corner;

    ASSERT_EQUAL_FLOAT(1.5f, ndc[0] / ndc[3], equalScalar, float_epsilon);
    ASSERT_EQUAL_FLOAT(1.0f, ndc[1] / ndc[3], equalScalar, float_epsilon);
    ASSERT_EQUAL_FLOAT(-1.0f, ndc[2] / ndc[3], equalScalar, float_epsilon);
}

TEST_CASE(PerspectiveProjection) {
    auto proj = perspectiveProjection(90.0f, 1.0f, 100.0f, 1.0f);

    Vector<float, 4> nearEdge{1.0f, 0.0f, -1.0f, 1.0f};
    Vector<float, 4> ndc = proj * nearEdge;
    ndc = ndc / ndc[3];

    ASSERT_EQUAL_FLOAT(1.0f, ndc[0], equalScalar, float_epsilon);
    ASSERT_EQUAL_FLOAT(-1.0f, ndc[2], equalScalar, float_epsilon);

    Vector<float, 4> farCenter{0.0f, 0.0f, -100.0f, 1.0f};
    ndc = proj * farCenter;
    ndc = ndc / ndc[3];

    ASSERT_EQUAL_FLOAT(1.0f, ndc[2], equalScalar, float_epsilon);
}

TEST_CASE(TransformDecomposition) {
    Vector<float, 3> pos{ 1.0f, 2.0f, 5.0f };
    Vector<float, 3> rot{ 45.0f, 0.0f, 45.0f };
    Vector<float, 3> scale{ 2.0f, 3.0f, 1.0f };
    auto transform = transformMatrix<float>(pos, rot, scale);

    Vector<float, 3> dPos;
    Matrix<float, 3, 3> dRot;
    Vector<float, 3> dScale;
    extractTransform(transform, dPos, dRot, dScale);

    ASSERT_EQUAL_FLOAT(pos, dPos, equal, float_epsilon);
    ASSERT_EQUAL_FLOAT(scale, dScale, equal, float_epsilon);
    
    auto rotMat = eulerRotation3DMatrix(rot);
    assertMatrixEqual(rotMat, dRot, float_epsilon);
}

TEST_CASE(MatrixEulerConversion) {
    Vector<float, 3> startRot{ 45.0f, 25.0f, 60.0f };

    Rotation3DAxisOrder orders[6] = {
        Rotation3DAxisOrder::XYZ,
        Rotation3DAxisOrder::XZY,
        Rotation3DAxisOrder::YXZ,
        Rotation3DAxisOrder::YZX,
        Rotation3DAxisOrder::ZXY,
        Rotation3DAxisOrder::ZYX,
    };

    for (size_t i = 0; i < 6; i++) {
        auto mat = eulerRotation3DMatrix(startRot, orders[i]);
        auto eul = toEulerAngles(mat, orders[i]);
        auto mat1 = eulerRotation3DMatrix(eul, orders[i]);

        assertMatrixEqual(mat, mat1, float_epsilon);
    }
}

TEST_CASE(QuaternionConversions) {
    Vector<float, 3> startRot{ 45.0f, 25.0f, 60.0f };

    // Matrix
    {
        auto mat = eulerRotation3DMatrix(startRot);
        auto quat = makeQuaternion(mat);
        auto mat1 = toMatrix(quat);
        
        assertMatrixEqual(mat, mat1, float_epsilon);
    }
    
    // Axis angle
    {
        AxisAnglePair<float, 3> rot{ { 1.0f, 0.0f, 0.0f }, 45.0f };
        auto quat = makeQuaternion(rot.axis, rot.angle);
        auto rot1 = toAxisAngle(quat);
        
        ASSERT_EQUAL_FLOAT(rot.axis, rot1.axis, equal, float_epsilon);
        ASSERT_EQUAL_FLOAT(rot.angle, rot1.angle, equalScalar, 0.00001f);
    }
    
    // Euler
    {
        Rotation3DAxisOrder orders[6] = {
            Rotation3DAxisOrder::XYZ,
            Rotation3DAxisOrder::XZY,
            Rotation3DAxisOrder::YXZ,
            Rotation3DAxisOrder::YZX,
            Rotation3DAxisOrder::ZXY,
            Rotation3DAxisOrder::ZYX,
        };

        for (size_t i = 0; i < 6; i++) {
            auto quat = makeQuaternion(startRot, orders[i]);
            auto eul = toEulerAngles(quat, orders[i]);
            auto quat1 = makeQuaternion(eul, orders[i]);

            if ((quat.w < 0 && quat1.w > 0) || (quat.w > 0 && quat1.w < 0)) {
                quat1 = conjugate(quat1);
            }

            ASSERT_EQUAL_FLOAT(quat.x, quat1.x, equalScalar, float_epsilon);
            ASSERT_EQUAL_FLOAT(quat.y, quat1.y, equalScalar, float_epsilon);
            ASSERT_EQUAL_FLOAT(quat.z, quat1.z, equalScalar, float_epsilon);
            ASSERT_EQUAL_FLOAT(quat.w, quat1.w, equalScalar, float_epsilon);
        }
    }
}

int main() {
    return RUN_TESTS();
}
