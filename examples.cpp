/*
 * lm2 usage examples
 */

#include "lm2.hpp"

#include <iostream>
#include <iomanip>
#include <cmath>

using namespace lm2;

// Small helper to pretty-print section headers
void section(const char* name) {
    std::cout << "\n=== " << name << " ===\n";
}

int main() {
    std::cout << std::fixed << std::setprecision(4);

    // -------------------------------------------------------------------------
    section("1. Vector basics");

    Vector<float, 3> a{ 1.0f, 2.0f, 3.0f };
    Vector<float, 3> b{ 4.0f, 5.0f, 6.0f };

    std::cout << "a            = " << a << "\n";
    std::cout << "b            = " << b << "\n";
    std::cout << "a + b        = " << (a + b) << "\n";
    std::cout << "a - b        = " << (a - b) << "\n";
    std::cout << "a * b        = " << (a * b) << "\n";
    std::cout << "a * 2.0f     = " << (a * 2.0f) << "\n";
    std::cout << "2.0f * a     = " << (2.0f * a) << "\n";
    std::cout << "-a           = " << (-a) << "\n";

    // -------------------------------------------------------------------------
    section("2. Vector math");

    std::cout << "dot(a, b)        = " << dot(a, b) << "\n";
    std::cout << "cross(a, b)      = " << cross(a, b) << "\n";
    std::cout << "magnitude(a)     = " << magnitude(a) << "\n";
    std::cout << "magnitudeSquared = " << magnitudeSquared(a) << "\n";
    std::cout << "normalize(a)     = " << normalize(a) << "\n";
    std::cout << "distance(a, b)   = " << distance(a, b) << "\n";
    std::cout << "angle(a, b)      = " << angle(a, b) << " degrees\n";
    std::cout << "lerp(a, b, 0.25) = " << lerp(a, b, 0.25f) << "\n";
    std::cout << "reflect(a, up)   = " << reflect(a, Vector<float,3>{0,1,0}) << "\n";

    // -------------------------------------------------------------------------
    section("3. Vector creation helpers");

    std::cout << "zeroVector<3>       = " << zeroVector<float, 3>() << "\n";
    std::cout << "filledVector<3>(7)  = " << filledVector<float, 3>(7.0f) << "\n";

    // -------------------------------------------------------------------------
    section("4. Component-wise operations");

    Vector<float, 3> v{ -1.5f, 2.3f, 0.7f };
    std::cout << "v          = " << v << "\n";
    std::cout << "abs(v)     = " << abs(v) << "\n";
    std::cout << "floor(v)   = " << floor(v) << "\n";
    std::cout << "ceil(v)    = " << ceil(v) << "\n";
    std::cout << "round(v)   = " << round(v) << "\n";
    std::cout << "fract(v)   = " << fract(v) << "\n";
    std::cout << "sign(v)    = " << sign(v) << "\n";
    std::cout << "clamp(v,-1,1) = " << clamp(v, -1.0f, 1.0f) << "\n";
    std::cout << "min(v, 0)  = " << min(v, 0.0f) << "\n";
    std::cout << "max(v, 0)  = " << max(v, 0.0f) << "\n";

    // -------------------------------------------------------------------------
    section("5. Matrix basics");

    Matrix<float, 3, 3> m{
        { 1, 2, 3 },
        { 4, 5, 6 },
        { 7, 8, 9 },
    };
    std::cout << "m = \n" << m << "\n\n";
    std::cout << "transpose(m) = \n" << transpose(m) << "\n\n";
    std::cout << "identityMatrix<3> = \n" << identityMatrix<float, 3>() << "\n\n";

    // Non-square matrices work too
    Matrix<float, 2, 3> rect{
        { 1, 2, 3 },
        { 4, 5, 6 },
    };
    std::cout << "rect (2x3) = \n" << rect << "\n\n";
    std::cout << "transpose(rect) (3x2) = \n" << transpose(rect) << "\n\n";

    // -------------------------------------------------------------------------
    section("6. Matrix-vector multiplication");

    Matrix<float, 3, 3> rotM = eulerRotation3DMatrix<float>(
        Vector<float, 3>{ 0.0f, 45.0f, 0.0f });  // 45 degrees around Y

    Vector<float, 3> point{ 1.0f, 0.0f, 0.0f };
    Vector<float, 3> rotated = rotM * point;

    std::cout << "rotM (45 deg around Y) = \n" << rotM << "\n\n";
    std::cout << "point            = " << point << "\n";
    std::cout << "rotM * point     = " << rotated << "\n";
    std::cout << "expected         = (0.7071, 0, -0.7071)\n";

    // -------------------------------------------------------------------------
    section("7. Matrix-matrix multiplication");

    Matrix<float, 3, 3> scale{
        { 2, 0, 0 },
        { 0, 3, 0 },
        { 0, 0, 4 },
    };

    Matrix<float, 3, 3> combined = scale * rotM;
    std::cout << "scale * rotM = \n" << combined << "\n\n";
    std::cout << "(scale * rotM) * point = " << (combined * point) << "\n";
    std::cout << "scale * (rotM * point) = " << (scale * rotated) << "\n";

    // -------------------------------------------------------------------------
    section("8. Transforms");

    Vector<float, 3> position{ 10.0f, 0.0f, 0.0f };
    Vector<float, 3> euler   { 0.0f, 90.0f, 0.0f };
    Vector<float, 3> scl     { 2.0f, 2.0f, 2.0f };

    Matrix<float, 4, 4> transform =
        positionMatrix(position) *
        eulerRotation3DMatrix(euler).cast<4, 4>() *
        scaleMatrix(scl).cast<4, 4>();

    Vector<float, 4> localPoint{ 1.0f, 0.0f, 0.0f, 1.0f };
    Vector<float, 4> worldPoint = transform * localPoint;

    std::cout << "transform = \n" << transform << "\n\n";
    std::cout << "localPoint  = " << localPoint << "\n";
    std::cout << "worldPoint  = " << worldPoint << "\n";

    // Extract position/scale from the transform
    Vector<float, 3> extractedPos = extractPosition(transform).cast<3>();
    Vector<float, 3> extractedScale = extractScale(transform.cast<3, 3>());
    std::cout << "extracted position = " << extractedPos << "\n";
    std::cout << "extracted scale    = " << extractedScale << "\n";

    // -------------------------------------------------------------------------
    section("9. View and projection");

    Matrix<float, 4, 4> view = viewMatrix(
        Vector<float, 3>{ 0.0f, 0.0f, 5.0f },   // eye
        Vector<float, 3>{ 0.0f, 0.0f, 0.0f },   // at
        Vector<float, 3>{ 0.0f, 1.0f, 0.0f });  // up

    Matrix<float, 4, 4> proj = perspectiveProjection(
        60.0f,   // fov (degrees)
        0.1f,    // near
        100.0f,  // far
        16.0f/9.0f); // aspect ratio

    Vector<float, 4> worldP{ 0.0f, 0.0f, 0.0f, 1.0f };
    Vector<float, 4> viewP  = view * worldP;
    Vector<float, 4> clipP  = proj * viewP;
    Vector<float, 4> ndcP   = clipP / clipP[3];

    std::cout << "world      = " << worldP << "\n";
    std::cout << "view       = " << viewP << "\n";
    std::cout << "clip       = " << clipP << "\n";
    std::cout << "ndc        = " << ndcP << "\n";

    // -------------------------------------------------------------------------
    section("10. Linear solvers");

    Matrix<float, 3, 3> A{
        { 2, 1, 1 },
        { 1, 3, 2 },
        { 1, 0, 0 },
    };
    Vector<float, 3> rhs{ 4.0f, 5.0f, 6.0f };

    // Solve A * x = rhs using PLU
    auto pluResult = plu(A);
    Vector<float, 3> x = pluResult.solve(rhs);

    std::cout << "A = \n" << A << "\n\n";
    std::cout << "rhs       = " << rhs << "\n";
    std::cout << "x         = " << x << "\n";
    std::cout << "A * x     = " << (A * x) << "  (should equal rhs)\n";

    // Gaussian elimination variant
    auto gaussResult = gaussElim(A, rhs);
    std::cout << "gauss x   = " << gaussResult.result << "\n";

    // -------------------------------------------------------------------------
    section("11. Determinant and inverse");

    std::cout << "determinant(A) = " << determinant(A) << "\n";

    Matrix<float, 3, 3> Ainv = inverse(A);
    std::cout << "inverse(A) = \n" << Ainv << "\n\n";
    std::cout << "A * inverse(A) = \n" << (A * Ainv) << "\n\n";
    std::cout << "inverse(A) * A = \n" << (Ainv * A) << "\n\n";

    // -------------------------------------------------------------------------
    section("12. Quaternions");

    // From axis-angle
    Quaternion<float> q1 = makeQuaternion(
        Vector<float, 3>{ 0.0f, 1.0f, 0.0f },  // axis
        90.0f);                                 // angle in degrees

    // From Euler
    Quaternion<float> q2 = makeQuaternion(Vector<float, 3>{ 0.0f, 45.0f, 0.0f });

    std::cout << "q1 (axis-angle)  = " << q1 << "\n";
    std::cout << "q2 (euler)       = " << q2 << "\n";
    std::cout << "magnitude(q1)    = " << magnitude(q1) << "\n";
    std::cout << "normalize(q1)    = " << normalize(q1) << "\n";
    std::cout << "conjugate(q1)    = " << conjugate(q1) << "\n";

    // Rotate a vector with a quaternion
    Vector<float, 3> qPoint{ 1.0f, 0.0f, 0.0f };
    Vector<float, 3> qRotated = rotate(q1, qPoint);
    std::cout << "rotate(q1, (1,0,0)) = " << qRotated << "\n";
    std::cout << "expected            = (0, 0, -1)\n";

    // Quaternion to matrix, and back
    Matrix<float, 3, 3> qMat = toMatrix(q1);
    std::cout << "toMatrix(q1) = \n" << qMat << "\n\n";

    Quaternion<float> qFromMat = makeQuaternion(qMat);
    std::cout << "makeQuaternion(toMatrix(q1)) = " << qFromMat << "\n";

    // -------------------------------------------------------------------------
    section("13. Equality with epsilon");

    Vector<float, 3> v1{ 1.0f,        2.0f,        3.0f };
    Vector<float, 3> v2{ 1.0f + 1e-8f, 2.0f - 1e-8f, 3.0f + 1e-8f };

    std::cout << "v1 = " << v1 << "\n";
    std::cout << "v2 = " << v2 << "\n";
    std::cout << "equal(v1, v2) = " << (equal(v1, v2) ? "true" : "false") << "\n";

    // -------------------------------------------------------------------------
    section("14. Integer vectors");

    Vector<int, 4> iv{ 10, 20, 30, 40 };
    std::cout << "iv      = " << iv << "\n";
    std::cout << "iv % 7  = " << (iv % 7) << "\n";
    std::cout << "iv + 5  = " << (iv + 5) << "\n";

    // -------------------------------------------------------------------------
    section("15. Iteration");

    Vector<float, 3> it{ 1.0f, 2.0f, 3.0f };
    float sum = 0.0f;
    for (float f : it) {
        sum += f;
    }
    std::cout << "sum of components of " << it << " = " << sum << "\n";

    // -------------------------------------------------------------------------
    section("Done");
    std::cout << "All examples ran successfully.\n";
    return 0;
}
