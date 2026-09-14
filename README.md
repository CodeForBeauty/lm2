# lm2

A simple single-header C++ linear math library.

## Overview

This library is not designed for performance or complex linear algebra.

It's primary purpose is to be lighweight and easy to use. Mainly being designed for graphics programming.

## How to use

Minimum tested standard: C++ 14.

The file lm2.hpp can be copied somewhere into target project's include directory. It then can be included and used where needed.

### Usage examples

All of the code lives inside lm2 namespace.

All the types use templates for their sizes, similar to std::array.

Vector:

```C++
lm2::Vector<float, 3> vec{ 1.0f, 3.0f, 4.0f };
vec *= 2.0f;
std::cout << vec << "\n";
```

```C++
lm2::Vector<float, 3> vec1{ 0.0f, 3.0f, 0.0f };
lm2::Vector<float, 3> vec2{ 1.5f, 1.5f, 0.0f };
float dotProduct = lm2::dot(vec1, vec2);
```

Matrix:

```C++
lm2::Matrix<float, 3, 3> rotMat{ lm2::eulerRotation3DMatrix<float>({ 45.0f, 0.0f, 9.0f }) };
lm2::Vector<float, 3> vec{ 1.0f, 3.0f, 4.0f };
lm2::Vector<float, 3> rotated = rotMat * vec;
```

It is recommended to define types with frequently used sizes like this:

```C++
using vec3 = lm2::Vector<float, 3>;
using mat3 = lm2::Matrix<float, 3, 3>;
```

For more usage examples you can look through test.cpp.

## Specification

Matrices use row-major storage for easier row iteration.

Vectors are treated differently depending on how they are multiplied by Matrices. Multiplying Matrix by Vector treats Vector as column-vector. 
And multiplying Vector by Matrix treats it as row-vector.

All of the internal Matrix functions are made to work with column-vectors. So it is recommended to multiply Matrix by Vector most of the time.

Multiplying Permutation1D with Matrix permutes Matrix's rows. Multiplying Matrix with Permutation1D permutes Matrix's columns.

Multiplying two Matrices goes in left to right order. Multiplying a * b * c matrices would mean: apply a first, b second and c last.
This differs from general row-major matrix multiplication as internal matrix multiplication logic is flipped to achieve this.

Quaternions are aggregate type, and have values of zero when not initialized. Using function lm2::identityQuaternion is recommended for unitialized quaternions.

## Tests

A simple unit tests are added into the project. To run them you will need to compile lm2_Tests.cpp file.

g++ example:

```bash
g++ lm2_Tests.cpp -std=c++14 -o test
test
```

Expected output:

```
Running tests from: lm2_Tests.cpp
  GaussElimSolver
    passed
  PLUSolver
    passed
  MatrixInverse
    passed
  OrthographicProjection
    passed
  PerspectiveProjection
    passed
  TransformDecomposition
    passed
  MatrixEulerConversion
    passed
  QuaternionConversions
    passed
Passed: 8, Failed: 0
```

## Considerations

The library heavily uses templates. This can increase build times quite considerably. 

All of the types use stack memory only. Working with large Matrices or Vectors is not recommended.

Most of the functions are marked as noexcept. This behaviour can be disabled by defining LM2_NOEXCEPT before including the file.

Most of the functions return a new object.
