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

## Documentation

The entire functionality is not explained here. Things not explained here should be straightforward.

### Scalars

Most of the scalar functions are wrappers for functions from cmath. This is done in case their implementations will need change in future. 
They are mostly for internal use, but can be used outside for consistency.

Other functions are described here.

#### degreesToRadians(value)/radiansToDegrees(value)

Converts a value from degrees to radians and wise versa.

#### equalScalar(a, b, epsilon)

Floating point comparison with optional epsilon argument. Default epsilon is static value: 0.0000001.

#### lerpScalar(a, b, t)

Linear interpolation between two values.

#### fractScalar(value)

Returns the fractional part of the floating point value.

#### fractScalarConst(value)

A more performant, constexpr version of the same function. Has precision issues.


### Vectors

All of the mathematical functions on Vectors are applied per element. And Vector by Vector operations are added element wise.

#### dot(a, b)

Dot product of two Vectors.

#### cross(a, b)

Cross product of two Vectors. Only available for 2 and 3 dimensional Vectors.

#### angle(a, b)

Returns the angle between two Vectors. Returned value is in degrees.

#### filledVector(value)

Creates a Vector filled with given value. Example: filledVector<float, 3>(1.0f);


### Matrix

#### filledMatrix(value)

Creates a Matrix filled with given value. Example: filledMatrix<float, 3, 3>(1.0f);

#### gaussElim(matrix)

Performs a Gaussian Elimination with partial pivoting. Returns Matrix in row echelon form and performed row permutations.

#### gaussElim(matrix, vector)

Solves for x in M * x = v using Gaussian Elimination.

#### plu(matrix, vector)

Performs PLU decomposition. Returns Upper and Lower Matrices along with row permutations.

#### determinant(matrix)

Calculates the determinant of the Matrix using Gaussian Elimination. For 2x2 and 3x3 Matrices uses special implementations for speed.

#### inverse(matrix)

Calculates the inverse of the Matrix using PLU decomposition. With determinant of 0 Matrix with all zeros is returned. 
2x2 Matrix calculates adjugate/determinant.

#### transformMatrix(position, rotation, scale)

Composes the transform Matrix. Has scale applied first, rotation second, position third.

#### extractTransform(matrix, &position, &rotation, &scale)

Decomposes the transform Matrix composed by transformMatrix function.


### Quaternion

#### conjugate(quaternion)

Conjugate of the quaternion. For unit quaternion can be used in place of inverse.


### Permutation1D

#### inverse(permutation)

Generates the inverse permutation of the given permutation.

#### toMatrix(permutation)

Converts the permutation to row permutation Matrix.


### Types

#### Vector

The data inside can be accessed through indexing operator. Example vec[1]. 
Accessing data this way asserts on out of bounds indexes in debug builds.

Alternatively the function at(index) can be used. Example: vec.at(1). 
This function throws out_of_range exception on out of bounds indexes.

**cast()** function expands or truncates the Vector depending on given size. Example: vec.cast<5>().

Internal data pointers can be accessed with begin() and end().

#### Matrix

When brace initializing Matrix excessive elements are just ignored.

```C++
lm2::Matrix<float, 2, 2> mat{
  { 1, 2, 3 },
}
```
The element 3 here will be silently ignored.

The elements in Matrix can be accessed and modified with operator()(row, column). Example: mat(1, 2) = 5.

Alternatively at(row, column) function can be used. This function throws on out of bounds indexes.

**cast()** function truncates or expands the Matrix. When expanding diagonal elements are set to 1. Example: mat.cast<4, 4>().

Functions **swapRow(aRow, bRow)/swapCol(aCol, bCol)** swap elements from given rows/columns.

#### Quaternion

Quaternion type is aggregate type. The elements are stored in: w, x, y, z order.

#### Permutation1D

This class is a quality of life addition, mostly for internal use with functions using partial pivoting.

**isPositive()** function returns if current permutation is positive.

The elements in Permutation1D can only be read not edited. Otherwise the access functions are same as Vector's.

**swap(a, b)** function swaps two indexes.

**move(a, b)** function move index from a to b.

#### PLUData

This type is outputted from **plu(matrix)** function.

**solve(vector)** function is used to solve for x in M * x = v.


### Extra

By default asserts are used throughout the library. They can be disabled by defining LM2_NOASSERT before including the library.

For optimization of the binary size LM2_NO_OUTPUT_FUNCTIONS can be defined before including the library. This will get rid of logging functions. 
If enabled **std::cout << vector** won't work.

Noexcept specifiers on the functions can be disabled by defining LM2_NOEXCEPT before including.
