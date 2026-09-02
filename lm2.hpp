/*
* A single header library for basic linear math
*/
#pragma once

#ifndef LM2_NO_OUTPUT_FUNCTIONS
#include <ostream>
#endif

#include <cmath>
#include <initializer_list>

namespace lm2 {

template<typename T> constexpr T PI = T(3.1415926535897932384626433832795);
template<typename T> constexpr T E = T(2.7182818284590452353602874713527);
template<typename T> constexpr T PIRAD = PI<T> / T(180);

namespace axes {
	constexpr size_t x = 0;
	constexpr size_t y = 1;
	constexpr size_t z = 2;
	constexpr size_t w = 3;
}

template<typename T> constexpr T EPSILON = T(0.0000001);

// Vector types
template<typename T, size_t N>
class Vector {
private:
	T data[N] {};

public:
	constexpr Vector() = default;
	constexpr Vector(const Vector<T, N>& lv) = default;
	constexpr Vector(Vector<T, N>& rv) = default;

	constexpr Vector(std::initializer_list<T> l) : data{} {
		size_t s = N < l.size() ? N : l.size();
		for (size_t i = 0; i < s; i++) {
			data[i] = *(l.begin() + i);
		}
	}

	constexpr T& operator[](size_t index) {
		return data[index];
	}

	constexpr const T& operator[](size_t index) const {
		return data[index];
	}

	template<size_t NOut>
	constexpr Vector<T, NOut> cast() const {
		Vector<T, NOut> output{};

		size_t s { N < NOut ? N : NOut };

		for (size_t i = 0; i < s; i++) {
			output[i] = data[i];
		}

		return output;
	}
};


template<typename T> struct vector3D;
template<typename T> struct vector4D;

template<typename T>
struct vector2D {
	T x, y;

	operator vector3D<T>() {
		return { x, y, 0 };
	}
	operator vector4D<T>() {
		return { x, y, 0, 0 };
	}
};

template<typename T>
struct vector3D {
	T x, y, z;

	operator vector4D<T>() {
		return { x, y, z, 0 };
	}
};

template<typename T>
struct vector4D {
	T x, y, z, w;
};

using vec2 = vector2D<float>;
using vec3 = vector3D<float>;
using vec4 = vector4D<float>;

// Matrix types
template<typename T, size_t NRow, size_t NCol>
class Matrix {
private:
	T data[NRow][NCol] {};

public:
	constexpr Matrix() = default;
	constexpr Matrix(const Matrix<T, NRow, NCol>& lv) = default;
	constexpr Matrix(Matrix<T, NRow, NCol>& rv) = default;

	constexpr Matrix(std::initializer_list<std::initializer_list<T>> l) : data{} {
		size_t s = NRow < l.size() ? NRow : l.size();
		for (size_t i = 0; i < s; i++) {
			size_t s1 = NCol < l.size() ? NCol : l.size();
			for (size_t j = 0; j < s1; j++) {
				data[i][j] = *((l.begin() + i)->begin() + j);
			}
		}
	}

	constexpr T& operator()(size_t rowIndex, size_t columnIndex) {
		return data[rowIndex][columnIndex];
	}

	constexpr const T& operator()(size_t rowIndex, size_t columnIndex) const {
		return data[rowIndex][columnIndex];
	}

	template<size_t NRowOut, size_t NColOut>
	constexpr Matrix<T, NRowOut, NColOut> cast() const {
		Matrix<T, NRowOut, NColOut> output{};

		size_t s { NRow < NRowOut ? NRow : NRowOut };
		size_t s1 { NCol < NColOut ? NCol : NColOut };

		for (size_t i = 0; i < s; i++) {
			for (size_t j = 0; j < s1; j++) {
				output(i, j) = data[i][j];
			}
		}

		return output;
	}
};


template<typename T> struct matrix3x3;
template<typename T> struct matrix4x4;

template<typename T>
struct matrix2x2 {
	vector2D<T> x, y;

	operator matrix3x3<T>() {
		return {
			{ x.x, x.y, 0 },
			{ y.x, y.y, 0 },
			{ 0,   0,   1 },
		};
	}
	operator matrix4x4<T>() {
		return {
			{ x.x, x.y, 0, 0 },
			{ y.x, y.y, 0, 0 },
			{ 0,   0,   1, 0 },
			{ 0,   0,   0, 1 },
		};
	}
};

template<typename T>
struct matrix3x3 {
	vector3D<T> x, y, z;

	operator matrix4x4<T>() {
		return {
			{ x.x, x.y, x.z, 0 },
			{ y.x, y.y, y.z, 0 },
			{ z.x, z.y, z.z, 0 },
			{ 0,   0,   0,   1 },
		};
	}
};

template<typename T>
struct matrix4x4 {
	vector4D<T> x, y, z, w;
};

using mat2 = matrix2x2<float>;
using mat3 = matrix3x3<float>;
using mat4 = matrix4x4<float>;

// Quaternion types
template<typename T>
struct quaternion_t {
	T w, x, y, z;
};

using quaternion = quaternion_t<float>;

// Scalar math functions
template<typename T>
constexpr T degrees2radians(T degrees) {
	return degrees * PIRAD<T>;
}
template<typename T>
constexpr T radians2degrees(T radians) {
	return radians / PIRAD<T>;
}

template<typename T>
constexpr T sqrtScalar(T val) {
	return std::sqrt(val);
}
template<typename T>
constexpr T absScalar(T val) noexcept {
	return std::abs(val);
}
template<typename T>
constexpr T fmodScalar(T val, T mod) {
	return std::fmod(val, mod);
}
template<> constexpr int fmodScalar<int>(int val, int mod) = delete;

template<typename T>
constexpr T sinScalar(T val) {
	return std::sin(val);
}
template<typename T>
constexpr T cosScalar(T val) {
	return std::cos(val);
}
template<typename T>
constexpr T tanScalar(T val) {
	return std::tan(val);
}

template<typename T>
constexpr T minScalar(T a, T b) noexcept {
	return a < b ? a : b;
}
template<typename T>
constexpr T maxScalar(T a, T b) noexcept {
	return a > b ? a : b;
}

// Basic math functions on vectors
template<typename T, size_t N>
constexpr Vector<T, N> degreesToRadians(const Vector<T, N>& vec) {
	return vec * PIRAD<T>;
}
template<typename T, size_t N>
constexpr Vector<T, N> radiansToDegrees(const Vector<T, N>& vec) {
	return vec * PIRAD<T>;
}

/// @brief Run function per component and return new vector
/// @tparam T vector data type
/// @tparam N vector dimensions
/// @param vec input vector
/// @param func function to run per component
/// @return a new vector with applied function
template<typename T, size_t N>
constexpr Vector<T, N> compFuncVector(const Vector<T, N>& vec, T (*func)(T)) {
	Vector<T, N> output{};

	for (size_t i = 0; i < N; i++) {
		output[i] = func(vec[i]);
	}

	return output;
}

template<typename T, size_t N>
constexpr Vector<T, N> sqrtVector(const Vector<T, N>& vec) {
	Vector<T, N> output{};

	for (size_t i = 0; i < N; i++) {
		output[i] = sqrtScalar(vec[i]);
	}

	return output;
}
template<typename T, size_t N>
constexpr Vector<T, N> absVector(const Vector<T, N>& vec) {
	Vector<T, N> output{};

	for (size_t i = 0; i < N; i++) {
		output[i] = absScalar(vec[i]);
	}

	return output;
}

template<typename T, size_t N>
constexpr Vector<T, N> sinVector(const Vector<T, N>& vec) {
	Vector<T, N> output{};

	for (size_t i = 0; i < N; i++) {
		output[i] = sinScalar(vec[i]);
	}

	return output;
}
template<typename T, size_t N>
constexpr Vector<T, N> cosVector(const Vector<T, N>& vec) {
	Vector<T, N> output{};

	for (size_t i = 0; i < N; i++) {
		output[i] = cosScalar(vec[i]);
	}

	return output;
}
template<typename T, size_t N>
constexpr Vector<T, N> tanVector(const Vector<T, N>& vec) {
	Vector<T, N> output{};

	for (size_t i = 0; i < N; i++) {
		output[i] = tanScalar(vec[i]);
	}

	return output;
}

// Functions
// Vector
// Dot
template<typename T>
T dot(vector2D<T> a, vector2D<T> b) {
	return a.x * b.x + a.y * b.y;
}
template<typename T>
T dot(vector3D<T> a, vector3D<T> b) {
	return a.x * b.x + a.y * b.y + a.z * b.z;
}
template<typename T>
T dot(vector4D<T> a, vector4D<T> b) {
	return a.x * b.x + a.y * b.y + a.z * b.z + a.w * b.w;
}
template<typename T, size_t N>
constexpr T dot(const Vector<T, N>& a, const Vector<T, N>& b) {
	T output{};

	for (size_t i = 0; i < N; i++) {
		output += a[i] * b[i];
	}

	return output;
}
// Cross
template<typename T>
T cross(vector2D<T> a, vector2D<T> b) {
	return a.x * b.y - a.y * b.x;
}
template<typename T>
vector3D<T> cross(vector3D<T> a, vector3D<T> b) {
	return {
		a.y * b.z - a.z * b.y,
		a.z * b.x - a.x * b.z,
		a.x * b.y - a.y * b.x
	};
}
template<typename T>
constexpr T cross(const Vector<T, 2>& a, const Vector<T, 2>& b) {
	return a[axes::x] * b[axes::y] - a[axes::y] * b[axes::x];
}
template<typename T>
constexpr Vector<T, 3> cross(const Vector<T, 3>& a, const Vector<T, 3>& b) {
	return {
		a[axes::y] * b[axes::z] - a[axes::z] * b[axes::y],
		a[axes::z] * b[axes::x] - a[axes::x] * b[axes::z],
		a[axes::x] * b[axes::y] - a[axes::y] * b[axes::x],
	};
}
// Magnitude
// Squared
template<typename T>
T magnitudeSquared(vector2D<T> vec) {
	return vec.x * vec.x + vec.y * vec.y;
}
template<typename T>
T magnitudeSquared(vector3D<T> vec) {
	return vec.x * vec.x + vec.y * vec.y + vec.z * vec.z;
}
template<typename T>
T magnitudeSquared(vector4D<T> vec) {
	return vec.x * vec.x + vec.y * vec.y + vec.z * vec.z + vec.w * vec.w;
}
template<typename T, size_t N>
constexpr T magnitudeSquared(const Vector<T, N>& vec) {
	T result{};

	for (size_t i = 0; i < N; i++) {
		result += vec[i] * vec[i];
	}

	return result;
}
// Normal
template<typename T>
T magnitude(vector2D<T> vec) {
	return std::sqrt(magnitudeSquared(vec));
}
template<typename T>
T magnitude(vector3D<T> vec) {
	return std::sqrt(magnitudeSquared(vec));
}
template<typename T>
T magnitude(vector4D<T> vec) {
	return std::sqrt(magnitudeSquared(vec));
}
template<typename T, size_t N>
constexpr T magnitude(const Vector<T, N>& vec) {
	return sqrtScalar(magnitudeSquared(vec));
}
// Normalize
template<typename T>
vector2D<T> normalize(vector2D<T> vec) {
	return vec / magnitude(vec);
}
template<typename T>
vector3D<T> normalize(vector3D<T> vec) {
	return vec / magnitude(vec);
}template<typename T>
vector4D<T> normalize(vector4D<T> vec) {
	return vec / magnitude(vec);
}
template<typename T, size_t N>
constexpr Vector<T, N> normalize(const Vector<T, N>& vec) {
	return vec / magnitude(vec);
}

// Matrix functions
// Get identity Matrices
template<typename T>
matrix2x2<T> identity2x2() {
	return {
		{ static_cast<T>(1.0), static_cast<T>(0.0) },
		{ static_cast<T>(0.0), static_cast<T>(1.0) },
	};
}
template<typename T>
matrix3x3<T> identity3x3() {
	return {
		{ static_cast<T>(1.0), static_cast<T>(0.0), static_cast<T>(0.0) },
		{ static_cast<T>(0.0), static_cast<T>(1.0), static_cast<T>(0.0) },
		{ static_cast<T>(0.0), static_cast<T>(0.0), static_cast<T>(1.0) },
	};
}
template<typename T>
matrix4x4<T> identity4x4() {
	return {
		{ static_cast<T>(1.0), static_cast<T>(0.0), static_cast<T>(0.0), static_cast<T>(0.0) },
		{ static_cast<T>(0.0), static_cast<T>(1.0), static_cast<T>(0.0), static_cast<T>(0.0) },
		{ static_cast<T>(0.0), static_cast<T>(0.0), static_cast<T>(1.0), static_cast<T>(0.0) },
		{ static_cast<T>(0.0), static_cast<T>(0.0), static_cast<T>(0.0), static_cast<T>(1.0) },
	};
}

template<typename T, size_t N>
constexpr Matrix<T, N, N> identityMatrix() {
	Matrix<T, N, N> output{};

	for (size_t i = 0; i < N; i++) {
		output(i, i) = static_cast<T>(1);
	}

	return output;
}
// Transpose
template<typename T>
matrix2x2<T> transpose(matrix2x2<T> mat) {
	return {
		{ static_cast<T>(mat.x.x), static_cast<T>(mat.y.x) },
		{ static_cast<T>(mat.x.y), static_cast<T>(mat.y.y) },
	};
}
template<typename T>
matrix3x3<T> transpose(matrix3x3<T> mat) {
	return {
		{ static_cast<T>(mat.x.x), static_cast<T>(mat.y.x), static_cast<T>(mat.z.x) },
		{ static_cast<T>(mat.x.y), static_cast<T>(mat.y.y), static_cast<T>(mat.z.y) },
		{ static_cast<T>(mat.x.z), static_cast<T>(mat.y.z), static_cast<T>(mat.z.z) },
	};
}
template<typename T>
matrix4x4<T> transpose(matrix4x4<T> mat) {
	return {
		{ static_cast<T>(mat.x.x), static_cast<T>(mat.y.x), static_cast<T>(mat.z.x), static_cast<T>(mat.w.x) },
		{ static_cast<T>(mat.x.y), static_cast<T>(mat.y.y), static_cast<T>(mat.z.y), static_cast<T>(mat.w.y) },
		{ static_cast<T>(mat.x.z), static_cast<T>(mat.y.z), static_cast<T>(mat.z.z), static_cast<T>(mat.w.z) },
		{ static_cast<T>(mat.x.w), static_cast<T>(mat.y.w), static_cast<T>(mat.z.w), static_cast<T>(mat.w.w) },
	};
}
template<typename T, size_t NRow, size_t NCol>
constexpr Matrix<T, NCol, NRow> transpose(const Matrix<T, NRow, NCol>& mat) {
	Matrix<T, NCol, NRow> output{};

	for (size_t i = 0; i < NRow; i++) {
		for (size_t j = 0; j < NCol; j++) {
			output(j, i) = mat(i, j);
		}
	}

	return output;
}
// Position Matrices
template<typename T>
matrix3x3<T> position2D(vector2D<T> pos) {
	return {
		{ static_cast<T>(1.0), static_cast<T>(0.0), pos.x },
		{ static_cast<T>(0.0), static_cast<T>(1.0), pos.y },
		{ static_cast<T>(0.0), static_cast<T>(0.0), static_cast<T>(1.0) },
	};
}
template<typename T>
matrix4x4<T> position3D(vector3D<T> pos) {
	return {
		{ static_cast<T>(1.0), static_cast<T>(0.0), static_cast<T>(0.0), pos.x },
		{ static_cast<T>(0.0), static_cast<T>(1.0), static_cast<T>(0.0), pos.y },
		{ static_cast<T>(0.0), static_cast<T>(0.0), static_cast<T>(1.0), pos.z },
		{ static_cast<T>(0.0), static_cast<T>(0.0), static_cast<T>(0.0), static_cast<T>(1.0) },
	};
}
template<typename T, size_t N>
constexpr Matrix<T, N + 1, N + 1> positionMatrix(const Vector<T, N>& pos) {
	Matrix<T, N + 1, N + 1> output = identityMatrix<T, N + 1>();

	for (size_t i = 0; i < N; i++) {
		output(i, N) = pos[i];
	}

	return output;
}

template<typename T>
matrix2x2<T> scale2D(vector2D<T> scale) {
	return {
		{ scale.x, 0 },
		{ 0, scale.y },
	};
}

template<typename T>
matrix3x3<T> scale3D(vector3D<T> scale) {
	return {
		{ scale.x, 0, 0 },
		{ 0, scale.y, 0 },
		{ 0, 0, scale.z },
	};
}
template<typename T, size_t N>
constexpr Matrix<T, N, N> scaleMatrix(const Vector<T, N>& scale) {
	Matrix<T, N, N> output{};

	for (size_t i = 0; i < N; i++) {
		output(i, i) = scale[i];
	}

	return output;
}

// Projection Matrices
// ratio = height / width
template<typename T>
matrix4x4<T> ortho(T left, T right, T bottom, T top, T near, T far, T ratio) {
	return {
		{ static_cast<T>(2.0) / (right - left) * ratio, static_cast<T>(0),                    static_cast<T>(0),                   -( (right + left) / (right - left) ) },
		{ static_cast<T>(0),                            static_cast<T>(2.0) / (top - bottom), static_cast<T>(0),                   -( (top + bottom) / (top - bottom) ) },
		{ static_cast<T>(0),                            static_cast<T>(0),                    static_cast<T>(-2.0) / (far - near), -( (far + near) / (far - near) ) },
		{ static_cast<T>(0),                            static_cast<T>(0),                    static_cast<T>(0),                   static_cast<T>(1.0) },
	};
}
template<typename T>
matrix4x4<T> ortho(T width, T height, T near, T far) {
	return ortho(-width, width, -height, height, near, far, height / width);
}
template<typename T>
constexpr Matrix<T, 4, 4> orthographicProjection(T left, T right, T bottom, T top, T near, T far) {
	T width = right - left;
	T height = top - bottom;
	T ratio = height / width;
	return {
		{ static_cast<T>(2) / width * ratio, static_cast<T>(0),          static_cast<T>(0),                 -((right + left) / width) },
		{ static_cast<T>(0),                 static_cast<T>(2) / height, static_cast<T>(0),                 -((top + bottom) / height) },
		{ static_cast<T>(0),                 static_cast<T>(0),          static_cast<T>(-2) / (far - near), -((far + near) / (far - near)) },
		{ static_cast<T>(0),                 static_cast<T>(0),          static_cast<T>(0),                 static_cast<T>(1) },
	};
}
template<typename T>
constexpr Matrix<T, 4, 4> orthographicProjection(T width, T height, T near, T far) {
	return {
		{ static_cast<T>(2) / width * (height / width), static_cast<T>(0),          static_cast<T>(0),                 static_cast<T>(0) },
		{ static_cast<T>(0),                            static_cast<T>(2) / height, static_cast<T>(0),                 static_cast<T>(0) },
		{ static_cast<T>(0),                            static_cast<T>(0),          static_cast<T>(-2) / (far - near), -((far + near) / (far - near)) },
		{ static_cast<T>(0),                            static_cast<T>(0),          static_cast<T>(0),                 static_cast<T>(1) },
	};
}
// ratio = height / width
template<typename T>
matrix4x4<T> perspective(T fov, T near, T far, T ratio) {
	T y = static_cast<T>(1) / std::tan( degrees2radians( fov / static_cast<T>(2) ) );
	return {
		{ y * ratio,  0,  0,                                0 },
		{ 0,         -y,  0,                                0 },
		{ 0,          0, -( (far + near) / (far - near) ), -( (2 * near * far) / (far - near) ) },
		{ 0,          0, -1,                                0 },
	};
}
template<typename T>
constexpr Matrix<T, 4, 4> perspectiveProjection(T fov, T near, T far, T ratio) {
	T s = static_cast<T>(1) / tanScalar(degrees2radians(fov / static_cast<T>(2)));
	return {
		{ s * ratio,         static_cast<T>(0), static_cast<T>(0),           static_cast<T>(0) },
		{ static_cast<T>(0), -s,                static_cast<T>(0),           static_cast<T>(0) },
		{ static_cast<T>(0), static_cast<T>(0), (far + near) / (near - far), (2 * far * near) / (near - far) },
		{ static_cast<T>(0), static_cast<T>(0), static_cast<T>(-1),          static_cast<T>(0) },
	};
}

// Rotation Matrices
template<typename T>
matrix2x2<T> rotation2D(T degrees) {
	T rad = degrees2radians(degrees);
	T sinV = std::sin(rad);
	T cosV = std::cos(rad);
	return {
		{ cosV, sinV },
		{ -sinV, cosV },
	};
}
template<typename T>
constexpr Matrix<T, 2, 2> rotation2DMatrix(T degrees) {
	T rad = degrees2radians(degrees);
	T sin = sinScalar(rad);
	T cos = cosScalar(rad);
	return {
		{  cos, sin },
		{ -sin, cos },
	};
}
// Axis order: YXZ
template<typename T>
matrix3x3<T> rotation3D(vector3D<T> degrees) {
	vector3D<T> rad{ degrees2radians(degrees.x), degrees2radians(degrees.y), degrees2radians(degrees.z) };
	vector3D<T> sinV{ std::sin(rad.x), std::sin(rad.y), std::sin(rad.z) };
	vector3D<T> cosV{ std::cos(rad.x), std::cos(rad.y), std::cos(rad.z) };

	matrix3x3<T> rotX{
		{ 1,  0,      0      },
		{ 0,  cosV.x, sinV.x },
		{ 0, -sinV.x, cosV.x },
	};
	matrix3x3<T> rotY{
		{  cosV.y, 0, sinV.y },
		{  0,      1, 0      },
		{ -sinV.y, 0, cosV.y },
	};
	matrix3x3<T> rotZ{
		{  cosV.z, sinV.z, 0 },
		{ -sinV.z, cosV.z, 0 },
		{  0,      0,      1 },
	};

	return rotZ * (rotX * rotY);
}
enum class Rotation3DAxisOrder {
	XYZ,
	XZY,
	YXZ,
	YZX,
	ZXY,
	ZYX,
};
template<typename T>
constexpr Matrix<T, 3, 3> eulerRotationMatrix(const Vector<T, 3>& degrees, const Rotation3DAxisOrder axisOrder = Rotation3DAxisOrder::YXZ) {
	Vector<T, 3> rad{ degreesToRadians(degrees) };
	Vector<T, 3> sin{ sinVector(rad) };
	Vector<T, 3> cos{ cosVector(rad) };

	Matrix<T, 3, 3> rotX {
		{ 1,  0,      0      },
		{ 0,  cos[0], sin[0] },
		{ 0, -sin[0], cos[0] },
	};
	Matrix<T, 3, 3> rotY {
		{  cos[1], 0, sin[1] },
		{  0,      1, 0      },
		{ -sin[1], 0, cos[1] },
	};
	Matrix<T, 3, 3> rotZ {
		{  cos[2], sin[2], 0 },
		{ -sin[2], cos[2], 0 },
		{  0,      0,      1 },
	};

	switch (axisOrder) {
	case Rotation3DAxisOrder::XYZ:
		return rotX * rotY * rotZ;
	case Rotation3DAxisOrder::XZY:
		return rotX * rotZ * rotY;
	case Rotation3DAxisOrder::YXZ:
		return rotY * rotX * rotZ;
	case Rotation3DAxisOrder::YZX:
		return rotY * rotZ * rotX;
	case Rotation3DAxisOrder::ZXY:
		return rotZ * rotX * rotY;
	case Rotation3DAxisOrder::ZYX:
		return rotZ * rotY * rotX;
	default:
		return rotX * rotY * rotZ;
	}
}

// Look at matrix
template<typename T>
matrix4x4<T> lookAt(vector3D<T> eye, vector3D<T> at, vector3D<T> up) {
	vector3D<T> forward = normalize(at - eye);
	vector3D<T> right = normalize(cross(forward, up));
	up = cross(forward, right);
	return {
		{ right.x,   right.y,   right.z,   dot(right, -eye) },
		{ up.x,      up.y,      up.z,      dot(up, -eye) },
		{ forward.x, forward.y, forward.z, dot(forward, -eye) },
		{ 0,         0,         0,         1}
	};
}
template<typename T>
constexpr Matrix<T, 4, 4> viewMatrix(const Vector<T, 3>& eye, const Vector<T, 3>& at, const Vector<T, 3>& up) {
	Vector<T, 3> forward = normalize(at - eye);
	Vector<T, 3> right = normalize(cross(forward, up));
	return {
		{ right.x,           right.y,           right.z,           dot(right, -eye) },
		{ up.x,              up.y,              up.z,              dot(up, -eye) },
		{ forward.x,         forward.y,         forward.z,         dot(forward, -eye) },
		{ static_cast<T>(0), static_cast<T>(0), static_cast<T>(0), static_cast<T>(1) },
	};
}

// Quaternions
template<typename T>
T magnitudeSquared(quaternion_t<T> quat) {
	return (quat.x * quat.x + quat.y * quat.y + quat.z * quat.z + quat.w * quat.w);
}
template<typename T>
T magnitude(quaternion_t<T> quat) {
	return std::sqrt(magnitudeSquared(quat));
}
template<typename T>
quaternion_t<T> normalize(quaternion_t<T> quat) {
	return quat / magnitude(quat);
}

template<typename T>
quaternion_t<T> inverse(quaternion_t<T> quat) {
	return {quat.w, -quat.x, -quat.y, -quat.z};
}

template<typename T>
matrix3x3<T> toMatrix(quaternion_t<T> quat) {
	return {
		{ 2 * (quat.w * quat.w + quat.x * quat.x) - 1, 2 * (quat.x * quat.y - quat.w * quat.z),     2 * (quat.x * quat.z + quat.w * quat.y) },
		{ 2 * (quat.x * quat.y + quat.w * quat.z),     2 * (quat.w * quat.w + quat.y * quat.y) - 1, 2 * (quat.y * quat.z - quat.w * quat.x) },
		{ 2 * (quat.x * quat.z - quat.w * quat.y),     2 * (quat.y * quat.z + quat.w * quat.x),     2 * (quat.w * quat.w + quat.z * quat.z) - 1 },
	};
}


// Equal
template<typename T>
constexpr bool equal(T a, T b, T epsilon = EPSILON<T>) noexcept {
	return absScalar(a - b) <= epsilon;
}
template<typename T>
bool equal(vector2D<T> a, vector2D<T> b, T epsilon = 0.0001) {
	return (
		std::abs(a.x - b.x) < epsilon &&
		std::abs(a.y - b.y) < epsilon
		);
}
template<typename T>
bool equal(vector3D<T> a, vector3D<T> b, T epsilon = 0.0001) {
	return (
		std::abs(a.x - b.x) < epsilon &&
		std::abs(a.y - b.y) < epsilon &&
		std::abs(a.z - b.z) < epsilon
		);
}
template<typename T>
bool equal(vector4D<T> a, vector4D<T> b, T epsilon = 0.0001) {
	return (
		std::abs(a.x - b.x) < epsilon &&
		std::abs(a.y - b.y) < epsilon &&
		std::abs(a.z - b.z) < epsilon &&
		std::abs(a.w - b.w) < epsilon
		);
}
template<typename T, size_t N>
constexpr bool equal(const Vector<T, N>& a, const Vector<T, N>& b, T epsilon = EPSILON<T>) noexcept {
	for (size_t i = 0; i < N; i++) {
		if (absScalar(a[i] - b[i]) > epsilon) {
			return false;
		}
	}

	return true;
}
template<typename T>
bool equal(vector2D<T> a, T b, T epsilon = 0.0001) {
	return equal(a, { b, b }, epsilon);
}
template<typename T>
bool equal(vector3D<T> a, T b, T epsilon = 0.0001) {
	return equal(a, { b, b, b }, epsilon);
}
template<typename T>
bool equal(vector4D<T> a, T b, T epsilon = 0.0001) {
	return equal(a, { b, b, b, b }, epsilon);
}
template<typename T, size_t N>
constexpr bool equal(const Vector<T, N>& a, T b, T epsilon = EPSILON<T>) noexcept {
	for (size_t i = 0; i < N; i++) {
		if (absScalar(a[i] - b) > epsilon) {
			return false;
		}
	}

	return true;
}

// Operator overloads
// Component-vise operations
// vector2D
template<typename T>
vector2D<T> operator+(vector2D<T> a, vector2D<T> b) {
	return { a.x + b.x, a.y + b.y };
}
template<typename T>
vector2D<T> operator-(vector2D<T> a, vector2D<T> b) {
	return { a.x - b.x, a.y - b.y };
}
template<typename T>
vector2D<T> operator*(vector2D<T> a, vector2D<T> b) {
	return { a.x * b.x, a.y * b.y };
}
template<typename T>
vector2D<T> operator/(vector2D<T> a, vector2D<T> b) {
	return { 
		b.x != 0 ? a.x / b.x : 0,
		b.y != 0 ? a.y / b.y : 0
	};
}
// vector3D
template<typename T>
vector3D<T> operator+(vector3D<T> a, vector3D<T> b) {
	return { a.x + b.x, a.y + b.y, a.z + b.z };
}
template<typename T>
vector3D<T> operator-(vector3D<T> a, vector3D<T> b) {
	return { a.x - b.x, a.y - b.y, a.z - b.z };
}
template<typename T>
vector3D<T> operator*(vector3D<T> a, vector3D<T> b) {
	return { a.x * b.x, a.y * b.y, a.z * b.z };
}
template<typename T>
vector3D<T> operator/(vector3D<T> a, vector3D<T> b) {
	return {
		b.x != 0 ? a.x / b.x : 0,
		b.y != 0 ? a.y / b.y : 0,
		b.z != 0 ? a.z / b.z : 0
	};
}
// vector4D
template<typename T>
vector4D<T> operator+(vector4D<T> a, vector4D<T> b) {
	return { a.x + b.x, a.y + b.y, a.z + b.z, a.w + b.w };
}
template<typename T>
vector4D<T> operator-(vector4D<T> a, vector4D<T> b) {
	return { a.x - b.x, a.y - b.y, a.z - b.z, a.w - b.w };
}
template<typename T>
vector4D<T> operator*(vector4D<T> a, vector4D<T> b) {
	return { a.x * b.x, a.y * b.y, a.z * b.z, a.w * b.w };
}
template<typename T>
vector4D<T> operator/(vector4D<T> a, vector4D<T> b) {
	return {
		b.x != 0 ? a.x / b.x : 0,
		b.y != 0 ? a.y / b.y : 0,
		b.z != 0 ? a.z / b.z : 0,
		b.w != 0 ? a.w / b.w : 0,
	};
}

template<typename T, size_t N>
constexpr Vector<T, N> operator+(const Vector<T, N>& a, const Vector<T, N>& b) {
	Vector<T, N> output{};

	for (size_t i = 0; i < N; i++) {
		output[i] = a[i] + b[i];
	}

	return output;
}
template<typename T, size_t N>
constexpr Vector<T, N> operator-(const Vector<T, N>& a, const Vector<T, N>& b) {
	Vector<T, N> output{};

	for (size_t i = 0; i < N; i++) {
		output[i] = a[i] - b[i];
	}

	return output;
}
template<typename T, size_t N>
constexpr Vector<T, N> operator*(const Vector<T, N>& a, const Vector<T, N>& b) {
	Vector<T, N> output{};

	for (size_t i = 0; i < N; i++) {
		output[i] = a[i] * b[i];
	}

	return output;
}
template<typename T, size_t N>
constexpr Vector<T, N> operator/(const Vector<T, N>& a, const Vector<T, N>& b) {
	Vector<T, N> output{};

	for (size_t i = 0; i < N; i++) {
		output[i] = a[i] / b[i];
	}

	return output;
}
// Scalar operations
// vector2D
template<typename T>
vector2D<T> operator+(vector2D<T> a, T s) {
	return { a.x + s, a.y + s };
}
template<typename T>
vector2D<T> operator-(vector2D<T> a, T s) {
	return { a.x - s, a.y - s };
}
template<typename T>
vector2D<T> operator*(vector2D<T> a, T s) {
	return { a.x * s, a.y * s };
}
template<typename T>
vector2D<T> operator/(vector2D<T> a, T s) {
	if (s == 0) {
		return { 0, 0 };
	}
	return { a.x / s, a.y / s };
}

template<typename T, size_t N>
constexpr Vector<T, N> operator+(const Vector<T, N>& a, T b) {
	Vector<T, N> output{};

	for (size_t i = 0; i < N; i++) {
		output[i] = a[i] + b;
	}

	return output;
}
template<typename T, size_t N>
constexpr Vector<T, N> operator-(const Vector<T, N>& a, T b) {
	Vector<T, N> output{};

	for (size_t i = 0; i < N; i++) {
		output[i] = a[i] - b;
	}

	return output;
}
template<typename T, size_t N>
constexpr Vector<T, N> operator*(const Vector<T, N>& a, T b) {
	Vector<T, N> output{};

	for (size_t i = 0; i < N; i++) {
		output[i] = a[i] * b;
	}

	return output;
}
template<typename T, size_t N>
constexpr Vector<T, N> operator/(const Vector<T, N>& a, T b) {
	Vector<T, N> output{};

	for (size_t i = 0; i < N; i++) {
		output[i] = a[i] / b;
	}

	return output;
}
// vector3D
template<typename T>
vector3D<T> operator+(vector3D<T> a, T s) {
	return { a.x + s, a.y + s, a.z + s };
}
template<typename T>
vector3D<T> operator-(vector3D<T> a, T s) {
	return { a.x - s, a.y - s, a.z - s};
}
template<typename T>
vector3D<T> operator*(vector3D<T> a, T s) {
	return { a.x * s, a.y * s, a.z * s };
}
template<typename T>
vector3D<T> operator/(vector3D<T> a, T s) {
	if (s == 0) {
		return { 0, 0 };
	}
	return { a.x / s, a.y / s, a.z / s };
}
// vector4D
template<typename T>
vector4D<T> operator+(vector4D<T> a, T s) {
	return { a.x + s, a.y + s, a.z + s, a.w + s };
}
template<typename T>
vector4D<T> operator-(vector4D<T> a, T s) {
	return { a.x - s, a.y - s, a.z - s, a.w - s };
}
template<typename T>
vector4D<T> operator*(vector4D<T> a, T s) {
	return { a.x * s, a.y * s, a.z * s, a.w * s };
}
template<typename T>
vector4D<T> operator/(vector4D<T> a, T s) {
	if (s == 0) {
		return { 0, 0 };
	}
	return { a.x / s, a.y / s, a.z / s, a.w / s };
}
// Modulo
template<typename T>
vector2D<T> operator%(vector2D<T> a, T s) {
	if (s == 0) {
		return { 0,  0 };
	}
	return { std::fmod(a.x, s), std::fmod(a.y, s) };
}
template<typename T>
vector3D<T> operator%(vector3D<T> a, T s) {
	if (s == 0) {
		return { 0,  0 };
	}
	return { std::fmod(a.x, s), std::fmod(a.y, s), std::fmod(a.z, s) };
}
template<typename T>
vector4D<T> operator%(vector4D<T> a, T s) {
	if (s == 0) {
		return { 0,  0 };
	}
	return { std::fmod(a.x, s), std::fmod(a.y, s), std::fmod(a.z, s), std::fmod(a.w, s) };
}

template<typename T, size_t N>
constexpr Vector<T, N> operator%(const Vector<T, N>& a, T b) {
	Vector<T, N> output{};

	for (size_t i = 0; i < N; i++) {
		output[i] = fmodScalar(a[i], b);
	}

	return output;
}
template<size_t N>
constexpr Vector<int, N> operator%(const Vector<int, N>& a, int b) {
	Vector<int, N> output{};

	for (size_t i = 0; i < N; i++) {
		output[i] = a[i] % b;
	}

	return output;
}
// With vectors
template<typename T>
vector2D<T> operator%(vector2D<T> a, vector2D<T> b) {
	return {
		b.x != 0 ? std::fmod(a.x, b.x) : 0,
		b.y != 0 ? std::fmod(a.y, b.y) : 0
	};
}
template<typename T>
vector3D<T> operator%(vector3D<T> a, vector3D<T> b) {
	return {
		b.x != 0 ? std::fmod(a.x, b.x) : 0,
		b.y != 0 ? std::fmod(a.y, b.y) : 0,
		b.z != 0 ? std::fmod(a.z, b.z) : 0
	};
}
template<typename T>
vector4D<T> operator%(vector4D<T> a, vector4D<T> b) {
	return {
		b.x != 0 ? std::fmod(a.x, b.x) : 0,
		b.y != 0 ? std::fmod(a.y, b.y) : 0,
		b.z != 0 ? std::fmod(a.z, b.z) : 0,
		b.w != 0 ? std::fmod(a.w, b.w) : 0
	};
}
template<typename T, size_t N>
constexpr Vector<T, N> operator%(const Vector<T, N>& a, const Vector<T, N>& b) {
	Vector<T, N> output{};

	for (size_t i = 0; i < N; i++) {
		output[i] = fmodScalar(a[i], b[i]);
	}

	return output;
}
template<size_t N>
constexpr Vector<int, N> operator%(const Vector<int, N>& a, const Vector<int, N>& b) {
	Vector<int, N> output{};

	for (size_t i = 0; i < N; i++) {
		output[i] = a[i] % b[i];
	}

	return output;
}
// Compound assign operations
// vector2D
template<typename T>
vector2D<T>& operator+=(vector2D<T>& a, vector2D<T> b) {
	return a = a + b;
}
template<typename T>
vector2D<T>& operator-=(vector2D<T>& a, vector2D<T> b) {
	return a = a - b;
}
template<typename T>
vector2D<T>& operator*=(vector2D<T>& a, vector2D<T> b) {
	return a = a * b;
}
template<typename T>
vector2D<T>& operator/=(vector2D<T>& a, vector2D<T> b) {
	return a = a / b;
}
// vector3D
template<typename T>
vector3D<T>& operator+=(vector3D<T>& a, vector3D<T> b) {
	return a = a + b;
}
template<typename T>
vector3D<T>& operator-=(vector3D<T>& a, vector3D<T> b) {
	return a = a - b;
}
template<typename T>
vector3D<T>& operator*=(vector3D<T>& a, vector3D<T> b) {
	return a = a * b;
}
template<typename T>
vector3D<T>& operator/=(vector3D<T>& a, vector3D<T> b) {
	return a = a / b;
}

template<typename T, size_t N>
constexpr Vector<T, N>& operator+=(Vector<T, N>& a, const Vector<T, N>& b) {
	for (size_t i = 0; i < N; i++) {
		a[i] += b[i];
	}

	return a;
}
template<typename T, size_t N>
constexpr Vector<T, N>& operator-=(Vector<T, N>& a, const Vector<T, N>& b) {
	for (size_t i = 0; i < N; i++) {
		a[i] -= b[i];
	}

	return a;
}
template<typename T, size_t N>
constexpr Vector<T, N>& operator*=(Vector<T, N>& a, const Vector<T, N>& b) {
	for (size_t i = 0; i < N; i++) {
		a[i] *= b[i];
	}

	return a;
}
template<typename T, size_t N>
constexpr Vector<T, N>& operator/=(Vector<T, N>& a, const Vector<T, N>& b) {
	for (size_t i = 0; i < N; i++) {
		a[i] /= b[i];
	}

	return a;
}
// vector4D
template<typename T>
vector4D<T>& operator+=(vector4D<T>& a, vector4D<T> b) {
	return a = a + b;
}
template<typename T>
vector4D<T>& operator-=(vector4D<T>& a, vector4D<T> b) {
	return a = a - b;
}
template<typename T>
vector4D<T>& operator*=(vector4D<T>& a, vector4D<T> b) {
	return a = a * b;
}
template<typename T>
vector4D<T>& operator/=(vector4D<T>& a, vector4D<T> b) {
	return a = a / b;
}
// Compound assign operations with scalar
// vector2D
template<typename T, size_t N>
constexpr Vector<T, N>& operator+=(Vector<T, N>& a, T b) {
	for (size_t i = 0; i < N; i++) {
		a[i] += b;
	}

	return a;
}
template<typename T, size_t N>
constexpr Vector<T, N>& operator-=(Vector<T, N>& a, T b) {
	for (size_t i = 0; i < N; i++) {
		a[i] -= b;
	}

	return a;
}
template<typename T, size_t N>
constexpr Vector<T, N>& operator*=(Vector<T, N>& a, T b) {
	for (size_t i = 0; i < N; i++) {
		a[i] *= b;
	}

	return a;
}
template<typename T, size_t N>
constexpr Vector<T, N>& operator/=(Vector<T, N>& a, T b) {
	for (size_t i = 0; i < N; i++) {
		a[i] /= b;
	}

	return a;
}

template<typename T>
vector2D<T>& operator+=(vector2D<T>& a, T s) {
	return a = a + s;
}
template<typename T>
vector2D<T>& operator-=(vector2D<T>& a, T s) {
	return a = a - s;
}
template<typename T>
vector2D<T>& operator*=(vector2D<T>& a, T s) {
	return a = a * s;
}
template<typename T>
vector2D<T>& operator/=(vector2D<T>& a, T s) {
	return a = a / s;
}
// vector3D
template<typename T>
vector3D<T>& operator+=(vector3D<T>& a, T s) {
	return a = a + s;
}
template<typename T>
vector3D<T>& operator-=(vector3D<T>& a, T s) {
	return a = a - s;
}
template<typename T>
vector3D<T>& operator*=(vector3D<T>& a, T s) {
	return a = a * s;
}
template<typename T>
vector3D<T>& operator/=(vector3D<T>& a, T s) {
	return a = a / s;
}
// vector4D
template<typename T>
vector4D<T>& operator+=(vector4D<T>& a, T s) {
	return a = a + s;
}
template<typename T>
vector4D<T>& operator-=(vector4D<T>& a, T s) {
	return a = a - s;
}
template<typename T>
vector4D<T>& operator*=(vector4D<T>& a, T s) {
	return a = a * s;
}
template<typename T>
vector4D<T>& operator/=(vector4D<T>& a, T s) {
	return a = a / s;
}
// Modulo
template<typename T>
vector2D<T>& operator%=(vector2D<T>& a, T s) {
	return a = a % s;
}
template<typename T>
vector3D<T>& operator%=(vector3D<T>& a, T s) {
	return a = a % s;
}
template<typename T>
vector4D<T>& operator%=(vector4D<T>& a, T s) {
	return a = a % s;
}

// Unary operations
// Negate
template<typename T>
vector2D<T> operator-(vector2D<T> a) {
	return { -a.x, -a.y };
}
template<typename T>
vector3D<T> operator-(vector3D<T> a) {
	return { -a.x, -a.y, -a.z };
}
template<typename T>
vector4D<T> operator-(vector4D<T> a) {
	return { -a.x, -a.y, -a.z, -a.w };
}

template<typename T, size_t N>
constexpr Vector<T, N> operator-(const Vector<T, N>& vec) {
	Vector<T, N> output{};

	for (size_t i = 0; i < N; i++) {
		output[i] = -vec[i];
	}

	return output;
}
// Increment
template<typename T>
vector2D<T>& operator++(vector2D<T>& a) {
	++a.x;
	++a.y;
	return a;
}
template<typename T>
vector3D<T>& operator++(vector3D<T>& a) {
	++a.x;
	++a.y;
	++a.z;
	return a;
}
template<typename T>
vector4D<T>& operator++(vector4D<T>& a) {
	++a.x;
	++a.y;
	++a.z;
	++a.w;
	return a;
}
template<typename T, size_t N>
constexpr Vector<T, N> operator++(Vector<T, N>& vec) {
	for (size_t i = 0; i < N; i++) {
		++vec[i];
	}

	return vec;
}
// Decrement
template<typename T>
vector2D<T>& operator--(vector2D<T>& a) {
	--a.x;
	--a.y;
	return a;
}
template<typename T>
vector3D<T>& operator--(vector3D<T>& a) {
	--a.x;
	--a.y;
	--a.z;
	return a;
}
template<typename T>
vector4D<T>& operator--(vector4D<T>& a) {
	--a.x;
	--a.y;
	--a.z;
	--a.w;
	return a;
}
template<typename T, size_t N>
constexpr Vector<T, N> operator--(Vector<T, N>& vec) {
	for (size_t i = 0; i < N; i++) {
		--vec[i];
	}

	return vec;
}

// Matrix multiplications
// With vector
template<typename T>
vector2D<T> operator*(matrix2x2<T> mat, vector2D<T> vec) {
	return {
		dot(mat.x, vec),
		dot(mat.y, vec),
	};
}
template<typename T>
vector3D<T> operator*(matrix3x3<T> mat, vector3D<T> vec) {
	return {
		dot(mat.x, vec),
		dot(mat.y, vec),
		dot(mat.z, vec),
	};
}
template<typename T>
vector4D<T> operator*(matrix4x4<T> mat, vector4D<T> vec) {
	return {
		dot(mat.x, vec),
		dot(mat.y, vec),
		dot(mat.z, vec),
		dot(mat.w, vec),
	};
}
template<typename T, size_t NRow, size_t NCol>
constexpr Vector<T, NRow> operator*(const Matrix<T, NRow, NCol>& mat, const Vector<T, NCol>& vec) {
	Vector<T, NRow> output{};

	for (size_t i = 0; i < NRow; i++) {
		for (size_t j = 0; j < NCol; j++) {
			output[i] += vec[j] * mat(i, j);
		}
	}

	return output;
}
template<typename T, size_t NRow, size_t NCol>
constexpr Vector<T, NCol> operator*(const Vector<T, NRow>& vec, const Matrix<T, NRow, NCol>& mat) {
	Vector<T, NCol> output{};

	for (size_t j = 0; j < NCol; j++) {
		for (size_t i = 0; i < NRow; i++) {
			output[j] += vec[i] * mat(i, j);
		}
	}

	return output;
}
// With matrix
template<typename T>
matrix2x2<T> operator*(matrix2x2<T> a, matrix2x2<T> b) {
	return {
		{ dot(a.x, { b.x.x, b.y.x }), dot(a.x, { b.x.y, b.y.y }) },
		{ dot(a.y, { b.x.x, b.y.x }), dot(a.y, { b.x.y, b.y.y }) },
	};
}
template<typename T>
matrix3x3<T> operator*(matrix3x3<T> a, matrix3x3<T> b) {
	return {
		{ dot(a.x, { b.x.x, b.y.x, b.z.x }), dot(a.x, { b.x.y, b.y.y, b.z.y }), dot(a.x, { b.x.z, b.y.z, b.z.z }) },
		{ dot(a.y, { b.x.x, b.y.x, b.z.x }), dot(a.y, { b.x.y, b.y.y, b.z.y }), dot(a.y, { b.x.z, b.y.z, b.z.z }) },
		{ dot(a.z, { b.x.x, b.y.x, b.z.x }), dot(a.z, { b.x.y, b.y.y, b.z.y }), dot(a.z, { b.x.z, b.y.z, b.z.z }) },
	};
}
template<typename T>
matrix4x4<T> operator*(matrix4x4<T> a, matrix4x4<T> b) {
	return {
		{ dot(a.x, { b.x.x, b.y.x, b.z.x, b.w.x }), dot(a.x, { b.x.y, b.y.y, b.z.y, b.w.y }), dot(a.x, { b.x.z, b.y.z, b.z.z, b.w.z }), dot(a.x, { b.x.w, b.y.w, b.z.w, b.w.w }) },
		{ dot(a.y, { b.x.x, b.y.x, b.z.x, b.w.x }), dot(a.y, { b.x.y, b.y.y, b.z.y, b.w.y }), dot(a.y, { b.x.z, b.y.z, b.z.z, b.w.z }), dot(a.y, { b.x.w, b.y.w, b.z.w, b.w.w }) },
		{ dot(a.z, { b.x.x, b.y.x, b.z.x, b.w.x }), dot(a.z, { b.x.y, b.y.y, b.z.y, b.w.y }), dot(a.z, { b.x.z, b.y.z, b.z.z, b.w.z }), dot(a.z, { b.x.w, b.y.w, b.z.w, b.w.w }) },
		{ dot(a.w, { b.x.x, b.y.x, b.z.x, b.w.x }), dot(a.w, { b.x.y, b.y.y, b.z.y, b.w.y }), dot(a.w, { b.x.z, b.y.z, b.z.z, b.w.z }), dot(a.w, { b.x.w, b.y.w, b.z.w, b.w.w }) },
	};
}
template<typename T, size_t NRow, size_t NCol, size_t NRow1>
constexpr Matrix<T, NRow1, NCol> operator*(const Matrix<T, NRow, NCol>& a, const Matrix<T, NRow1, NRow>& b) {
	Matrix<T, NRow1, NCol> output{};

	for (size_t i = 0; i < NRow1; i++) {
		for (size_t j = 0; j < NCol; j++) {
			for (size_t k = 0; k < NRow; k++) {
				output(i, j) += a(k, j) * b(i, k);
			}
		}
	}

	return output;
}

// Quaternions
template<typename T>
quaternion_t<T> operator-(quaternion_t<T> quat) {
	return { -quat.w, -quat.x, -quat.y, -quat.z };
}


#ifndef LM2_NO_OUTPUT_FUNCTIONS

template<typename T>
std::ostream& operator<<(std::ostream& os, vector2D<T> vec) {
	return os << "X: " << vec.x << " Y: " << vec.y;
}

template<typename T>
std::ostream& operator<<(std::ostream& os, vector3D<T> vec) {
	return os << "X: " << vec.x << " Y: " << vec.y << " Z: " << vec.z;
}

template<typename T>
std::ostream& operator<<(std::ostream& os, vector4D<T> vec) {
	return os << "X: " << vec.x << " Y: " << vec.y << " Z: " << vec.z << " W: " << vec.w;
}

template<typename T, size_t N>
std::ostream& operator<<(std::ostream& os, Vector<T, N> vec) {
	for (size_t i = 0; i < N; i++) {
		os << vec[i];
		if (i != N - 1) {
			os << ", ";
		}
	}

	return os;
}


template<typename T>
std::ostream& operator<<(std::ostream& os, matrix2x2<T> mat) {
	return os << "X - " << mat.x << "\nY - " << mat.y;
}

template<typename T>
std::ostream& operator<<(std::ostream& os, matrix3x3<T> mat) {
	return os << "X - " << mat.x << "\nY - " << mat.y << "\nZ - " << mat.z;
}

template<typename T>
std::ostream& operator<<(std::ostream& os, matrix4x4<T> mat) {
	return os << "X - " << mat.x << "\nY - " << mat.y << "\nZ - " << mat.z << "\nW - " << mat.w;
}

template<typename T, size_t NRow, size_t NCol>
constexpr std::ostream& operator<<(std::ostream& os, const Matrix<T, NRow, NCol>& mat) {
	for (size_t i = 0; i < NRow; i++) {
		for (size_t j = 0; j < NCol; j++) {
			os << mat(i, j);
			if (j != NCol - 1) {
				os << ", ";
			}
		}
		if (i != NRow - 1) {
			os << "\n";
		}
	}

	return os;
}

#endif // #ifndef LM2_NO_OUTPUT_FUNCTIONS
} // namespace lm2
