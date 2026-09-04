/*
* A single header library for basic linear math
*/
#pragma once

#ifndef LM2_NO_OUTPUT_FUNCTIONS
#include <ostream>
#endif

#include <cmath>
#include <initializer_list>
#include <tuple>

namespace lm2 {

template<typename T> constexpr T PI = T(3.1415926535897932384626433832795);
template<typename T> constexpr T E = T(2.7182818284590452353602874713527);
template<typename T> constexpr T PIRAD = PI<T> / T(180);

namespace axes {
	enum {
		x = 0,
		y = 1,
		z = 2,
		w = 3,
	};
};

template<typename T> constexpr T EPSILON = T(0.0000001);

// Vector types
template<typename T, size_t N>
class Vector {
private:
	T data[N] {};

public:
	constexpr Vector() = default;
	constexpr Vector(const Vector<T, N>& rv) = default;
	constexpr Vector(Vector<T, N>& lv) = default;

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


// Matrix types
template<typename T, size_t NRow, size_t NCol>
class Matrix {
private:
	T data[NRow][NCol] {};

public:
	constexpr Matrix() = default;
	constexpr Matrix(const Matrix<T, NRow, NCol>& rv) = default;
	constexpr Matrix(Matrix<T, NRow, NCol>& lv) = default;

	constexpr Matrix(std::initializer_list<std::initializer_list<T>> l) : data{} {
		size_t s = NRow < l.size() ? NRow : l.size();
		for (size_t i = 0; i < s; i++) {
			size_t innerSize = (l.begin() + i)->size();
			size_t s1 = NCol < innerSize ? NCol : innerSize;
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

	void swapCol(size_t aCol, size_t bCol) {
		if (aCol == bCol) {
			return;
		}
		for (size_t row = 0; row < NRow; row++) {
			std::swap(data[row][aCol], data[row][bCol]);
		}
	}

	void swapRow(size_t aRow, size_t bRow) {
		if (aRow == bRow) {
			return;
		}
		for (size_t col = 0; col < NCol; col++) {
			std::swap(data[aRow][col], data[bRow][col]);
		}
	}
};


// Quaternion types
template<typename T>
struct quaternion_t {
	T w, x, y, z;
};

using quaternion = quaternion_t<float>;

// Permutation types
template<size_t N>
class Permutation1D {
private:
	size_t data[N] {};
	bool isPositive = true;

public:
	constexpr Permutation1D(const Permutation1D<N>& rv) = default;
	constexpr Permutation1D(Permutation1D<N>& lv) = default;

	constexpr Permutation1D() : data{} {
		for (size_t i = 0; i < N; i++) {
			data[i] = i;
		}
	}

	constexpr bool isPositive() const {
		return isPositive;
	}

	constexpr size_t size() const {
		return N;
	}

	constexpr const size_t& operator[](size_t index) const {
		return data[index];
	}

	constexpr void swap(size_t aIdx, size_t bIdx) {
		std::swap(data[aIdx], data[bIdx]);
		isPositive = !isPositive;
	}

	constexpr void move(size_t from, size_t to) {
		if (from == to) {
			return;
		}
		
		size_t tmpVal = data[from];
		if (from < to) {
			for (size_t i = from; i < to; i++) {
				data[i] = data[i + 1];
			}

			if ((to - from) % 2 == 0) {
				isPositive = !isPositive;
			}
		}
		else {
			for (size_t i = from; i > to; i--) {
				data[i] = data[i - 1];
			}

			if ((from - to) % 2 == 0) {
				isPositive = !isPositive;
			}
		}
		data[to] = tmpVal;
	}
};

// Scalar math functions
template<typename T>
constexpr T degreesToRadians(T degrees) {
	return degrees * PIRAD<T>;
}
template<typename T>
constexpr T radiansToDegrees(T radians) {
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
template<typename T, std::enable_if_t<std::is_floating_point<T>::value, bool> = true>
constexpr T fmodScalar(T val, T mod) {
	return std::fmod(val, mod);
}

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
	return vec / PIRAD<T>;
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
template<typename T, size_t N>
constexpr T magnitudeSquared(const Vector<T, N>& vec) {
	T result{};

	for (size_t i = 0; i < N; i++) {
		result += vec[i] * vec[i];
	}

	return result;
}
// Normalized
template<typename T, size_t N>
constexpr T magnitude(const Vector<T, N>& vec) {
	return sqrtScalar(magnitudeSquared(vec));
}
// Normalize
template<typename T, size_t N>
constexpr Vector<T, N> normalize(const Vector<T, N>& vec) {
	return vec / magnitude(vec);
}

// Matrix functions
// Get identity Matrices
template<typename T, size_t N>
constexpr Matrix<T, N, N> identityMatrix() {
	Matrix<T, N, N> output{};

	for (size_t i = 0; i < N; i++) {
		output(i, i) = static_cast<T>(1);
	}

	return output;
}
// Transpose
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
// Gaussian elimination
template<typename T, size_t NRow, size_t NCol>
struct RowPermutMatrixPair {
	Permutation1D<NCol> permutation;
	Matrix<T, NRow, NCol> matrix;
};
template<typename T, size_t NRow, size_t NCol>
constexpr RowPermutMatrixPair<T, NRow, NCol> gaussElim(const Matrix<T, NRow, NCol>& mat) {
	RowPermutMatrixPair<T, NRow, NCol> output{ {}, mat };

	for (size_t i = 0; i < NCol; i++) {

		T maxVal = absScalar(output.matrix(i, i));
		size_t maxIndex = i;

		for (size_t row = i + 1; row < NRow; row++) {
			T absVal = absScalar(output.matrix(row, i));
			if (absVal > maxVal) {
				maxVal = absVal;
				maxIndex = row;
			}
		}

		output.matrix.swapRow(i, maxIndex);
		output.permutation.swap(i, maxIndex);

		for (size_t row = i + 1; row < NRow; row++) {
			T factor = output.matrix(row, i) / output.matrix(i, i);

			output.matrix(row, i) = static_cast<T>(0);

			for (size_t col = i + 1; col < NCol; col++) {
				output.matrix(row, col) -= factor * output.matrix(i, col);
			}
		}
	}

	return output;
}

template<typename T, size_t N>
struct RowPermutMatrixResult {
	Permutation1D<N> permutation;
	Matrix<T, N, N> matrix;
	Vector<T, N> result;
};
template<typename T, size_t N>
constexpr RowPermutMatrixResult<T, N> gaussElim(const Matrix<T, N, N>& mat, const Vector<T, N>& vec) {
	Vector<T, N> tmpVec{ vec };
	RowPermutMatrixResult<T, N> output{ {}, mat, {} };

	for (size_t i = 0; i < N; i++) {

		T maxVal = absScalar(output.matrix(i, i));
		size_t maxIndex = i;

		for (size_t row = i + 1; row < N; row++) {
			T absVal = absScalar(output.matrix(row, i));
			if (absVal > maxVal) {
				maxVal = absVal;
				maxIndex = row;
			}
		}

		output.matrix.swapRow(i, maxIndex);

		std::swap(tmpVec[i], tmpVec[maxIndex]);

		for (size_t row = i + 1; row < N; row++) {
			T factor = output.matrix(row, i) / output.matrix(i, i);

			output.matrix(row, i) = static_cast<T>(0);

			for (size_t col = i + 1; col < N; col++) {
				output.matrix(row, col) -= factor * output.matrix(i, col);
			}

			tmpVec[row] -= factor * tmpVec[i];
		}
	}

	for (size_t i = N; i > 0; i--) {
		T sum = tmpVec[i - 1];

		for (size_t j = i; j < N; j++) {
			sum -= output.matrix(i - 1, j) * output.result[j];
		}

		output.result[i - 1] = sum / output.matrix(i - 1, i - 1);
	}

	return output;
}
// LU decomposition
template<typename T, size_t NRow, size_t NCol>
struct PLUData {
	Permutation1D<NCol> permutation;
	Matrix<T, NCol, NCol> lower;
	Matrix<T, NRow, NCol> upper;
};
template<typename T, size_t NRow, size_t NCol>
constexpr PLUData<T, NRow, NCol> plu(const Matrix<T, NRow, NCol>& mat) {
	PLUData<T, NRow, NCol> output{ {}, {}, {mat} };

	for (size_t i = 0; i < NCol; i++) {

		T maxVal = absScalar(output.upper(i, i));
		size_t maxIndex = i;

		for (size_t row = i + 1; row < NRow; row++) {
			T absVal = absScalar(output.upper(row, i));
			if (absVal > maxVal) {
				maxVal = absVal;
				maxIndex = row;
			}
		}

		output.upper.swapRow(i, maxIndex);
		output.permutation.swap(i, maxIndex);

		for (size_t row = i + 1; row < NRow; row++) {
			T factor = output.upper(row, i) / output.upper(i, i);

			output.upper(row, i) = static_cast<T>(0);

			for (size_t col = i + 1; col < NCol; col++) {
				output.upper(row, col) -= factor * output.upper(i, col);
			}

			output.lower(row, i) = factor;
		}
	}

	for (size_t i = 0; i < NCol; i++) {
		output.lower(i, i) = static_cast<T>(1);
	}

	return output;
}
// Cholesky decomposition
// Determinant
template<typename T, size_t N>
constexpr T determinant(const Matrix<T, N, N>& mat) {
	// TODO: Implement generic version
	return 0;
}
template<typename T>
constexpr T determinant(const Matrix<T, 2, 2>& mat) {
	return (mat(0, 0) * mat(1, 1)) - (mat(0, 1) * mat(1, 0));
}
template<typename T>
constexpr T determinant(const Matrix<T, 3, 3>& mat) {
	return mat(axes::x, axes::x) * (mat(axes::y, axes::y) * mat(axes::z, axes::z) - mat(axes::z, axes::y) * mat(axes::y, axes::z))
		- mat(axes::y, axes::x) * (mat(axes::x, axes::y) * mat(axes::z, axes::z) - mat(axes::z, axes::y) * mat(axes::x, axes::z))
		+ mat(axes::z, axes::x) * (mat(axes::x, axes::y) * mat(axes::y, axes::z) - mat(axes::y, axes::y) * mat(axes::x, axes::z));
}
// Inverse
// Adjugate
// Cofactor
// Eigenvalues
// Eigenvectors
// Decompose transform

// Position Matrices
template<typename T, size_t N>
constexpr Matrix<T, N + 1, N + 1> positionMatrix(const Vector<T, N>& pos) {
	Matrix<T, N + 1, N + 1> output = identityMatrix<T, N + 1>();

	for (size_t i = 0; i < N; i++) {
		output(i, N) = pos[i];
	}

	return output;
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
constexpr Matrix<T, 4, 4> perspectiveProjection(T fov, T near, T far, T ratio) {
	T s = static_cast<T>(1) / tanScalar(degreesToRadians(fov / static_cast<T>(2)));
	return {
		{ s * ratio,         static_cast<T>(0), static_cast<T>(0),           static_cast<T>(0) },
		{ static_cast<T>(0), -s,                static_cast<T>(0),           static_cast<T>(0) },
		{ static_cast<T>(0), static_cast<T>(0), (far + near) / (near - far), (2 * far * near) / (near - far) },
		{ static_cast<T>(0), static_cast<T>(0), static_cast<T>(-1),          static_cast<T>(0) },
	};
}

// Rotation Matrices
template<typename T>
constexpr Matrix<T, 2, 2> rotation2DMatrix(T degrees) {
	T rad = degreesToRadians(degrees);
	T sin = sinScalar(rad);
	T cos = cosScalar(rad);
	return {
		{  cos, sin },
		{ -sin, cos },
	};
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
constexpr Matrix<T, 3, 3> eulerRotation3DMatrix(const Vector<T, 3>& degrees, const Rotation3DAxisOrder axisOrder = Rotation3DAxisOrder::YXZ) {
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

// View matrix
template<typename T>
constexpr Matrix<T, 4, 4> viewMatrix(const Vector<T, 3>& eye, const Vector<T, 3>& at, const Vector<T, 3>& up) {
	Vector<T, 3> forward = normalize(at - eye);
	Vector<T, 3> right = normalize(cross(forward, up));
	Vector<T, 3> upDir = cross(forward, right);
	return {
		{ right.x,           right.y,           right.z,           dot(right, -eye) },
		{ upDir.x,           upDir.y,           upDir.z,           dot(upDir, -eye) },
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

// template<typename T>
// matrix3x3<T> toMatrix(quaternion_t<T> quat) {
// 	return {
// 		{ 2 * (quat.w * quat.w + quat.x * quat.x) - 1, 2 * (quat.x * quat.y - quat.w * quat.z),     2 * (quat.x * quat.z + quat.w * quat.y) },
// 		{ 2 * (quat.x * quat.y + quat.w * quat.z),     2 * (quat.w * quat.w + quat.y * quat.y) - 1, 2 * (quat.y * quat.z - quat.w * quat.x) },
// 		{ 2 * (quat.x * quat.z - quat.w * quat.y),     2 * (quat.y * quat.z + quat.w * quat.x),     2 * (quat.w * quat.w + quat.z * quat.z) - 1 },
// 	};
// }


// Equal
template<typename T>
constexpr bool equal(T a, T b, T epsilon = EPSILON<T>) noexcept {
	return absScalar(a - b) <= epsilon;
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
// Modulo
template<typename T, size_t N, std::enable_if_t<std::is_floating_point<T>::value, bool> = true>
constexpr Vector<T, N> operator%(const Vector<T, N>& a, T b) {
	Vector<T, N> output{};

	for (size_t i = 0; i < N; i++) {
		output[i] = fmodScalar(a[i], b);
	}

	return output;
}
template<typename T, size_t N, std::enable_if_t<std::is_integral<T>::value, bool> = true>
constexpr Vector<T, N> operator%(const Vector<T, N>& a, T b) {
	Vector<T, N> output{};

	for (size_t i = 0; i < N; i++) {
		output[i] = a[i] % b;
	}

	return output;
}
// With vectors
template<typename T, size_t N, std::enable_if_t<std::is_floating_point<T>::value, bool> = true>
constexpr Vector<T, N> operator%(const Vector<T, N>& a, const Vector<T, N>& b) {
	Vector<T, N> output{};

	for (size_t i = 0; i < N; i++) {
		output[i] = fmodScalar(a[i], b[i]);
	}

	return output;
}
template<typename T, size_t N, std::enable_if_t<std::is_integral<T>::value, bool> = true>
constexpr Vector<T, N> operator%(const Vector<T, N>& a, const Vector<T, N>& b) {
	Vector<T, N> output{};

	for (size_t i = 0; i < N; i++) {
		output[i] = a[i] % b[i];
	}

	return output;
}
// Compound assign operations
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
// Compound assign operations with scalar
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
// Unary operations
// Negate
template<typename T, size_t N>
constexpr Vector<T, N> operator-(const Vector<T, N>& vec) {
	Vector<T, N> output{};

	for (size_t i = 0; i < N; i++) {
		output[i] = -vec[i];
	}

	return output;
}
// Increment
template<typename T, size_t N>
constexpr Vector<T, N> operator++(Vector<T, N>& vec) {
	for (size_t i = 0; i < N; i++) {
		++vec[i];
	}

	return vec;
}
// Decrement
template<typename T, size_t N>
constexpr Vector<T, N> operator--(Vector<T, N>& vec) {
	for (size_t i = 0; i < N; i++) {
		--vec[i];
	}

	return vec;
}

// Matrix multiplications
// With vector
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

template<typename T, size_t NRow, size_t NCol>
std::ostream& operator<<(std::ostream& os, const Matrix<T, NRow, NCol>& mat) {
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
