/*
* A single header library for basic linear math.
* 
* Best suited for graphics programming and and basic linear algebra operations.
* 
*	Usage examples:
* Vector:
*	lm2::Vector<float, 3> vec{ 0, 5, 1 };
*	vec += 2.0f;
*	vec = lm2::normalize(vec);
*
* Matrix:
*	lm2::Matrix<float, 4, 4> rotMat = lm2::eulerRotation3DMatrix<float>({ 0, 90, 0 }).cast<4, 4>();
*	lm2::Matrix<float, 4, 4> posMat = lm2::positionMatrix<float, 3>({ 2, 0, 0 });
*	lm2::Matrix<float, 4, 4> rotPos = rotMat * posMat;
*
* Quaternion:
*	lm2::Quaternion<float> quat = lm2::makeQuaternion<float>({ 0, 90, 0 }); // Make quaternion out of euler angles
*	lm2::Vector<float, 3> vec{ 1, 0, 0 };
*	lm2::Vector<float, 3> rotatedVec = lm2::rotate(quat, vec);
*/
#pragma once

#ifndef LM2_NOEXCEPT
#define LM2_NOEXCEPT noexcept
#endif

#ifndef LM2_NO_OUTPUT_FUNCTIONS
#include <ostream>
#endif

#include <cstddef>

#include <cmath>
#include <initializer_list>

#include <stdexcept>

#ifdef LM2_NOASSERT
#define LM2_ASSERT(_expression) ((void)0)
#else
#include <cassert>
#define LM2_ASSERT(_expression) assert(_expression)
#endif

namespace lm2 {

template<typename T> constexpr T PI = static_cast<T>(3.1415926535897932384626433832795);
template<typename T> constexpr T E = static_cast<T>(2.7182818284590452353602874713527);
template<typename T> constexpr T PIRAD = PI<T> / static_cast<T>(180);

namespace axes {
	enum {
		x = 0,
		y = 1,
		z = 2,
		w = 3,
	};
};

template<typename T> constexpr T EPSILON = T(0.0000001);

template<typename T>
constexpr T minScalar(T a, T b) LM2_NOEXCEPT {
	return a < b ? a : b;
}
template<typename T>
constexpr T maxScalar(T a, T b) LM2_NOEXCEPT {
	return a > b ? a : b;
}
template<typename T>
constexpr T clampScalar(T val, T low, T high) LM2_NOEXCEPT {
	return minScalar(maxScalar(val, low), high);
}

// Vector types

/// @brief Generic Vector type
/// @tparam T Data type
/// @tparam N Number of dimensions
template<typename T, size_t N>
class Vector {
private:
	T data[N] {};

public:
	static_assert(N > 0, "Non-zero size is required");

	constexpr Vector() = default;
	constexpr Vector(const Vector<T, N>& v) = default;
	constexpr Vector(Vector<T, N>&& v) = default;

	template<typename... TArgs>
	constexpr Vector(TArgs... args) : data{static_cast<T>(args)...} {
		static_assert(sizeof...(args) <= N, "Too many initializers");
	}

	constexpr size_t size() const {
		return N;
	}

	constexpr T* begin() {
		return data;
	}
	constexpr T* end() {
		return data + N;
	}
	constexpr const T* begin() const {
		return data;
	}
	constexpr const T* end() const {
		return data + N;
	}

	constexpr T& operator[](size_t index) {
		LM2_ASSERT(index >= 0 && index < N && "Index out of bounds");
		return data[index];
	}

	constexpr const T& operator[](size_t index) const {
		LM2_ASSERT(index >= 0 && index < N && "Index out of bounds");
		return data[index];
	}

	T& at(size_t index) {
		if (index >= N) {
			throw std::out_of_range("Vector index out of bounds");
		}
		return data[index];
	}
	const T& at(size_t index) const {
		if (index >= N) {
			throw std::out_of_range("Vector index out of bounds");
		}
		return data[index];
	}

	/// @brief Resizes the vector to given size, will truncate the data to fit the given size
	/// @tparam NOut output size
	/// @return A new Vector with given size
	template<size_t NOut>
	constexpr Vector<T, NOut> cast() const {
		static_assert(NOut > 0, "Can't cast Vector to size 0");

		Vector<T, NOut> output{};

		size_t s { N < NOut ? N : NOut };

		for (size_t i = 0; i < s; i++) {
			output[i] = data[i];
		}

		return output;
	}

	constexpr Vector<T, N>& operator=(const Vector<T, N>& other) = default;
};


// Matrix types

/// @brief Generic Matrix type
/// @tparam T Data type
/// @tparam NRow Row count
/// @tparam NCol Column count
template<typename T, size_t NRow, size_t NCol>
class Matrix {
private:
	T data[NRow][NCol] {};

public:
	static_assert(NRow > 0 && NCol > 0, "Non-zero size is required");

	constexpr Matrix() = default;
	constexpr Matrix(const Matrix<T, NRow, NCol>& m) = default;
	constexpr Matrix(Matrix<T, NRow, NCol>&& m) = default;

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

	constexpr size_t sizeRow() const {
		return NRow;
	}
	constexpr size_t sizeCol() const {
		return NCol;
	}

	constexpr T* begin() {
		return &data[0][0];
	}
	constexpr T* end() {
		return (&data[0][0]) + (NRow * NCol);
	}
	constexpr const T* begin() const {
		return &data[0][0];
	}
	constexpr const T* end() const {
		return (&data[0][0]) + (NRow * NCol);
	}

	constexpr T& operator()(size_t rowIndex, size_t columnIndex) {
		LM2_ASSERT(rowIndex >= 0 && rowIndex < NRow && "Row out of bounds");
		LM2_ASSERT(columnIndex >= 0 && columnIndex < NCol && "Column out of bounds");
		return data[rowIndex][columnIndex];
	}

	constexpr const T& operator()(size_t rowIndex, size_t columnIndex) const {
		LM2_ASSERT(rowIndex >= 0 && rowIndex < NRow && "Row out of bounds");
		LM2_ASSERT(columnIndex >= 0 && columnIndex < NCol && "Column out of bounds");
		return data[rowIndex][columnIndex];
	}

	T& at(size_t rowIndex, size_t columnIndex) {
		if (rowIndex >= NRow || columnIndex >= NCol) {
			throw std::out_of_range("Matrix index out of bounds");
		}
		return data[rowIndex][columnIndex];
	}
	const T& at(size_t rowIndex, size_t columnIndex) const {
		if (rowIndex >= NRow || columnIndex >= NCol) {
			throw std::out_of_range("Matrix index out of bounds");
		}
		return data[rowIndex][columnIndex];
	}

	/// @brief Resize the Matrix to given size. Truncates data to fit smaller sizes and expands with diagonal values being set to 1
	/// @tparam NRowOut Output row count
	/// @tparam NColOut Output column count
	/// @return A new Matrix with the given size
	template<size_t NRowOut, size_t NColOut>
	constexpr Matrix<T, NRowOut, NColOut> cast() const {
		static_assert(NRowOut > 0 && NColOut > 0, "Can't cast Matrix to size 0");

		Matrix<T, NRowOut, NColOut> output{};

		size_t s { NRow < NRowOut ? NRow : NRowOut };
		size_t s1 { NCol < NColOut ? NCol : NColOut };

		for (size_t i = 0; i < s; i++) {
			for (size_t j = 0; j < s1; j++) {
				output(i, j) = data[i][j];
			}
		}

		for (size_t i = minScalar(NRow, NCol); i < minScalar(NRowOut, NColOut); i++) {
			output(i, i) = static_cast<T>(1);
		}

		return output;
	}

	/// @brief Permutes Matrix by swapping values from given columns
	/// @param aCol Column A to swap from
	/// @param bCol Column B to swap to
	constexpr void swapCol(size_t aCol, size_t bCol) {
		if (aCol == bCol) {
			return;
		}
		LM2_ASSERT(aCol >= 0 && aCol < NCol && "Column a out of bounds");
		LM2_ASSERT(bCol >= 0 && bCol < NCol && "Column b out of bounds");
		for (size_t row = 0; row < NRow; row++) {
			T tmp = data[row][aCol];
			data[row][aCol] = data[row][bCol];
			data[row][bCol] = tmp;
		}
	}
	
	/// @brief Permutes Matrix by swapping values from given rows
	/// @param aRow Row A to swap from
	/// @param bRow Row B to swap to
	constexpr void swapRow(size_t aRow, size_t bRow) {
		if (aRow == bRow) {
			return;
		}
		LM2_ASSERT(aRow >= 0 && aRow < NRow && "Row a out of bounds");
		LM2_ASSERT(bRow >= 0 && bRow < NRow && "Row b out of bounds");
		for (size_t col = 0; col < NCol; col++) {
			T tmp = data[aRow][col];
			data[aRow][col] = data[bRow][col];
			data[bRow][col] = tmp;
		}
	}

	constexpr Matrix<T, NRow, NCol>& operator=(const Matrix<T, NRow, NCol>& other) = default;
};

/// @brief Axis angle, used to return data from the functions
/// @tparam T Type for data
/// @tparam N Number of dimensions for the axis
template<typename T, size_t N>
struct AxisAnglePair {
	Vector<T, N> axis;
	T angle;
};

// Quaternion

/// @brief Generic Quaternion type, member order is: w, x, y, z
/// @tparam T Data type, required to be floating point number
template<typename T, std::enable_if_t<std::is_floating_point<T>::value, bool> = true>
struct Quaternion {
	T w, x, y, z;
};

// Permutation types

/// @brief A simple one dimensional pemutation type
/// @tparam N Number of elements
template<size_t N>
class Permutation1D {
private:
	size_t data[N] {};
	bool sign = true;

public:
	static_assert(N > 0, "Non-zero size is required");

	constexpr Permutation1D(const Permutation1D<N>& p) = default;
	constexpr Permutation1D(Permutation1D<N>&& p) = default;

	constexpr Permutation1D() : data{} {
		for (size_t i = 0; i < N; i++) {
			data[i] = i;
		}
	}

	/// @brief Check if current state of the permutation positive
	/// @return Internal sign
	constexpr bool isPositive() const {
		return sign;
	}

	constexpr size_t size() const {
		return N;
	}

	constexpr const size_t* begin() const {
		return data;
	}
	constexpr const size_t* end() const {
		return data + N;
	}

	constexpr const size_t& operator[](size_t index) const {
		LM2_ASSERT(index >= 0 && index < N && "Index out of bounds");
		return data[index];
	}

	const size_t& at(size_t index) const {
		if (index >= N) {
			throw std::out_of_range("Permutation index out of bounds");
		}
		return data[index];
	}

	/// @brief Swap two elements
	constexpr void swap(size_t aIdx, size_t bIdx) {
		if (aIdx == bIdx) {
			return;
		}
		LM2_ASSERT(aIdx >= 0 && aIdx < N && "Index a out of bounds");
		LM2_ASSERT(bIdx >= 0 && bIdx < N && "Index b out of bounds");
		size_t tmp = data[aIdx];
		data[aIdx] = data[bIdx];
		data[bIdx] = tmp;
		sign = !sign;
	}

	/// @brief Move one element to given index
	constexpr void move(size_t from, size_t to) {
		if (from == to) {
			return;
		}
		LM2_ASSERT(from >= 0 && from < N && "From index out of bounds");
		LM2_ASSERT(to >= 0 && to < N && "To index out of bounds");
		
		size_t tmpVal = data[from];
		if (from < to) {
			for (size_t i = from; i < to; i++) {
				data[i] = data[i + 1];
			}

			if ((to - from) % 2 != 0) {
				sign = !sign;
			}
		}
		else {
			for (size_t i = from; i > to; i--) {
				data[i] = data[i - 1];
			}

			if ((from - to) % 2 != 0) {
				sign = !sign;
			}
		}
		data[to] = tmpVal;
	}

	constexpr Permutation1D<N>& operator=(const Permutation1D<N>& other) = default;

	template<size_t N1>
	friend constexpr Permutation1D<N1> operator*(const Permutation1D<N1>& a, const Permutation1D<N1>& b) LM2_NOEXCEPT;
	template<size_t N1>
	friend constexpr Permutation1D<N1> inverse(const Permutation1D<N1>& p) LM2_NOEXCEPT;
};

// Scalar math functions
template<typename T, std::enable_if_t<std::is_floating_point<T>::value, bool> = true>
constexpr T degreesToRadians(T degrees) LM2_NOEXCEPT {
	return degrees * PIRAD<T>;
}
template<typename T, std::enable_if_t<std::is_floating_point<T>::value, bool> = true>
constexpr T radiansToDegrees(T radians) LM2_NOEXCEPT {
	return radians / PIRAD<T>;
}

template<typename T>
constexpr T sqrtScalar(T val) LM2_NOEXCEPT {
	return std::sqrt(val);
}
template<typename T, std::enable_if_t<std::is_signed<T>::value, bool> = true>
constexpr T absScalar(T val) LM2_NOEXCEPT {
	return std::abs(val);
}
template<typename T, std::enable_if_t<std::is_floating_point<T>::value, bool> = true>
constexpr T fmodScalar(T val, T mod) LM2_NOEXCEPT {
	return std::fmod(val, mod);
}

template<typename T>
constexpr T sinScalar(T val) LM2_NOEXCEPT {
	return std::sin(val);
}
template<typename T>
constexpr T cosScalar(T val) LM2_NOEXCEPT {
	return std::cos(val);
}
template<typename T>
constexpr T tanScalar(T val) LM2_NOEXCEPT {
	return std::tan(val);
}

template<typename T>
constexpr T arcSinScalar(T val) LM2_NOEXCEPT {
	return std::asin(val);
}
template<typename T>
constexpr T arcCosScalar(T val) LM2_NOEXCEPT {
	return std::acos(val);
}
template<typename T>
constexpr T arcTanScalar(T val) LM2_NOEXCEPT {
	return std::atan(val);
}
template<typename T>
constexpr T arcTan2Scalar(T y, T x) LM2_NOEXCEPT {
	return std::atan2(y, x);
}


template<typename T>
constexpr bool equalScalar(T a, T b, T epsilon = EPSILON<T>) LM2_NOEXCEPT {
	return absScalar(a - b) <= epsilon;
}

/// @brief Linearly interpolate between two values
template<typename T, typename FT, std::enable_if_t<std::is_floating_point<FT>::value, bool> = true>
constexpr T lerpScalar(T a, T b, FT f) LM2_NOEXCEPT {
	return static_cast<T>(static_cast<FT>(a) * f + (static_cast<FT>(1) - f) * static_cast<FT>(b));
}

template<typename T, std::enable_if_t<std::is_floating_point<T>::value, bool> = true>
constexpr T ceilScalar(T val) LM2_NOEXCEPT {
	return std::ceil(val);
}
template<typename T, std::enable_if_t<std::is_floating_point<T>::value, bool> = true>
constexpr T floorScalar(T val) LM2_NOEXCEPT {
	return std::floor(val);
}
template<typename T, std::enable_if_t<std::is_floating_point<T>::value, bool> = true>
constexpr T roundScalar(T val) LM2_NOEXCEPT {
	return std::round(val);
}

/// @brief Get fractional part of a scalar value. This is a more precise version.
template<typename T, std::enable_if_t<std::is_floating_point<T>::value, bool> = true>
T fractScalar(T val) LM2_NOEXCEPT {
	T whole;
	return std::modf(val, &whole);
}
/// @brief This is a more performant approach with precision tradeoff. This function is also constexpr friendly.
template<typename T, std::enable_if_t<std::is_floating_point<T>::value, bool> = true>
constexpr T fractScalarConst(T val) LM2_NOEXCEPT {
	T whole = static_cast<T>(static_cast<long long>(val));
	return val - whole;
}
template<typename T, std::enable_if_t<std::is_floating_point<T>::value, bool> = true>
constexpr T signScalar(T val) LM2_NOEXCEPT {
	return std::copysign(static_cast<T>(1), val);
}
template<typename T, std::enable_if_t<std::is_integral<T>::value, bool> = true>
constexpr T signScalar(T val) LM2_NOEXCEPT {
	return static_cast<T>(val > static_cast<T>(0)) - (val < static_cast<T>(0));
}
template<typename T>
constexpr T stepScalar(T edge, T val) LM2_NOEXCEPT {
	return static_cast<T>(val >= edge);
}
template<typename T>
constexpr T smoothstepScalar(T edge1, T edge2, T val) LM2_NOEXCEPT {
	if (edge1 == edge2) {
		return val >= edge1 ? static_cast<T>(1) : static_cast<T>(0);
	}
	T t = clampScalar((val - edge1) / (edge2 - edge1), static_cast<T>(0), static_cast<T>(1));
	return t * t * (static_cast<T>(3) - static_cast<T>(2) * t);
}

template<typename T>
constexpr T powScalar(T x, T y) LM2_NOEXCEPT {
	return std::pow(x, y);
}
template<typename T>
constexpr T expScalar(T val) LM2_NOEXCEPT {
	return std::exp(val);
}
template<typename T>
constexpr T exp2Scalar(T x) LM2_NOEXCEPT {
	return std::exp2(x);
}


// Basic math functions on vectors
template<typename T, size_t N>
constexpr Vector<T, N> degreesToRadians(const Vector<T, N>& vec) LM2_NOEXCEPT {
	return vec * PIRAD<T>;
}
template<typename T, size_t N>
constexpr Vector<T, N> radiansToDegrees(const Vector<T, N>& vec) LM2_NOEXCEPT {
	return vec / PIRAD<T>;
}

/// @brief Run function per element and add it's output to the output
/// @param vec Input Vector
/// @param func Function to run per component, is required to take (T value, size_t index) and return a T value
/// @return A new Vector with applied function
template<typename T, size_t N, typename FuncT>
constexpr Vector<T, N> compFuncVector(const Vector<T, N>& vec, FuncT&& func) {
	Vector<T, N> output{};

	for (size_t i = 0; i < N; i++) {
		output[i] = func(vec[i], i);
	}

	return output;
}
/// @brief Check if any element in the Vector meets the condition
/// @param vec Input Vector
/// @param condition Function returning bool and taking (T value)
/// @return true if any element meets the condition, false otherwise
template<typename T, size_t N, typename FuncT>
constexpr bool anyVector(const Vector<T, N>& vec, FuncT&& condition) {
	for (size_t i = 0; i < N; i++) {
		if (condition(vec[i])) {
			return true;
		}
	}

	return false;
}
/// @brief Check if all elements in the Vector meet the condition
/// @param vec Input Vector
/// @param condition Function returning bool and taking (T value)
/// @return false if any element doesn't meet the condition, true otherwise
template<typename T, size_t N, typename FuncT>
constexpr bool allVector(const Vector<T, N>& vec, FuncT&& condition) {
	for (size_t i = 0; i < N; i++) {
		if (!condition(vec[i], i)) {
			return false;
		}
	}

	return true;
}
/// @brief Sum all of the elements in the Vector
template<typename T, size_t N>
constexpr T sumVector(const Vector<T, N>& vec) LM2_NOEXCEPT {
	T output{};

	for (size_t i = 0; i < N; i++) {
		output += vec[i];
	}

	return output;
}
/// @brief Multiply all of the elements in the Vector
template<typename T, size_t N>
constexpr T productVector(const Vector<T, N>& vec) LM2_NOEXCEPT {
	T output{};

	for (size_t i = 0; i < N; i++) {
		output *= vec[i];
	}

	return output;
}

/// @brief Sum all of the elements in the Matrix
template<typename T, size_t NRow, size_t NCol>
constexpr T sumMatrix(const Matrix<T, NRow, NCol>& mat) LM2_NOEXCEPT {
	T output{};

	for (size_t i = 0; i < NRow; i++) {
		for (size_t j = 0; j < NCol; j++) {
			output += mat(i, j);
		}
	}

	return output;
}
/// @brief Multiply all of the elements in the Matrix
template<typename T, size_t NRow, size_t NCol>
constexpr T prodMatrix(const Matrix<T, NRow, NCol>& mat) LM2_NOEXCEPT {
	T output{};

	for (size_t i = 0; i < NRow; i++) {
		for (size_t j = 0; j < NCol; j++) {
			output *= mat(i, j);
		}
	}

	return output;
}
/// @brief Sum all of the diagonal elements in the Matrix
template<typename T, size_t NRow, size_t NCol>
constexpr T diagSumMatrix(const Matrix<T, NRow, NCol>& mat) LM2_NOEXCEPT {
	T output{};

	for (size_t i = 0; i < minScalar(NRow, NCol); i++) {
		output += mat(i, i);
	}

	return output;
}
/// @brief Multiply all of the diagonal elements in the Matrix
template<typename T, size_t NRow, size_t NCol>
constexpr T diagProdMatrix(const Matrix<T, NRow, NCol>& mat) LM2_NOEXCEPT {
	T output{};

	for (size_t i = 0; i < minScalar(NRow, NCol); i++) {
		output *= mat(i, i);
	}

	return output;
}

/// @brief Check if any element in the Matrix meets the condition
/// @param mat Input Matrix
/// @param condition Function returning bool and taking (T value)
/// @return True if any element meets the condition, false otherwise
template<typename T, size_t NRow, size_t NCol, typename FuncT>
constexpr bool anyMatrix(const Matrix<T, NRow, NCol>& mat, FuncT&& condition) {
	for (size_t i = 0; i < NRow; i++) {
		for (size_t j = 0; j < NCol; j++) {
			if (condition(mat(i, j))) {
				return true;
			}
		}
	}

	return false;
}
/// @brief Check if all of the elements in the Matrix meet the condition
/// @param mat Input Matrix
/// @param condition Function returning bool and taking (T value)
/// @return False if any element doesn't meet the condition, true otherwise
template<typename T, size_t NRow, size_t NCol, typename FuncT>
constexpr bool allMatrix(const Matrix<T, NRow, NCol>& mat, FuncT&& condition) {
	for (size_t i = 0; i < NRow; i++) {
		for (size_t j = 0; j < NCol; j++) {
			if (!condition(mat(i, j))) {
				return false;
			}
		}
	}

	return true;
}

/// @brief Check if any diagonal element in the Matrix meets the condition
/// @param mat Input Matrix
/// @param condition Function returning bool and taking (T value)
/// @return True if any element meets the condition, false otherwise
template<typename T, size_t NRow, size_t NCol, typename FuncT>
constexpr bool diagAnyMatrix(const Matrix<T, NRow, NCol>& mat, FuncT&& condition) {
	for (size_t i = 0; i < minScalar(NRow, NCol); i++) {
		if (condition(mat(i, i))) {
			return true;
		}
	}

	return false;
}
/// @brief Check if all of the diagonal elements in the Matrix meet the condition
/// @param mat Input Matrix
/// @param condition Function returning bool and taking (T value)
/// @return False if any element doesn't meet the condition, true otherwise
template<typename T, size_t NRow, size_t NCol, typename FuncT>
constexpr bool diadAllMatrix(const Matrix<T, NRow, NCol>& mat, FuncT&& condition) {
	for (size_t i = 0; i < minScalar(NRow, NCol); i++) {
		if (!condition(mat(i, i))) {
			return false;
		}
	}

	return true;
}


template<typename T, size_t N>
constexpr Vector<T, N> sqrt(const Vector<T, N>& vec) LM2_NOEXCEPT {
	Vector<T, N> output{};

	for (size_t i = 0; i < N; i++) {
		output[i] = sqrtScalar(vec[i]);
	}

	return output;
}
template<typename T, size_t N>
constexpr Vector<T, N> abs(const Vector<T, N>& vec) LM2_NOEXCEPT {
	Vector<T, N> output{};

	for (size_t i = 0; i < N; i++) {
		output[i] = absScalar(vec[i]);
	}

	return output;
}

template<typename T, size_t N>
constexpr Vector<T, N> sin(const Vector<T, N>& vec) LM2_NOEXCEPT {
	Vector<T, N> output{};

	for (size_t i = 0; i < N; i++) {
		output[i] = sinScalar(vec[i]);
	}

	return output;
}
template<typename T, size_t N>
constexpr Vector<T, N> cos(const Vector<T, N>& vec) LM2_NOEXCEPT {
	Vector<T, N> output{};

	for (size_t i = 0; i < N; i++) {
		output[i] = cosScalar(vec[i]);
	}

	return output;
}
template<typename T, size_t N>
constexpr Vector<T, N> tan(const Vector<T, N>& vec) LM2_NOEXCEPT {
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
constexpr T dot(const Vector<T, N>& a, const Vector<T, N>& b) LM2_NOEXCEPT {
	T output{};

	for (size_t i = 0; i < N; i++) {
		output += a[i] * b[i];
	}

	return output;
}
// Cross
template<typename T>
constexpr T cross(const Vector<T, 2>& a, const Vector<T, 2>& b) LM2_NOEXCEPT {
	return a[axes::x] * b[axes::y] - a[axes::y] * b[axes::x];
}
template<typename T>
constexpr Vector<T, 3> cross(const Vector<T, 3>& a, const Vector<T, 3>& b) LM2_NOEXCEPT {
	return {
		a[axes::y] * b[axes::z] - a[axes::z] * b[axes::y],
		a[axes::z] * b[axes::x] - a[axes::x] * b[axes::z],
		a[axes::x] * b[axes::y] - a[axes::y] * b[axes::x],
	};
}
// Magnitude
// Squared
template<typename T, size_t N>
constexpr T magnitudeSquared(const Vector<T, N>& vec) LM2_NOEXCEPT {
	T result{};

	for (size_t i = 0; i < N; i++) {
		result += vec[i] * vec[i];
	}

	return result;
}
// Normalized
template<typename T, size_t N>
constexpr T magnitude(const Vector<T, N>& vec) LM2_NOEXCEPT {
	return sqrtScalar(magnitudeSquared(vec));
}
// Normalize
template<typename T, size_t N>
constexpr Vector<T, N> normalize(const Vector<T, N>& vec) LM2_NOEXCEPT {
	return vec / magnitude(vec);
}
// Distance
template<typename T, size_t N>
constexpr T distanceSquared(const Vector<T, N>& a, const Vector<T, N>& b) LM2_NOEXCEPT {
	return magnitudeSquared(a - b);
}
template<typename T, size_t N>
constexpr T distance(const Vector<T, N>& a, const Vector<T, N>& b) LM2_NOEXCEPT {
	return sqrtScalar(distanceSquared(a, b));
}
// Lerp
/// @brief Linearly interpolate two Vectors
template<typename T, size_t N, typename FT, std::enable_if_t<std::is_floating_point<FT>::value, bool> = true>
constexpr Vector<T, N> lerp(const Vector<T, N>& a, const Vector<T, N>& b, FT f) LM2_NOEXCEPT {
	Vector<T, N> output{};

	for (size_t i = 0; i < N; i++) {
		output[i] = lerpScalar(a[i], b[i], f);
	}

	return output;
}
// Min
template<typename T, size_t N>
constexpr Vector<T, N> min(const Vector<T, N>& a, const Vector<T, N>& b) LM2_NOEXCEPT {
	Vector<T, N> output{};

	for (size_t i = 0; i < N; i++) {
		output[i] = minScalar(a[i], b[i]);
	}

	return output;
}
template<typename T, size_t N>
constexpr Vector<T, N> min(const Vector<T, N>& a, T b) LM2_NOEXCEPT {
	Vector<T, N> output{};

	for (size_t i = 0; i < N; i++) {
		output[i] = minScalar(a[i], b);
	}

	return output;
}
// Max
template<typename T, size_t N>
constexpr Vector<T, N> max(const Vector<T, N>& a, const Vector<T, N>& b) LM2_NOEXCEPT {
	Vector<T, N> output{};

	for (size_t i = 0; i < N; i++) {
		output[i] = maxScalar(a[i], b[i]);
	}

	return output;
}
template<typename T, size_t N>
constexpr Vector<T, N> max(const Vector<T, N>& a, T b) LM2_NOEXCEPT {
	Vector<T, N> output{};

	for (size_t i = 0; i < N; i++) {
		output[i] = maxScalar(a[i], b);
	}

	return output;
}
// Clamp
template<typename T, size_t N>
constexpr Vector<T, N> clamp(const Vector<T, N>& a, const Vector<T, N>& low, const Vector<T, N>& high) LM2_NOEXCEPT {
	Vector<T, N> output{};

	for (size_t i = 0; i < N; i++) {
		output[i] = clampScalar(a[i], low[i], high[i]);
	}

	return output;
}
template<typename T, size_t N>
constexpr Vector<T, N> clamp(const Vector<T, N>& a, T low, T high) LM2_NOEXCEPT {
	Vector<T, N> output{};

	for (size_t i = 0; i < N; i++) {
		output[i] = clampScalar(a[i], low, high);
	}

	return output;
}
// Useful functions
template<typename T, size_t N, std::enable_if_t<std::is_floating_point<T>::value, bool> = true>
constexpr Vector<T, N> ceil(const Vector<T, N>& vec) LM2_NOEXCEPT {
	Vector<T, N> output{};

	for (size_t i = 0; i < N; i++) {
		output[i] = ceilScalar(vec[i]);
	}

	return output;
}
template<typename T, size_t N, std::enable_if_t<std::is_floating_point<T>::value, bool> = true>
constexpr Vector<T, N> floor(const Vector<T, N>& vec) LM2_NOEXCEPT {
	Vector<T, N> output{};

	for (size_t i = 0; i < N; i++) {
		output[i] = floorScalar(vec[i]);
	}

	return output;
}
template<typename T, size_t N, std::enable_if_t<std::is_floating_point<T>::value, bool> = true>
constexpr Vector<T, N> round(const Vector<T, N>& vec) LM2_NOEXCEPT {
	Vector<T, N> output{};

	for (size_t i = 0; i < N; i++) {
		output[i] = roundScalar(vec[i]);
	}

	return output;
}

template<typename T, size_t N, std::enable_if_t<std::is_floating_point<T>::value, bool> = true>
constexpr Vector<T, N> fract(const Vector<T, N>& vec) LM2_NOEXCEPT {
	Vector<T, N> output{};

	for (size_t i = 0; i < N; i++) {
		output[i] = fractScalar(vec[i]);
	}

	return output;
}
template<typename T, size_t N>
constexpr Vector<T, N> sign(const Vector<T, N>& vec) LM2_NOEXCEPT {
	Vector<T, N> output{};

	for (size_t i = 0; i < N; i++) {
		output[i] = signScalar(vec[i]);
	}

	return output;
}
template<typename T, size_t N>
constexpr Vector<T, N> step(T edge, const Vector<T, N>& vec) LM2_NOEXCEPT {
	Vector<T, N> output{};

	for (size_t i = 0; i < N; i++) {
		output[i] = stepScalar(edge, vec[i]);
	}

	return output;
}
template<typename T, size_t N>
constexpr Vector<T, N> step(const Vector<T, N>& edge, const Vector<T, N>& vec) LM2_NOEXCEPT {
	Vector<T, N> output{};

	for (size_t i = 0; i < N; i++) {
		output[i] = stepScalar(edge[i], vec[i]);
	}

	return output;
}
template<typename T, size_t N>
constexpr Vector<T, N> smoothstep(T edge1, T edge2, const Vector<T, N>& vec) LM2_NOEXCEPT {
	Vector<T, N> output{};

	for (size_t i = 0; i < N; i++) {
		output[i] = smoothstepScalar(edge1, edge2, vec[i]);
	}

	return output;
}
template<typename T, size_t N>
constexpr Vector<T, N> smoothstep(const Vector<T, N>& edge1, const Vector<T, N>& edge2, const Vector<T, N>& vec) LM2_NOEXCEPT {
	Vector<T, N> output{};

	for (size_t i = 0; i < N; i++) {
		output[i] = smoothstepScalar(edge1[i], edge2[i], vec[i]);
	}

	return output;
}

template<typename T, size_t N>
constexpr Vector<T, N> pow(const Vector<T, N>& x, T y) LM2_NOEXCEPT {
	Vector<T, N> output{};

	for (size_t i = 0; i < N; i++) {
		output[i] = powScalar(x[i], y);
	}

	return output;
}
template<typename T, size_t N>
constexpr Vector<T, N> pow(T x, const Vector<T, N>& y) LM2_NOEXCEPT {
	Vector<T, N> output{};

	for (size_t i = 0; i < N; i++) {
		output[i] = powScalar(x, y[i]);
	}

	return output;
}
template<typename T, size_t N>
constexpr Vector<T, N> exp(const Vector<T, N>& vec) LM2_NOEXCEPT {
	Vector<T, N> output{};

	for (size_t i = 0; i < N; i++) {
		output[i] = expScalar(vec[i]);
	}

	return output;
}
template<typename T, size_t N>
constexpr Vector<T, N> exp2(const Vector<T, N>& x) LM2_NOEXCEPT {
	Vector<T, N> output{};

	for (size_t i = 0; i < N; i++) {
		output[i] = exp2Scalar(x[i]);
	}

	return output;
}
// Angle
/// @brief Get the angle between two Vectors
template<typename T, size_t N>
constexpr T angle(const Vector<T, N>& a, const Vector<T, N>& b) LM2_NOEXCEPT {
	return arcCosScalar(dot(a, b) / (magnitude(a) * magnitude(b)));
}
// Reflect
/// @brief Reflect a Vector from a normal
/// @param v Input Vector
/// @param n Normal to reflect from, expected to be unit length
/// @return Reflected Vector
template<typename T, size_t N>
constexpr Vector<T, N> reflect(const Vector<T, N>& v, const Vector<T, N> n) LM2_NOEXCEPT {
	return v - static_cast<T>(2) * dot(v, n) * n;
}
// Refract
/// @brief Refract a Vector from a normal
/// @param normal Normal to refract from, expected to be unit length
/// @param incident Direction of the hit, expected to be unit length
/// @param n1 IOR of the incident
/// @param n2 IOR of the hit surface
template<typename T, size_t N>
constexpr Vector<T, N> refract(const Vector<T, N>& normal, const Vector<T, N>& incident, T n1, T n2) LM2_NOEXCEPT {
	T r = n1 / n2;
	T cosI = dot(-normal, incident);
	T sinT2 = r * r * (static_cast<T>(1) - cosI * cosI);
	T cosT = sqrtScalar(maxScalar(static_cast<T>(1) - sinT2, static_cast<T>(0)));
	return r * incident + (r * cosI - cosT) * normal;
}
// Project
template<typename T, size_t N>
constexpr Vector<T, N> project(const Vector<T, N>& a, const Vector<T, N>& b) LM2_NOEXCEPT {
	return (dot(a, b) / dot(b, b)) * b;
}
// Reject
template<typename T, size_t N>
constexpr Vector<T, N> reject(const Vector<T, N>& a, const Vector<T, N>& b) LM2_NOEXCEPT {
	return a - project(a, b);
}

// Vector creation functions
template<typename T, size_t N>
constexpr Vector<T, N> filledVector(T val) LM2_NOEXCEPT {
	Vector<T, N> output{};

	for (size_t i = 0; i < N; i++) {
		output[i] = val;
	}

	return output;
}

template<typename T, size_t N>
constexpr Vector<T, N> zeroVector() LM2_NOEXCEPT {
	return {};
}

// Matrix functions
// Get identity Matrices
template<typename T, size_t N>
constexpr Matrix<T, N, N> identityMatrix() LM2_NOEXCEPT {
	Matrix<T, N, N> output{};

	for (size_t i = 0; i < N; i++) {
		output(i, i) = static_cast<T>(1);
	}

	return output;
}
// Filled matrix
template<typename T, size_t NRow, size_t NCol>
constexpr Matrix<T, NRow, NCol> filledMatrix(T val) LM2_NOEXCEPT {
	Matrix<T, NRow, NCol> output;

	for (size_t i = 0; i < NRow; i++) {
		for (size_t j = 0; j < NCol; j++) {
			output(i, j) = val;
		}
	}

	return output;
}
// Transpose
template<typename T, size_t NRow, size_t NCol>
constexpr Matrix<T, NCol, NRow> transpose(const Matrix<T, NRow, NCol>& mat) LM2_NOEXCEPT {
	Matrix<T, NCol, NRow> output{};

	for (size_t i = 0; i < NRow; i++) {
		for (size_t j = 0; j < NCol; j++) {
			output(j, i) = mat(i, j);
		}
	}

	return output;
}
// Gaussian elimination
/// @brief Output type from internal functions
template<typename T, size_t NRow, size_t NCol>
struct RowPermutMatrixPair {
	Permutation1D<NRow> permutation; /// Row permutation
	Matrix<T, NRow, NCol> matrix; /// Row echelon form
};
/// @brief Basic Gaussian Elimination with partial pivoting of rows
/// @param mat Input Matrix
/// @return Matrix Permutation pair
template<typename T, size_t NRow, size_t NCol>
constexpr RowPermutMatrixPair<T, NRow, NCol> gaussElim(const Matrix<T, NRow, NCol>& mat) {
	RowPermutMatrixPair<T, NRow, NCol> output{ {}, mat };

	for (size_t i = 0; i < minScalar(NCol, NRow); i++) {

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
/// @brief Output type from internal functions
template<typename T, size_t N>
struct RowPermutMatrixResult {
	Permutation1D<N> permutation; /// Row permutation
	Matrix<T, N, N> matrix; /// Row echelon form
	Vector<T, N> result; /// Calculated result
};
/// @brief Solve for x from mat * x = vec
/// @param mat Input Matrix
/// @param vec Input Vector
/// @return A structure with permutation, row echelon Matrix, and calculated x
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

		T tmp = tmpVec[i];
		tmpVec[i] = tmpVec[maxIndex];
		tmpVec[maxIndex] = tmp;

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
/// @brief Data structure for PLU decomposition
template<typename T, size_t NRow, size_t NCol>
struct PLUData {
	Permutation1D<NRow> permutation; /// Row permutation
	Matrix<T, NCol, NCol> lower;
	Matrix<T, NRow, NCol> upper;

	/// @brief Perform forward substitution on a given Vector
	/// @param vec Input Vector
	/// @return Result after forward substitution
	constexpr Vector<T, NCol> solve(Vector<T, NCol> vec) const {
		static_assert(NRow == NCol && "Concrete solution only exists for square matrices");

		vec = permutation * vec;

		for (size_t i = 0; i < NCol; i++) {
			for (size_t j = i + 1; j < NRow; j++) {
				vec[j] -= lower(j, i) * vec[i];
			}
		}

		Vector<T, NCol> result{};

		for (size_t i = NRow; i > 0; i--) {
			T sum = vec[i - 1];

			for (size_t j = i; j < NCol; j++) {
				sum -= upper(i - 1, j) * result[j];
			}

			result[i - 1] = sum / upper(i - 1, i - 1);
		}

		return result;
	}
};
/// @brief Perform PLU decomposition on a Matrix
/// @param mat Input Matrix
/// @return PLU data structure
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
// Determinant
template<typename T, size_t N>
constexpr T determinant(const Matrix<T, N, N>& mat) {
	RowPermutMatrixPair<T, N, N> gauss = gaussElim(mat);

	T det = static_cast<T>(1);

	for (size_t i = 0; i < N; i++) {
		det *= gauss.matrix(i, i);
	}
	return gauss.permutation.isPositive() ? det : -det;
}
template<typename T>
constexpr T determinant(const Matrix<T, 2, 2>& mat) LM2_NOEXCEPT {
	return (mat(axes::x, axes::x) * mat(axes::y, axes::y)) - (mat(axes::x, axes::y) * mat(axes::y, axes::x));
}
template<typename T>
constexpr T determinant(const Matrix<T, 3, 3>& mat) LM2_NOEXCEPT {
	return mat(axes::x, axes::x) * (mat(axes::y, axes::y) * mat(axes::z, axes::z) - mat(axes::z, axes::y) * mat(axes::y, axes::z))
		- mat(axes::y, axes::x) * (mat(axes::x, axes::y) * mat(axes::z, axes::z) - mat(axes::z, axes::y) * mat(axes::x, axes::z))
		+ mat(axes::z, axes::x) * (mat(axes::x, axes::y) * mat(axes::y, axes::z) - mat(axes::y, axes::y) * mat(axes::x, axes::z));
}
// Inverse
/// @brief Find the inverse of a Matrix using PLU decomposition
template<typename T, size_t N>
constexpr Matrix<T, N, N> inverse(const Matrix<T, N, N>& mat) {
	PLUData<T, N, N> solver = plu(mat);

	Matrix<T, N, N> output{};

	for (size_t i = 0; i < N; i++) {
		Vector<T, N> dir{};
		dir[i] = static_cast<T>(1);

		dir = solver.solve(dir);

		for (size_t j = 0; j < N; j++) {
			output(j, i) = dir[j];
		}
	}

	return output;
}
template<typename T>
constexpr Matrix<T, 2, 2> inverse(const Matrix<T, 2, 2>& mat) LM2_NOEXCEPT {
	T det = determinant(mat);
	if (equalScalar(det, static_cast<T>(0))) {
		return {};
	}
	T invDet = static_cast<T>(1) / det;
	return {
		{  mat(axes::y, axes::y) * invDet, -mat(axes::x, axes::y) * invDet },
		{ -mat(axes::y, axes::x) * invDet,  mat(axes::x, axes::x) * invDet },
	};
}
// Adjugate
template<typename T, size_t N>
constexpr Matrix<T, N, N> adjugate(const Matrix<T, N, N>& mat) {
	PLUData<T, N, N> solver = plu(mat);

	Matrix<T, N, N> inv{};

	for (size_t i = 0; i < N; i++) {
		Vector<T, N> dir{};
		dir[i] = static_cast<T>(1);

		dir = solver.solve(dir);

		for (size_t j = 0; j < N; j++) {
			inv(j, i) = dir[j];
		}
	}

	T det = static_cast<T>(1);

	for (size_t i = 0; i < N; i++) {
		det *= solver.upper(i, i);
	}

	return inv * (solver.permutation.isPositive() ? det : -det);
}
// Cofactor
template<typename T, size_t N>
constexpr Matrix<T, N, N> cofactor(const Matrix<T, N, N>& mat) {
	return transpose(adjugate(mat));
}

// Position Matrices
template<typename T, size_t N>
constexpr Matrix<T, N + 1, N + 1> positionMatrix(const Vector<T, N>& pos) LM2_NOEXCEPT {
	Matrix<T, N + 1, N + 1> output = identityMatrix<T, N + 1>();

	for (size_t i = 0; i < N; i++) {
		output(i, N) = pos[i];
	}

	return output;
}
// Scale Matrices
template<typename T, size_t N>
constexpr Matrix<T, N, N> scaleMatrix(const Vector<T, N>& scale) LM2_NOEXCEPT {
	Matrix<T, N, N> output{};

	for (size_t i = 0; i < N; i++) {
		output(i, i) = scale[i];
	}

	return output;
}

// Projection Matrices
template<typename T>
constexpr Matrix<T, 4, 4> orthographicProjection(T left, T right, T bottom, T top, T near, T far) LM2_NOEXCEPT {
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
constexpr Matrix<T, 4, 4> orthographicProjection(T width, T height, T near, T far) LM2_NOEXCEPT {
	return {
		{ static_cast<T>(2) / width * (height / width), static_cast<T>(0),          static_cast<T>(0),                 static_cast<T>(0) },
		{ static_cast<T>(0),                            static_cast<T>(2) / height, static_cast<T>(0),                 static_cast<T>(0) },
		{ static_cast<T>(0),                            static_cast<T>(0),          static_cast<T>(-2) / (far - near), -((far + near) / (far - near)) },
		{ static_cast<T>(0),                            static_cast<T>(0),          static_cast<T>(0),                 static_cast<T>(1) },
	};
}
// ratio = height / width
template<typename T>
constexpr Matrix<T, 4, 4> perspectiveProjection(T fov, T near, T far, T ratio) LM2_NOEXCEPT {
	if (fov == static_cast<T>(0)) {
		return identityMatrix<T, 4, 4>();
	}
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
constexpr Matrix<T, 2, 2> rotation2DMatrix(T degrees) LM2_NOEXCEPT {
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
constexpr Matrix<T, 3, 3> eulerRotation3DMatrix(const Vector<T, 3>& degrees, const Rotation3DAxisOrder axisOrder = Rotation3DAxisOrder::YXZ) LM2_NOEXCEPT {
	Vector<T, 3> rad{ degreesToRadians(degrees) };
	Vector<T, 3> s{ sin(rad) };
	Vector<T, 3> c{ cos(rad) };

	Matrix<T, 3, 3> rotX {
		{ 1,  0,      0      },
		{ 0, c[0], -s[0] },
		{ 0, s[0],  c[0] },
	};
	Matrix<T, 3, 3> rotY {
		{  c[1], 0, s[1] },
		{  0,      1, 0      },
		{ -s[1], 0, c[1] },
	};
	Matrix<T, 3, 3> rotZ {
		{ c[2], -s[2], 0 },
		{ s[2],  c[2], 0 },
		{ 0,      0,      1 },
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

template<typename T>
constexpr Matrix<T, 3, 3> rotation3DMatrix(Vector<T, 3> axis, T angle) LM2_NOEXCEPT {
	angle = degreesToRadians(angle);

	T c = cosScalar(angle);
    T s = sinScalar(angle);
    T t = static_cast<T>(1) - c;

	T xt = axis[axes::x] * t;
	T yt = axis[axes::y] * t;
	T zt = axis[axes::z] * t;

	T xyt = axis[axes::x] * yt;
	T xzt = axis[axes::x] * zt;
	T yzt = axis[axes::y] * zt;

	T xs = axis[axes::x] * s;
	T ys = axis[axes::y] * s;
	T zs = axis[axes::z] * s;

	return {
		{ c + axis[axes::x] * xt, xyt - zs,               xzt + ys },
		{ xyt + zs,               c + axis[axes::y] * yt, yzt - xs },
		{ xzt - ys,               yzt + xs,               c + axis[axes::z] * zt },
	};
}

// Transform matrix
/// @brief Construct a TRS matrix with scale applied first, rotation second, translation third
template<typename T>
constexpr Matrix<T, 4, 4> transformMatrix(const Vector<T, 3>& position, const Vector<T, 3>& rotEuler, const Vector<T, 3>& scale) LM2_NOEXCEPT {
	Matrix<T, 4, 4> r = eulerRotation3DMatrix(rotEuler).template cast<4, 4>();
	Matrix<T, 4, 4> s = scaleMatrix(scale).template cast<4, 4>();
	return s * r * positionMatrix(position);
}
template<typename T, size_t NRow, size_t NCol>
constexpr Vector<T, NRow> extractPosition(const Matrix<T, NRow, NCol>& mat) LM2_NOEXCEPT {
	Vector<T, NRow> output{};

	for (size_t i = 0; i < NRow; i++) {
		output[i] = mat(i, NCol - 1);
	}

	return output;
}
template<typename T, size_t N>
constexpr Vector<T, N> extractScale(const Matrix<T, N, N>& mat) LM2_NOEXCEPT {
	Vector<T, N> output{};

	for (size_t i = 0; i < N; i++) {
		for (size_t j = 0; j < N; j++) {
			output[i] += mat(j, i) * mat(j, i);
		}
		output[i] = sqrtScalar(output[i]);
	}

	return output;
}
template<typename T>
void extractTransform(const Matrix<T, 4, 4>& mat, Vector<T, 3>& outPos, Matrix<T, 3, 3>& outRot, Vector<T, 3>& outScale) LM2_NOEXCEPT {
	outRot = mat.template cast<3, 3>();
	outPos = extractPosition(mat).template cast<3>();
	outScale = extractScale(outRot);

	for (size_t i = 0; i < 3; i++) {
		for (size_t j = 0; j < 3; j++) {
			outRot(i, j) /= outScale[i];
		}
	}
}

template<typename T>
void extractTransform(const Matrix<T, 4, 4>& mat, Vector<T, 3>& outPos, Quaternion<T>& outRot, Vector<T, 3>& outScale) LM2_NOEXCEPT {
	Matrix<T, 3, 3> tmpRot{ mat.template cast<3, 3>() };
	outPos = extractPosition(mat).template cast<3>();
	outScale = extractScale(tmpRot);

	for (size_t i = 0; i < 3; i++) {
		for (size_t j = 0; j < 3; j++) {
			tmpRot(i, j) /= outScale[i];
		}
	}

	outRot = makeQuaternion(tmpRot);
}

// View matrix
template<typename T>
constexpr Matrix<T, 4, 4> viewMatrix(const Vector<T, 3>& eye, const Vector<T, 3>& at, const Vector<T, 3>& up) LM2_NOEXCEPT {
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

template<typename T>
constexpr Vector<T, 3> toEulerAngles(const Matrix<T, 3, 3>& mat, Rotation3DAxisOrder axisOrder = Rotation3DAxisOrder::YXZ) LM2_NOEXCEPT {
	switch (axisOrder) {
    case Rotation3DAxisOrder::XYZ:
        return {
			radiansToDegrees( arcTan2Scalar(mat(axes::z, axes::y), mat(axes::z, axes::z)) ),
			radiansToDegrees( arcSinScalar(clampScalar(-mat(axes::z, axes::x), static_cast<T>(-1), static_cast<T>(1))) ),
			radiansToDegrees( arcTan2Scalar(mat(axes::y, axes::x), mat(axes::x, axes::x)) ),
		};
    case Rotation3DAxisOrder::XZY:
		return {
			radiansToDegrees( arcTan2Scalar(-mat(axes::y, axes::z), mat(axes::y, axes::y)) ),
			radiansToDegrees( arcTan2Scalar(-mat(axes::z, axes::x), mat(axes::x, axes::x)) ),
			radiansToDegrees( arcSinScalar(clampScalar(mat(axes::y, axes::x), static_cast<T>(-1), static_cast<T>(1))) ),
		};
    case Rotation3DAxisOrder::YXZ:
		return {
			radiansToDegrees( arcSinScalar(clampScalar(mat(axes::z, axes::y), static_cast<T>(-1), static_cast<T>(1))) ),
			radiansToDegrees( arcTan2Scalar(-mat(axes::z, axes::x), mat(axes::z, axes::z)) ),
			radiansToDegrees( arcTan2Scalar(-mat(axes::x, axes::y), mat(axes::y, axes::y)) ),
		};
    case Rotation3DAxisOrder::YZX:
		return {
			radiansToDegrees( arcTan2Scalar(mat(axes::z, axes::y), mat(axes::y, axes::y)) ),
			radiansToDegrees( arcTan2Scalar(mat(axes::x, axes::z), mat(axes::x, axes::x)) ),
			radiansToDegrees( arcSinScalar(clampScalar(-mat(axes::x, axes::y), static_cast<T>(-1), static_cast<T>(1))) ),
		};
    case Rotation3DAxisOrder::ZXY:
		return {
			radiansToDegrees( arcSinScalar(clampScalar(-mat(axes::y, axes::z), static_cast<T>(-1), static_cast<T>(1))) ),
			radiansToDegrees( arcTan2Scalar(mat(axes::x, axes::z), mat(axes::z, axes::z)) ),
			radiansToDegrees( arcTan2Scalar(mat(axes::y, axes::x), mat(axes::y, axes::y)) ),
		};
    case Rotation3DAxisOrder::ZYX:
		return {
			radiansToDegrees( arcTan2Scalar(-mat(axes::y, axes::z), mat(axes::z, axes::z)) ),
			radiansToDegrees( arcSinScalar(clampScalar(mat(axes::x, axes::z), static_cast<T>(-1), static_cast<T>(1))) ),
			radiansToDegrees( arcTan2Scalar(-mat(axes::x, axes::y), mat(axes::x, axes::x)) ),
		};
	default:
		return {
			radiansToDegrees( arcTan2Scalar(mat(axes::z, axes::y), mat(axes::z, axes::z)) ),
			radiansToDegrees( arcSinScalar(clampScalar(-mat(axes::z, axes::x), static_cast<T>(-1), static_cast<T>(1))) ),
			radiansToDegrees( arcTan2Scalar(mat(axes::y, axes::x), mat(axes::x, axes::x)) ),
		};
    }
}

// Quaternions
template<typename T>
constexpr Quaternion<T> identityQuaternion() LM2_NOEXCEPT {
	return { static_cast<T>(1), static_cast<T>(0), static_cast<T>(0), static_cast<T>(0) };
}
template<typename T>
constexpr T magnitudeSquared(const Quaternion<T>& quat) LM2_NOEXCEPT {
	return (quat.x * quat.x) + (quat.y * quat.y) + (quat.z * quat.z) + (quat.w * quat.w);
}
template<typename T>
constexpr T magnitude(const Quaternion<T>& quat) LM2_NOEXCEPT {
	return sqrtScalar(magnitudeSquared(quat));
}
template<typename T>
constexpr Quaternion<T> normalize(const Quaternion<T>& quat) {
	T mag = magnitude(quat);
	return { quat.w / mag, quat.x / mag, quat.y / mag, quat.z / mag };
}

template<typename T>
constexpr Quaternion<T> conjugate(const Quaternion<T>& quat) LM2_NOEXCEPT {
	return { quat.w, -quat.x, -quat.y, -quat.z };
}

template<typename T>
constexpr Quaternion<T> slerp(const Quaternion<T>& a, const Quaternion<T>& b, T t) {
	T cosom = a.w * b.w + a.x * b.x + a.y * b.y + a.z * b.z;

	Quaternion<T> end{ b };
	if (cosom < static_cast<T>(0)) {
		cosom = -cosom;
		end.x = -end.x;
		end.y = -end.y;
		end.z = -end.z;
		end.w = -end.w;
	}

	T omega = arcCosScalar(cosom);
	T sinom = sinScalar(omega);
	T sclp = sinScalar((static_cast<T>(1) - t) * omega) / sinom;
	T sclq = sinScalar(t * omega) / sinom;
	if ((static_cast<T>(1) - cosom) < EPSILON<T>) {
		sclp = static_cast<T>(1) - t;
		sclq = t;
	}

	return {
		sclp * a.w + sclq * end.w,
		sclp * a.x + sclq * end.x,
		sclp * a.y + sclq * end.y,
		sclp * a.z + sclq * end.z,
	};
}

/// @brief Apply quaternion rotation to a Vector
template<typename T>
constexpr Vector<T, 3> rotate(const Quaternion<T>& quat, const Vector<T, 3>& vec) LM2_NOEXCEPT {
	Vector<T, 3> complex{ quat.x, quat.y, quat.z };
	T real = quat.w;
	return static_cast<T>(2) * dot(complex, vec) * complex
		+ (real * real - dot(complex, complex)) * vec
		+ static_cast<T>(2) * real * cross(complex, vec);
}

template<typename T>
constexpr Matrix<T, 3, 3> toMatrix(const Quaternion<T>& quat) LM2_NOEXCEPT {
	return {
		{ 2 * (quat.w * quat.w + quat.x * quat.x) - 1, 2 * (quat.x * quat.y - quat.w * quat.z),     2 * (quat.x * quat.z + quat.w * quat.y) },
		{ 2 * (quat.x * quat.y + quat.w * quat.z),     2 * (quat.w * quat.w + quat.y * quat.y) - 1, 2 * (quat.y * quat.z - quat.w * quat.x) },
		{ 2 * (quat.x * quat.z - quat.w * quat.y),     2 * (quat.y * quat.z + quat.w * quat.x),     2 * (quat.w * quat.w + quat.z * quat.z) - 1 },
	};
}

template<typename T>
constexpr AxisAnglePair<T, 3> toAxisAngle(const Quaternion<T>& quat) LM2_NOEXCEPT {
	LM2_ASSERT(quat.w <= static_cast<T>(1) && "Quaternion is expected to be normalized");
	T div = sqrtScalar(static_cast<T>(1) - quat.w * quat.w);
	if (div == static_cast<T>(0)) {
		return {};
	}
	return {
		{
			quat.x / div,
			quat.y / div,
			quat.z / div,
		},
		radiansToDegrees(static_cast<T>(2) * arcCosScalar(quat.w)),
	};
}

template<typename T>
constexpr Vector<T, 3> toEulerAngles(Quaternion<T> quat, Rotation3DAxisOrder axisOrder = Rotation3DAxisOrder::YXZ) LM2_NOEXCEPT {
	return toEulerAngles(toMatrix(quat), axisOrder);
}

template<typename T>
constexpr Quaternion<T> makeQuaternion(const Vector<T, 3>& axis, T angle) LM2_NOEXCEPT {
	angle = degreesToRadians(angle / static_cast<T>(2));
	T s = sinScalar(angle);
	return {
		cosScalar(angle),
		axis[axes::x] * s,
		axis[axes::y] * s,
		axis[axes::z] * s,
	};
}

template<typename T>
constexpr Quaternion<T> makeQuaternion(const Vector<T, 3>& eulerAngles, Rotation3DAxisOrder axisOrder = Rotation3DAxisOrder::YXZ) LM2_NOEXCEPT{
	Quaternion<T> qx = makeQuaternion(Vector<T, 3>{ static_cast<T>(1), static_cast<T>(0), static_cast<T>(0) }, eulerAngles[axes::x]);
	Quaternion<T> qy = makeQuaternion(Vector<T, 3>{ static_cast<T>(0), static_cast<T>(1), static_cast<T>(0) }, eulerAngles[axes::y]);
	Quaternion<T> qz = makeQuaternion(Vector<T, 3>{ static_cast<T>(0), static_cast<T>(0), static_cast<T>(1) }, eulerAngles[axes::z]);

	switch (axisOrder) {
	case Rotation3DAxisOrder::XYZ:
		return qx * qy * qz;
	case Rotation3DAxisOrder::XZY:
		return qx * qz * qy;
	case Rotation3DAxisOrder::YXZ:
		return qy * qx * qz;
	case Rotation3DAxisOrder::YZX:
		return qy * qz * qx;
	case Rotation3DAxisOrder::ZXY:
		return qz * qx * qy;
	case Rotation3DAxisOrder::ZYX:
		return qz * qy * qx;
	default:
		return qx * qy * qz;
	}
}

template<typename T>
constexpr Quaternion<T> makeQuaternion(const Matrix<T, 3, 3>& rotMatrix) LM2_NOEXCEPT {
	T dw = static_cast<T>(1) + rotMatrix(axes::x, axes::x) + rotMatrix(axes::y, axes::y) + rotMatrix(axes::z, axes::z);
	T dx = static_cast<T>(1) + rotMatrix(axes::x, axes::x) - rotMatrix(axes::y, axes::y) - rotMatrix(axes::z, axes::z);
	T dy = static_cast<T>(1) - rotMatrix(axes::x, axes::x) + rotMatrix(axes::y, axes::y) - rotMatrix(axes::z, axes::z);
	T dz = static_cast<T>(1) - rotMatrix(axes::x, axes::x) - rotMatrix(axes::y, axes::y) + rotMatrix(axes::z, axes::z);
	Quaternion<T> output{};
	if (dw >= dx && dw >= dy && dw >= dz) {
		T s = sqrtScalar(dw) * static_cast<T>(2);
		output.w = s / static_cast<T>(4);
		output.x = (rotMatrix(axes::z, axes::y) - rotMatrix(axes::y, axes::z)) / s;
		output.y = (rotMatrix(axes::x, axes::z) - rotMatrix(axes::z, axes::x)) / s;
		output.z = (rotMatrix(axes::y, axes::x) - rotMatrix(axes::x, axes::y)) / s;
	} else if (dx >= dy && dx >= dz) {
		T s = sqrtScalar(dx) * static_cast<T>(2);
		output.x = s / static_cast<T>(4);
		output.w = (rotMatrix(axes::z, axes::y) - rotMatrix(axes::y, axes::z)) / s;
		output.y = (rotMatrix(axes::x, axes::y) + rotMatrix(axes::y, axes::x)) / s;
		output.z = (rotMatrix(axes::x, axes::z) + rotMatrix(axes::z, axes::x)) / s;
	} else if (dy >= dz) {
		T s = sqrtScalar(dy) * static_cast<T>(2);
		output.y = s / static_cast<T>(4);
		output.w = (rotMatrix(axes::x, axes::z) - rotMatrix(axes::z, axes::x)) / s;
		output.x = (rotMatrix(axes::x, axes::y) + rotMatrix(axes::y, axes::x)) / s;
		output.z = (rotMatrix(axes::y, axes::z) + rotMatrix(axes::z, axes::y)) / s;
	} else {
		T s = sqrtScalar(dz) * static_cast<T>(2);
		output.z = s / static_cast<T>(4);
		output.w = (rotMatrix(axes::y, axes::x) - rotMatrix(axes::x, axes::y)) / s;
		output.x = (rotMatrix(axes::x, axes::z) + rotMatrix(axes::z, axes::x)) / s;
		output.y = (rotMatrix(axes::y, axes::z) + rotMatrix(axes::z, axes::y)) / s;
	}
	return output;
}

// Permutations
template<size_t N>
constexpr Permutation1D<N> inverse(const Permutation1D<N>& p) LM2_NOEXCEPT {
	Permutation1D<N> output{};

	for (size_t i = 0; i < N; i++) {
		output.data[p[i]] = i;
	}

	output.sign = p.sign;

	return output;
}
template<size_t N>
constexpr Matrix<size_t, N, N> toMatrix(const Permutation1D<N>& p) LM2_NOEXCEPT{
	Matrix<size_t, N, N> output{};

	for (size_t i = 0; i < N; i++) {
		output(i, p[i]) = 1;
	}

	return output;
}


// Equal
template<typename T, size_t N>
constexpr bool equal(const Vector<T, N>& a, const Vector<T, N>& b, T epsilon = EPSILON<T>) LM2_NOEXCEPT {
	for (size_t i = 0; i < N; i++) {
		if (absScalar(a[i] - b[i]) > epsilon) {
			return false;
		}
	}

	return true;
}
template<typename T, size_t N>
constexpr bool equal(const Vector<T, N>& a, T b, T epsilon = EPSILON<T>) LM2_NOEXCEPT {
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
constexpr Vector<T, N> operator+(const Vector<T, N>& a, const Vector<T, N>& b) LM2_NOEXCEPT {
	Vector<T, N> output{};

	for (size_t i = 0; i < N; i++) {
		output[i] = a[i] + b[i];
	}

	return output;
}
template<typename T, size_t N>
constexpr Vector<T, N> operator-(const Vector<T, N>& a, const Vector<T, N>& b) LM2_NOEXCEPT {
	Vector<T, N> output{};

	for (size_t i = 0; i < N; i++) {
		output[i] = a[i] - b[i];
	}

	return output;
}
template<typename T, size_t N>
constexpr Vector<T, N> operator*(const Vector<T, N>& a, const Vector<T, N>& b) LM2_NOEXCEPT {
	Vector<T, N> output{};

	for (size_t i = 0; i < N; i++) {
		output[i] = a[i] * b[i];
	}

	return output;
}
template<typename T, size_t N>
constexpr Vector<T, N> operator/(const Vector<T, N>& a, const Vector<T, N>& b) LM2_NOEXCEPT {
	Vector<T, N> output{};

	for (size_t i = 0; i < N; i++) {
		output[i] = a[i] / b[i];
	}

	return output;
}
// Scalar operations
template<typename T, size_t N>
constexpr Vector<T, N> operator+(const Vector<T, N>& a, T b) LM2_NOEXCEPT {
	Vector<T, N> output{};

	for (size_t i = 0; i < N; i++) {
		output[i] = a[i] + b;
	}

	return output;
}
template<typename T, size_t N>
constexpr Vector<T, N> operator-(const Vector<T, N>& a, T b) LM2_NOEXCEPT {
	Vector<T, N> output{};

	for (size_t i = 0; i < N; i++) {
		output[i] = a[i] - b;
	}

	return output;
}
template<typename T, size_t N>
constexpr Vector<T, N> operator*(const Vector<T, N>& a, T b) LM2_NOEXCEPT {
	Vector<T, N> output{};

	for (size_t i = 0; i < N; i++) {
		output[i] = a[i] * b;
	}

	return output;
}
template<typename T, size_t N>
constexpr Vector<T, N> operator/(const Vector<T, N>& a, T b) LM2_NOEXCEPT {
	Vector<T, N> output{};

	for (size_t i = 0; i < N; i++) {
		output[i] = a[i] / b;
	}

	return output;
}

template<typename T, size_t N>
constexpr Vector<T, N> operator+(T a, const Vector<T, N>& b) LM2_NOEXCEPT {
	return b + a;
}
template<typename T, size_t N>
constexpr Vector<T, N> operator-(T a, const Vector<T, N>& b) LM2_NOEXCEPT {
	Vector<T, N> output{};

	for (size_t i = 0; i < N; i++) {
		output[i] = a - b[i];
	}

	return output;
}
template<typename T, size_t N>
constexpr Vector<T, N> operator*(T a, const Vector<T, N>& b) LM2_NOEXCEPT {
	return b * a;
}
template<typename T, size_t N>
constexpr Vector<T, N> operator/(T a, const Vector<T, N>& b) LM2_NOEXCEPT {
	Vector<T, N> output{};

	for (size_t i = 0; i < N; i++) {
		output[i] = a / b[i];
	}

	return output;
}
// Modulo
template<typename T, size_t N, std::enable_if_t<std::is_floating_point<T>::value, bool> = true>
constexpr Vector<T, N> operator%(const Vector<T, N>& a, T b) LM2_NOEXCEPT {
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
constexpr Vector<T, N>& operator+=(Vector<T, N>& a, const Vector<T, N>& b) LM2_NOEXCEPT {
	for (size_t i = 0; i < N; i++) {
		a[i] += b[i];
	}

	return a;
}
template<typename T, size_t N>
constexpr Vector<T, N>& operator-=(Vector<T, N>& a, const Vector<T, N>& b) LM2_NOEXCEPT {
	for (size_t i = 0; i < N; i++) {
		a[i] -= b[i];
	}

	return a;
}
template<typename T, size_t N>
constexpr Vector<T, N>& operator*=(Vector<T, N>& a, const Vector<T, N>& b) LM2_NOEXCEPT {
	for (size_t i = 0; i < N; i++) {
		a[i] *= b[i];
	}

	return a;
}
template<typename T, size_t N>
constexpr Vector<T, N>& operator/=(Vector<T, N>& a, const Vector<T, N>& b) LM2_NOEXCEPT {
	for (size_t i = 0; i < N; i++) {
		a[i] /= b[i];
	}

	return a;
}
// Compound assign operations with scalar
template<typename T, size_t N>
constexpr Vector<T, N>& operator+=(Vector<T, N>& a, T b) LM2_NOEXCEPT {
	for (size_t i = 0; i < N; i++) {
		a[i] += b;
	}

	return a;
}
template<typename T, size_t N>
constexpr Vector<T, N>& operator-=(Vector<T, N>& a, T b) LM2_NOEXCEPT {
	for (size_t i = 0; i < N; i++) {
		a[i] -= b;
	}

	return a;
}
template<typename T, size_t N>
constexpr Vector<T, N>& operator*=(Vector<T, N>& a, T b) LM2_NOEXCEPT {
	for (size_t i = 0; i < N; i++) {
		a[i] *= b;
	}

	return a;
}
template<typename T, size_t N>
constexpr Vector<T, N>& operator/=(Vector<T, N>& a, T b) LM2_NOEXCEPT {
	for (size_t i = 0; i < N; i++) {
		a[i] /= b;
	}

	return a;
}
// Unary operations
// Negate
template<typename T, size_t N>
constexpr Vector<T, N> operator-(const Vector<T, N>& vec) LM2_NOEXCEPT {
	Vector<T, N> output{};

	for (size_t i = 0; i < N; i++) {
		output[i] = -vec[i];
	}

	return output;
}
// Increment
template<typename T, size_t N>
constexpr Vector<T, N>& operator++(Vector<T, N>& vec) LM2_NOEXCEPT {
	for (size_t i = 0; i < N; i++) {
		++vec[i];
	}

	return vec;
}
// Decrement
template<typename T, size_t N>
constexpr Vector<T, N>& operator--(Vector<T, N>& vec) LM2_NOEXCEPT {
	for (size_t i = 0; i < N; i++) {
		--vec[i];
	}

	return vec;
}

// Matrix operations
// With vector
template<typename T, size_t NRow, size_t NCol>
constexpr Vector<T, NRow> operator*(const Matrix<T, NRow, NCol>& mat, const Vector<T, NCol>& vec) LM2_NOEXCEPT {
	Vector<T, NRow> output{};

	for (size_t i = 0; i < NRow; i++) {
		for (size_t j = 0; j < NCol; j++) {
			output[i] += vec[j] * mat(i, j);
		}
	}

	return output;
}
template<typename T, size_t NRow, size_t NCol>
constexpr Vector<T, NCol> operator*(const Vector<T, NRow>& vec, const Matrix<T, NRow, NCol>& mat) LM2_NOEXCEPT {
	Vector<T, NCol> output{};

	for (size_t j = 0; j < NCol; j++) {
		for (size_t i = 0; i < NRow; i++) {
			output[j] += vec[i] * mat(i, j);
		}
	}

	return output;
}
// With scalar
template<typename T, size_t NRow, size_t NCol, std::enable_if_t<std::is_integral<T>::value || std::is_floating_point<T>::value, bool> = true>
constexpr Matrix<T, NRow, NCol> operator*(const Matrix<T, NRow, NCol>& a, T b) LM2_NOEXCEPT {
	Matrix<T, NRow, NCol> output{};

	for (size_t i = 0; i < NRow; i++) {
		for (size_t j = 0; j < NCol; j++) {
			output(i, j) = a(i, j) * b;
		}
	}

	return output;
}
template<typename T, size_t NRow, size_t NCol, std::enable_if_t<std::is_integral<T>::value || std::is_floating_point<T>::value, bool> = true>
constexpr Matrix<T, NRow, NCol> operator/(const Matrix<T, NRow, NCol>& a, T b) LM2_NOEXCEPT {
	Matrix<T, NRow, NCol> output{};

	for (size_t i = 0; i < NRow; i++) {
		for (size_t j = 0; j < NCol; j++) {
			output(i, j) = a(i, j) / b;
		}
	}

	return output;
}
template<typename T, size_t NRow, size_t NCol, std::enable_if_t<std::is_integral<T>::value || std::is_floating_point<T>::value, bool> = true>
constexpr Matrix<T, NRow, NCol> operator/(T a, const Matrix<T, NRow, NCol>& b) LM2_NOEXCEPT {
	Matrix<T, NRow, NCol> output{};

	for (size_t i = 0; i < NRow; i++) {
		for (size_t j = 0; j < NCol; j++) {
			output(i, j) = a / b(i, j);
		}
	}

	return output;
}
template<typename T, size_t NRow, size_t NCol, std::enable_if_t<std::is_integral<T>::value || std::is_floating_point<T>::value, bool> = true>
constexpr Matrix<T, NRow, NCol> operator+(const Matrix<T, NRow, NCol>& a, T b) LM2_NOEXCEPT {
	Matrix<T, NRow, NCol> output{};

	for (size_t i = 0; i < NRow; i++) {
		for (size_t j = 0; j < NCol; j++) {
			output(i, j) = a(i, j) + b;
		}
	}

	return output;
}
template<typename T, size_t NRow, size_t NCol, std::enable_if_t<std::is_integral<T>::value || std::is_floating_point<T>::value, bool> = true>
constexpr Matrix<T, NRow, NCol> operator-(const Matrix<T, NRow, NCol>& a, T b) LM2_NOEXCEPT {
	Matrix<T, NRow, NCol> output{};

	for (size_t i = 0; i < NRow; i++) {
		for (size_t j = 0; j < NCol; j++) {
			output(i, j) = a(i, j) - b;
		}
	}

	return output;
}
template<typename T, size_t NRow, size_t NCol, std::enable_if_t<std::is_integral<T>::value || std::is_floating_point<T>::value, bool> = true>
constexpr Matrix<T, NRow, NCol> operator-(T a, const Matrix<T, NRow, NCol>& b) LM2_NOEXCEPT {
	Matrix<T, NRow, NCol> output{};

	for (size_t i = 0; i < NRow; i++) {
		for (size_t j = 0; j < NCol; j++) {
			output(i, j) = a - b(i, j);
		}
	}

	return output;
}
// With matrix
template<typename T, size_t NRow, size_t NCol, size_t NRow1>
constexpr Matrix<T, NRow1, NCol> operator*(const Matrix<T, NRow, NCol>& a, const Matrix<T, NRow1, NRow>& b) LM2_NOEXCEPT {
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
constexpr Quaternion<T> operator-(const Quaternion<T>& quat) LM2_NOEXCEPT {
	return { -quat.w, -quat.x, -quat.y, -quat.z };
}

template<typename T>
constexpr Quaternion<T> operator*(const Quaternion<T>& a, T b) LM2_NOEXCEPT {
	return { a.w * b, a.x * b, a.y * b, a.z * b };
}
template<typename T>
constexpr Quaternion<T> operator/(const Quaternion<T>& a, T b) LM2_NOEXCEPT {
	return { a.w / b, a.x / b, a.y / b, a.z / b };
}

template<typename T>
constexpr Quaternion<T> operator*(const Quaternion<T>& a, const Quaternion<T>& b) LM2_NOEXCEPT {
	return {
		b.w * a.w - b.x * a.x - b.y * a.y - b.z * a.z,
		b.w * a.x + b.x * a.w + b.y * a.z - b.z * a.y,
		b.w * a.y + b.y * a.w + b.z * a.x - b.x * a.z,
		b.w * a.z + b.z * a.w + b.x * a.y - b.y * a.x,
	};
}

// Permutations
template<size_t N>
constexpr Permutation1D<N> operator*(const Permutation1D<N>& a, const Permutation1D<N>& b) LM2_NOEXCEPT {
	Permutation1D<N> output{ a };

	for (size_t i = 0; i < N; i++) {
		output.data[i] = a[b[i]];
	}

	output.sign = a.sign == b.sign;

	return output;
}
template<typename T, size_t N>
constexpr Vector<T, N> operator*(const Permutation1D<N>& perm, const Vector<T, N>& vec) LM2_NOEXCEPT {
	Vector<T, N> output{};

	for (size_t i = 0; i < N; i++) {
		output[i] = vec[perm[i]];
	}

	return output;
}
template<typename T, size_t N>
constexpr Vector<T, N> operator*(const Vector<T, N>& vec, const Permutation1D<N>& perm) LM2_NOEXCEPT {
	return perm * vec;
}

template<typename T, size_t NRow, size_t NCol>
constexpr Matrix<T, NRow, NCol> operator*(const Matrix<T, NRow, NCol>& mat, const Permutation1D<NCol>& perm) LM2_NOEXCEPT {
	Matrix<T, NRow, NCol> output{};

	for (size_t i = 0; i < NRow; i++) {
		for (size_t j = 0; j < NCol; j++) {
			output(i, j) = mat(i, perm[j]);
		}
	}

	return output;
}
template<typename T, size_t NRow, size_t NCol>
constexpr Matrix<T, NRow, NCol> operator*(const Permutation1D<NRow>& perm, const Matrix<T, NRow, NCol>& mat) LM2_NOEXCEPT {
	Matrix<T, NRow, NCol> output{};

	for (size_t i = 0; i < NRow; i++) {
		for (size_t j = 0; j < NCol; j++) {
			output(i, j) = mat(perm[i], j);
		}
	}

	return output;
}


#ifndef LM2_NO_OUTPUT_FUNCTIONS

template<typename T, size_t N>
std::ostream& operator<<(std::ostream& os, const Vector<T, N>& vec) {
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

template<typename T>
std::ostream& operator<<(std::ostream& os, const Quaternion<T>& quat) {
	return os << quat.w << ", " << quat.x << ", " << quat.y << ", " << quat.z;
}

template<size_t N>
std::ostream& operator<<(std::ostream& os, const Permutation1D<N>& perm) {
	for (size_t i = 0; i < N; i++) {
		os << perm[i];
		if (i != N - 1) {
			os << ", ";
		}
	}

	return os;
}

#endif // #ifndef LM2_NO_OUTPUT_FUNCTIONS
} // namespace lm2
