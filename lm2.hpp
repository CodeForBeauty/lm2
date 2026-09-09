/*
* A single header library for basic linear math
*/
#pragma once

#ifndef LM2_NOEXCEPT
#define LM2_NOEXCEPT noexcept
#endif

#ifndef LM2_NO_OUTPUT_FUNCTIONS
#include <ostream>
#endif

#include <cmath>
#include <initializer_list>

#include <cassert>

#define LM2_ASSERT(_expression) assert(_expression)

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
	static_assert(N > 0, "Non-zero size is required");

	constexpr Vector() = default;
	constexpr Vector(const Vector<T, N>& v) = default;
	constexpr Vector(Vector<T, N>&& v) = default;

	template<typename... TArgs>
	constexpr Vector(TArgs... args) : data{static_cast<T>(args)...} {
		static_assert(sizeof...(args) <= N, "Too many initializers");
	}

	constexpr T& operator[](size_t index) {
		LM2_ASSERT(index >= 0 && index < N && "Index out of bounds");
		return data[index];
	}

	constexpr const T& operator[](size_t index) const {
		LM2_ASSERT(index >= 0 && index < N && "Index out of bounds");
		return data[index];
	}

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
};


// Matrix types
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

		return output;
	}

	void swapCol(size_t aCol, size_t bCol) {
		if (aCol == bCol) {
			return;
		}
		LM2_ASSERT(aCol >= 0 && aCol < NCol && "Column a out of bounds");
		LM2_ASSERT(bCol >= 0 && bCol < NCol && "Column b out of bounds");
		for (size_t row = 0; row < NRow; row++) {
			std::swap(data[row][aCol], data[row][bCol]);
		}
	}
	
	void swapRow(size_t aRow, size_t bRow) {
		if (aRow == bRow) {
			return;
		}
		LM2_ASSERT(aRow >= 0 && aRow < NRow && "Row a out of bounds");
		LM2_ASSERT(bRow >= 0 && bRow < NRow && "Row b out of bounds");
		for (size_t col = 0; col < NCol; col++) {
			std::swap(data[aRow][col], data[bRow][col]);
		}
	}
};


// Rotation types
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

	constexpr bool isPositive() const {
		return sign;
	}

	constexpr size_t size() const {
		return N;
	}

	constexpr const size_t& operator[](size_t index) const {
		LM2_ASSERT(index >= 0 && index < N && "Index out of bounds");
		return data[index];
	}

	constexpr void swap(size_t aIdx, size_t bIdx) {
		if (aIdx == bIdx) {
			return;
		}
		LM2_ASSERT(aIdx >= 0 && aIdx < N && "Index a out of bounds");
		LM2_ASSERT(bIdx >= 0 && bIdx < N && "Index b out of bounds");
		std::swap(data[aIdx], data[bIdx]);
		sign = !sign;
	}

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

template<typename T>
constexpr bool equalScalar(T a, T b, T epsilon = EPSILON<T>) LM2_NOEXCEPT {
	return absScalar(a - b) <= epsilon;
}

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

template<typename T, std::enable_if_t<std::is_floating_point<T>::value, bool> = true>
T fractScalar(T val) LM2_NOEXCEPT {
	T whole;
	return std::modf(val, &whole);
}
template<typename T, std::enable_if_t<std::is_floating_point<T>::value, bool> = true>
constexpr T fractScalarConst(T val) LM2_NOEXCEPT {
	T whole = static_cast<long long>(val);
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
	return val >= edge;
}
template<typename T>
constexpr T smoothstepScalar(T edge1, T edge2, T val) LM2_NOEXCEPT {
	if (edge1 == edge2) {
		return val >= edge2 ? static_cast<T>(1) : static_cast<T>(0);
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
constexpr T exp2Scalar(T x, T y) LM2_NOEXCEPT {
	return std::exp2(x, y);
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
constexpr Vector<T, N> sqrtVector(const Vector<T, N>& vec) LM2_NOEXCEPT {
	Vector<T, N> output{};

	for (size_t i = 0; i < N; i++) {
		output[i] = sqrtScalar(vec[i]);
	}

	return output;
}
template<typename T, size_t N>
constexpr Vector<T, N> absVector(const Vector<T, N>& vec) LM2_NOEXCEPT {
	Vector<T, N> output{};

	for (size_t i = 0; i < N; i++) {
		output[i] = absScalar(vec[i]);
	}

	return output;
}

template<typename T, size_t N>
constexpr Vector<T, N> sinVector(const Vector<T, N>& vec) LM2_NOEXCEPT {
	Vector<T, N> output{};

	for (size_t i = 0; i < N; i++) {
		output[i] = sinScalar(vec[i]);
	}

	return output;
}
template<typename T, size_t N>
constexpr Vector<T, N> cosVector(const Vector<T, N>& vec) LM2_NOEXCEPT {
	Vector<T, N> output{};

	for (size_t i = 0; i < N; i++) {
		output[i] = cosScalar(vec[i]);
	}

	return output;
}
template<typename T, size_t N>
constexpr Vector<T, N> tanVector(const Vector<T, N>& vec) LM2_NOEXCEPT {
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
constexpr Vector<T, N> normalize(const Vector<T, N>& vec) {
	return vec / magnitude(vec);
}
// Distance
template<typename T, size_t N>
constexpr T distanceSquared(const Vector<T, N>& a, const Vector<T, N>& b) LM2_NOEXCEPT {
	return magnitudeSquared(a - b);
}
template<typename T, size_t N>
constexpr T distance(const Vector<T, N>& a, const Vector<T, N>& b) {
	return sqrtScalar(distanceSquared(a, b));
}
// Lerp
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
		output[i] = fractScalarec(vec[i]);
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
constexpr Vector<T, N> exp2(const Vector<T, N>& x, T y) LM2_NOEXCEPT {
	Vector<T, N> output{};

	for (size_t i = 0; i < N; i++) {
		output[i] = exp2Scalar(x[i], y);
	}

	return output;
}
template<typename T, size_t N>
constexpr Vector<T, N> exp2(T x, const Vector<T, N>& y) LM2_NOEXCEPT {
	Vector<T, N> output{};

	for (size_t i = 0; i < N; i++) {
		output[i] = exp2Scalar(x, y[i]);
	}

	return output;
}
// Angle
template<typename T, size_t N>
constexpr T angle(const Vector<T, N>& a, const Vector<T, N>& b) {
	return arcCosScalar(dot(a, b) / (magnitude(a) * magnitude(b)));
}
// Reflect
template<typename T, size_t N>
constexpr Vector<T, N> reflect(const Vector<T, N>& v, const Vector<T, N> n) LM2_NOEXCEPT {
	return v - static_cast<T>(2) * dot(v, n) * n;
}
// Refract
template<typename T, size_t N>
constexpr Vector<T, N> refract(const Vector<T, N>& normal, const Vector<T, N>& incident, T n1, T n2) {
	T r = n1 / n2;
	T cosI = dot(-normal, incident);
	T sinT2 = r * r * (static_cast<T>(1) - cosI * cosI);
	T cosT = sqrtScalar(maxScalar(static_cast<T>(1) - sinT2, static_cast<T>(0)));
	return r * incident + (r * cosI - cosT) * normal;
}
// Project
template<typename T, size_t N>
constexpr Vector<T, N> project(const Vector<T, N>& a, const Vector<T, N>& b) {
	return (dot(a, b) / dot(b, b)) * b;
}
// Reject
template<typename T, size_t N>
constexpr Vector<T, N> reject(const Vector<T, N>& a, const Vector<T, N>& b) {
	return a - project(a, b);
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
// Cholesky decomposition TODO
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
	return inverse(adjugate(mat));
}
// Eigenvalues TODO
// Eigenvectors TODO
// Decompose transform TODO
// QR decomposition TODO

// Position Matrices
template<typename T, size_t N>
constexpr Matrix<T, N + 1, N + 1> positionMatrix(const Vector<T, N>& pos) LM2_NOEXCEPT {
	Matrix<T, N + 1, N + 1> output = identityMatrix<T, N + 1>();

	for (size_t i = 0; i < N; i++) {
		output(i, N) = pos[i];
	}

	return output;
}

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

// Transform matrix
template<typename T>
constexpr Matrix<T, 4, 4> transformMatrix(const Vector<T, 3>& position, const Vector<T, 3>& rotEuler, const Vector<T, 3>& scale) LM2_NOEXCEPT {
	return scaleMatrix(scale) * eulerRotation3DMatrix(rotEuler) * scaleMatrix(scale);
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
			output[i] += mat(j, i);
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
constexpr Vector<T, N> operator/(const Vector<T, N>& a, const Vector<T, N>& b) {
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
constexpr Vector<T, N> operator/(const Vector<T, N>& a, T b) {
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
constexpr Vector<T, N> operator/(T a, const Vector<T, N>& b) {
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
constexpr Vector<T, N>& operator/=(Vector<T, N>& a, const Vector<T, N>& b) {
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
constexpr Vector<T, N>& operator/=(Vector<T, N>& a, T b) {
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
constexpr Vector<T, N> operator++(Vector<T, N>& vec) LM2_NOEXCEPT {
	for (size_t i = 0; i < N; i++) {
		++vec[i];
	}

	return vec;
}
// Decrement
template<typename T, size_t N>
constexpr Vector<T, N> operator--(Vector<T, N>& vec) LM2_NOEXCEPT {
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
constexpr Matrix<T, NRow, NCol> operator/(const Matrix<T, NRow, NCol>& a, T b) {
	Matrix<T, NRow, NCol> output{};

	for (size_t i = 0; i < NRow; i++) {
		for (size_t j = 0; j < NCol; j++) {
			output(i, j) = a(i, j) / b;
		}
	}

	return output;
}
template<typename T, size_t NRow, size_t NCol, std::enable_if_t<std::is_integral<T>::value || std::is_floating_point<T>::value, bool> = true>
constexpr Matrix<T, NRow, NCol> operator/(T a, const Matrix<T, NRow, NCol>& b) {
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
quaternion_t<T> operator-(quaternion_t<T> quat) {
	return { -quat.w, -quat.x, -quat.y, -quat.z };
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
constexpr Matrix<T, NRow, NCol> operator*(const Permutation1D<NCol>& perm, const Matrix<T, NRow, NCol>& mat) LM2_NOEXCEPT {
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
