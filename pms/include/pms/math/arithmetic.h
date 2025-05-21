#pragma once

#include <cmath>
#include <concepts>
#include <limits>
#include <type_traits>

#include "pms/math/definitions.h"

namespace pms {
namespace Arithmetic {

template <typename T>
concept Number = std::is_arithmetic_v<T> && std::totally_ordered<T>;

template <Number T>
constexpr T truncateToDecimalPlaces(const T& number, const int decimalPlaces) {
    Scalar factor = std::pow(10.0, decimalPlaces);
    return std::trunc(number * factor) / factor;
}

/**
 * @brief Sign function
 */
template <Number T>
constexpr int sgn(const T& number) {
    return (T(0) < number) - (number < T(0));
}

/**
 * @brief Sign function (0 is considered positive)
 */
template <Number T>
constexpr T sgnPos(const T& number) {
    return (number >= T(0)) - (number < T(0));
}

/**
 * @brief Sign function (0 is considered negative)
 */
template <Number T>
constexpr T sgnNeg(const T& number) {
    return (number > T(0)) - (number <= T(0));
}

template <Number T>
bool isZero(T number, Scalar eps = std::numeric_limits<Scalar>::epsilon()) {
    return std::abs(number) < eps;
}

template <Number T>
bool isEqual(T a, T b, Scalar eps = std::numeric_limits<Scalar>::epsilon()) {
    const T diff = std::abs(a - b);
    return diff < eps || diff < eps * std::max(std::abs(a), std::abs(b));
}

}  // namespace Arithmetic
}  // namespace pms