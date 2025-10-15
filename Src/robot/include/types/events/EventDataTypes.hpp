/**
 *
 * File to list all the data types used in the event system.
 * NOTE: This is where you should add new data types if they are not already 
 *       included in the list.
 *
 */
#pragma once

#include <boost/variant.hpp>

#include "types/geometry/AbsCoord.hpp"

/**
 * @typedef PackedDataVariant
 * @brief A boost::variant that holds all possible PackedDataTypes.
 *
 * Centralizes all PackedDataTypes to simplify data transmission and ensure type safety.
 */
using PackedDataVariant = boost::variant<
    bool,
    int,
    float,
    double,
    std::string,
    std::array<short, 2>,
    std::array<short, 3>,
    char,
    unsigned int,
    long,
    long long,
    unsigned long,
    unsigned char,
    short,
    unsigned short,
    unsigned long long,
    wchar_t,
    char16_t,
    char32_t,
    uint8_t
    // Add additional PackedDataTypes here
    // , NewPackedDataType
>;

/**
 * @typedef UnpackedDataVariant
 * @brief A boost::variant that holds all possible UnpackedDataTypes.
 *
 * Centralizes all UnpackedDataTypes to simplify data handling and ensure type safety.
 */
using UnpackedDataVariant = boost::variant<
    bool,
    int,
    float,
    double,
    std::string,
    AbsCoord,
    char,
    unsigned int,
    long,
    long long,
    unsigned long,
    unsigned char,
    short,
    unsigned short,
    unsigned long long,
    wchar_t,
    char16_t,
    char32_t
    // Add additional UnpackedDataTypes here
    // , NewUnpackedDataType
>;

// Ensure that all data types are printable

// Overload the << operator for std::array
template <typename T, size_t N>
std::ostream &operator<<(std::ostream &os, const std::array<T, N> &arr) {
    os << "[";
    for (size_t i = 0; i < N; ++i) {
        os << arr[i];
        if (i < N - 1) {
            os << ", ";
        }
    }
    os << "]";
    return os;
}

// Visitor to safely extract a value of type T or fallback
template <typename T>
struct VariantVisitor : public boost::static_visitor<T> {
    template <typename U>
    T operator()(const U&) const {
        // Default fallback if type doesn't match
        return T();
    }

    T operator()(const T &val) const {
        return val;
    }
};

// Helper that uses the visitor to retrieve a packed data type, or returns a default if mismatched
template <typename T>
T getValueOr(const PackedDataVariant &v, const T &defaultValue = T()) {
    return boost::apply_visitor(VariantVisitor<T>(), v);
}

// Helper that uses the visitor to retrieve a packed data type, or returns a default if mismatched
template <typename T>
T getValueOr(const UnpackedDataVariant &v, const T &defaultValue = T()) {
    return boost::apply_visitor(VariantVisitor<T>(), v);
}
