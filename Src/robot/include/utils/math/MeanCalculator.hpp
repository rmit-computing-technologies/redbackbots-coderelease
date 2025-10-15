/**
 * @author Felix Thielke
 */

#include "types/math/Eigen.hpp"

#include <vector>
#include <array>
#include <type_traits>
#include <utility>
#include "bits/std_function.h"


namespace ZeroTypeSelect {
    template<typename T>
    struct EigenZeroType {
        static inline T value() { return T::Zero(); }
    };

    template<typename T>
    struct OtherZeroType {
        static inline constexpr T value() { return T(0); }
    };

    template <typename T>
    static EigenZeroType<T> select(const Eigen::MatrixBase<T>*);

    template <typename T>
    static OtherZeroType<T> select(...);
}

template <typename T>
struct ZeroType : public decltype(ZeroTypeSelect::select<T>(std::declval<T*>())) {};

template<typename ResultType, typename ValueType>
class MeanCalculator {
public:
    using CallbackType = std::function<ResultType(const ValueType&)>;

protected:
    ResultType sum;
    size_t count;
    CallbackType callback;

public:
    MeanCalculator(const CallbackType& callback = MeanCalculator<ResultType, ValueType>::id) : sum(ZeroType<ResultType>::value()), count(0), callback(callback) {}

    static inline ResultType id(const ValueType& v) { return static_cast<ResultType>(v); }

    MeanCalculator& add(const MeanCalculator& v) {
        sum += v.sum;
        count += v.count;
        return *this;
    }

    MeanCalculator& add(const ValueType& v) {
        sum += callback(v);
        count++;
        return *this;
    }

    template<typename IteratorType>
    MeanCalculator& add(const IteratorType& begin, const IteratorType& end) {
        for(auto it = begin; it != end; std::advance(it, 1)) {
            sum += callback(static_cast<ValueType>(*it));
        }
        count += std::distance(begin, end);
        return *this;
    }

    MeanCalculator& add(const std::vector<ValueType>& data) {
        return add(data.cbegin(), data.cend());
    }

    template<size_t N>
    MeanCalculator& add(const std::array<ValueType, N>& data) {
        return add(data.cbegin(), data.cend());
    }

    operator ResultType() const {
        return ResultType(sum / count);
    }
};