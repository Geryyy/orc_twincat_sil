/*
    Project: ORC - Open Robot Control Library
    Author: anonymous
    Date: Fri Dec 1 22:02:39 2023 +0100
    License: See accompanying LICENSE file
    */

#pragma once

#ifndef TIME_H
#define TIME_H

#include <vector>

#ifndef TC_VER
#include <iomanip>
#include <iostream>
#include <sstream>
#include <stdexcept>
#endif
namespace orc {

#if defined(_MSC_VER)
#pragma pack(push, 1)
#endif

/**
 * @brief
 *
 */
class Time {
private:
    int64_t sec;   // seconds
    int64_t nsec;  // nanoseconds

public:
    constexpr Time() : sec(0), nsec(0) {}
    constexpr Time(int64_t sec, int64_t nsec) : sec(sec), nsec(nsec) {}

    /**
     * @brief Convert Time to seconds as a double.
     *
     * @return constexpr double
     */
    constexpr double toSec() const {
        return static_cast<double>(sec) + static_cast<double>(nsec) / 1'000'000'000LL;
    }

    /**
     * @brief Quantize Time to the nearest multiple of the given Time step.
     *
     * @param time_step Quantization step as a Time object.
     * @return constexpr Time
     */
    constexpr Time quantize(Time time_step) const {
        if (time_step.toNSec() == 0) {
            return *this;  // Time step cannot be zero
        }

        // Calculate the remainder of the division
        int64_t remainder = toNSec() % time_step.toNSec();

        // Calculate the adjustment needed to round to the closest multiple
        int64_t adjust = (2 * remainder >= time_step.toNSec()) ? time_step.toNSec() : 0;

        // Round to the closest multiple of time_step
        int64_t rounded_nsec = (toNSec() / time_step.toNSec()) * time_step.toNSec() + adjust;
        int64_t rounded_sec = rounded_nsec / 1'000'000'000LL;
        rounded_nsec %= 1'000'000'000LL;

        return Time(rounded_sec, rounded_nsec);
    }

    /**
     * @brief Construct Time from seconds as a double.
     *
     * @param seconds Time in seconds as a double.
     */
    constexpr Time(double seconds)
        : sec(static_cast<int64_t>(seconds)),
          nsec(static_cast<int64_t>((seconds - static_cast<double>(sec)) * 1'000'000'000LL)) {}

    /**
     * @brief Getters for seconds and nanoseconds.
     *
     * @return constexpr int64_t
     */
    constexpr int64_t get_sec() const { return sec; }

    /**
     * @brief Getters for nanoseconds.
     *
     * @return constexpr int64_t
     */
    constexpr int64_t get_nsec() const { return nsec; }

    /**
     * @brief Conversion operator to double for seconds.
     *
     * @return double
     */
    explicit constexpr operator double() const { return toSec(); }

    /**
     * @brief Conversion operator to int64_t for nanoseconds.
     *
     * @return constexpr int64_t
     */
    explicit constexpr operator int64_t() const { return toNSec(); }

    /**
     * @brief Convert Time to nanoseconds.
     *
     * @return constexpr int64_t
     */
    constexpr int64_t toNSec() const {
        return static_cast<int64_t>(sec) * 1'000'000'000LL + static_cast<int64_t>(nsec);
    }

    /**
     * @brief Construct a Time object from nanoseconds.
     *
     * Unlike the (int64, int64) constructor, this handles values where ns
     * exceeds 1e9 and returns a normalized result.
     */
    static constexpr Time fromNSec(int64_t total_ns) {
        return Time(total_ns / 1'000'000'000LL, total_ns % 1'000'000'000LL);
    }

    /**
     * @brief Normalize the Time object to ensure nsec is within [0, 1'000'000'000).
     *
     */
    constexpr void normalize() {
        if (nsec >= 1'000'000'000LL) {
            sec += nsec / 1'000'000'000LL;
            nsec %= 1'000'000'000LL;
        } else if (nsec < 0) {
            sec += (nsec - 1'000'000'000LL + 1) / 1'000'000'000LL;
            nsec = 1'000'000'000LL + (nsec % 1'000'000'000LL);
            // Boundary: for nsec = -k·1e9 the sec adjustment already absorbs
            // the full second, so the correction above yields nsec == 1e9.
            // Fold it back to 0 without re-incrementing sec.
            if (nsec == 1'000'000'000LL)
                nsec = 0;
        }
    }

    /**
     * @brief Overloading Operator - to negate the Time object.
     *
     * @return constexpr Time
     */
    constexpr Time operator-() const {
        // Negate both seconds and nanoseconds
        return Time(-sec, -nsec);
    }

    /**
     * @brief Overloading Operator + to add two Time objects
     *
     * @param other Time object to add
     * @return constexpr Time
     */
    constexpr Time operator+(const Time& other) const {
        int64_t newSec = sec + other.sec;
        int64_t newNSec = nsec + other.nsec;
        if (newNSec >= 1'000'000'000LL) {
            newSec++;
            newNSec -= 1'000'000'000LL;
        }
        return Time(newSec, newNSec);
    }

    /**
     * @brief Overloading Operator + to add a double value to Time value.
     *
     * @param value double value to add
     * @return constexpr Time
     */
    constexpr Time operator+(double value) const {
        double totalSec = toSec() + value;

        int64_t sec = static_cast<int64_t>(totalSec);
        int64_t nsec =
            static_cast<int64_t>((totalSec - static_cast<double>(sec)) * 1'000'000'000LL);

        return Time(sec, nsec);
    }

    /**
     * @brief Overloading Operator + to add a double value and a Time value.
     *
     * @param lhs double value to add
     * @param rhs Time object to add
     * @return constexpr Time
     */
    constexpr friend Time operator+(double lhs, const Time& rhs) {
        return rhs + lhs;  // Commutative property; reusing the Time + double operator
    }

    /**
     * @brief Overloading Operator += to add a Time object.
     *
     * @param other Time object to add
     * @return constexpr Time&
     */
    constexpr Time& operator+=(const Time& other) {
        *this = *this + other;
        return *this;
    }

    /**
     * @brief Overloading Operator - to subtract two Time objects.
     *
     * @param other Time object to subtract
     * @return constexpr Time
     */
    constexpr Time operator-(const Time& other) const {
        int64_t newSec = sec - other.sec;
        int64_t newNSec = nsec - other.nsec;
        if (newNSec < 0) {
            newSec--;
            newNSec += 1'000'000'000LL;
        }
        return Time(newSec, newNSec);
    }

    /**
     * @brief Overloading Operator - to subtract a double value from Time value.
     *
     * @param value double value to subtract
     * @return constexpr Time
     */
    constexpr Time operator-(double value) const {
        double totalSec = toSec() - value;

        int64_t sec = static_cast<int64_t>(totalSec);
        int64_t nsec =
            static_cast<int64_t>((totalSec - static_cast<double>(sec)) * 1'000'000'000LL);

        Time result(sec, nsec);
        result.normalize();
        return result;
    }

    /**
     * @brief Overloading Operator - to subtract a double value from Time value.
     *
     * @param lhs double value to subtract
     * @param rhs Time object to subtract
     * @return constexpr Time
     */
    constexpr friend Time operator-(double lhs, const Time& rhs) {
        return Time(lhs) - rhs;  // Convert lhs to Time and use Time - Time operator
    }

    /**
     * @brief Overloading Operator -= to subtract a Time object.
     *
     * @param other
     * @return constexpr Time&
     */
    constexpr Time& operator-=(const Time& other) {
        *this = *this - other;
        return *this;
    }

    /**
     * @brief Overloading Operator * to multiply Time by an int factor.
     *
     * @param factor int factor to multiply
     * @return constexpr Time
     */
    constexpr Time operator*(const int& factor) const {
        int64_t totalNsec = toNSec() * factor;
        return Time(totalNsec / 1'000'000'000LL, totalNsec % 1'000'000'000LL);
    }

    /**
     * @brief Overloading Operator * to multiply Time by an int factor.
     *
     * @param factor int factor to multiply
     * @param time Time object to multiply
     * @return constexpr Time
     */
    friend constexpr Time operator*(const int& factor, const Time& time) { return time * factor; }

    /**
     * @brief Overloading Operator * to multiply Time by a Time object.
     *
     * @param other Time object to multiply
     * @return constexpr Time
     */
    Time operator*(const Time& other) const {
        int64_t sec_prod = static_cast<int64_t>(sec) * static_cast<int64_t>(other.sec);
        int64_t nsec_prod =
            (static_cast<int64_t>(nsec) * static_cast<int64_t>(other.nsec)) / 1'000'000'000LL;

        int64_t sec_nsec_prod = static_cast<int64_t>(sec) * static_cast<int64_t>(other.nsec);
        int64_t nsec_sec_prod = static_cast<int64_t>(nsec) * static_cast<int64_t>(other.sec);

        nsec_prod += sec_nsec_prod + nsec_sec_prod;

        sec_prod += nsec_prod / 1'000'000'000LL;
        nsec_prod %= 1'000'000'000LL;

        Time result(sec_prod, nsec_prod);
        return result;
    }

    /**
     * @brief Overloading Operator * to multiply Time by a double value.
     *
     * @param value double value to multiply
     * @return constexpr Time
     */
    constexpr Time operator*(double value) const {
        double totalSec = toSec() * value;

        int64_t sec = static_cast<int64_t>(totalSec);
        int64_t nsec =
            static_cast<int64_t>((totalSec - static_cast<double>(sec)) * 1'000'000'000LL);

        return Time(sec, nsec);
    }

    /**
     * @brief Overloading Operator * to multiply a double value by a Time object.
     *
     * @param lhs double value to multiply
     * @param rhs Time object to multiply
     * @return constexpr Time
     */
    constexpr friend Time operator*(double lhs, const Time& rhs) {
        return rhs * lhs;  // Commutative property; reusing the Time * double operator
    }

    /**
     * @brief Overloading Operator *= to multiply by a Time object.
     *
     * @param other int64_t factor to multiply
     * @return constexpr Time&
     */
    constexpr Time operator*(const int64_t& factor) const {
        int64_t totalNsec = toNSec() * factor;
        return Time(totalNsec / 1'000'000'000LL, totalNsec % 1'000'000'000LL);
    }

    /**
     * @brief Overloading Operator / to divide Time by a Time object.
     *
     * @param lhs Time object to divide
     * @param rhs Time object to divide by
     * @return constexpr Time
     */
    friend constexpr Time operator/(const Time& lhs, const Time& rhs) {
        // Convert everything to nanoseconds for accurate division
        double lhs_total_ns = static_cast<double>(lhs.toNSec());
        double rhs_total_ns = static_cast<double>(rhs.toNSec());

        // RT-safe divide-by-zero handling: exceptions are forbidden in TwinCAT /
        // Codesys RT C++ environments. Outside TC_VER we throw so misuse fails
        // loudly; inside TC_VER we degrade to Time{0,0} and rely on the caller
        // to have validated the denominator.
        if (rhs_total_ns == 0) {
#ifndef TC_VER
            throw std::runtime_error("orc::Time::operator/: division by zero");
#else
            return Time{0, 0};
#endif
        }

        // Perform division using floating-point arithmetic
        double result_ns = lhs_total_ns / rhs_total_ns;

        // Separate seconds and nanoseconds from the result
        int64_t result_sec = static_cast<int64_t>(result_ns);
        int64_t result_nsec =
            static_cast<int64_t>((result_ns - static_cast<double>(result_sec)) * 1'000'000'000LL);

        return Time{static_cast<int64_t>(result_sec), static_cast<int64_t>(result_nsec)};
    }

    /**
     * @brief Overloading Operator / to divide Time by a double value.
     *
     * @param value double value to divide by
     * @return constexpr Time
     */
    constexpr Time operator/(double value) const {
        double totalSec = toSec() / value;

        int64_t sec = static_cast<int64_t>(totalSec);
        int64_t nsec =
            static_cast<int64_t>((totalSec - static_cast<double>(sec)) * 1'000'000'000LL);

        return Time(sec, nsec);
    }

    /**
     * @brief Overloading Operator / to divide a double value by a Time object.
     *
     * @param lhs double value to divide
     * @param rhs Time object to divide by
     * @return constexpr Time
     */
    friend constexpr Time operator/(double lhs, const Time& rhs) {
        return Time(lhs) / rhs;  // Convert lhs to Time and use Time / Time operator
    }

    /**
     * @brief Overloading Operator == to compare two Time objects for equality.
     *
     * @param other Time object to compare with
     * @return true if equal
     * @return false otherwise
     */
    constexpr bool operator==(const Time& other) const {
        return sec == other.sec && nsec == other.nsec;
    }

    /**
     * @brief Overloading Operator < to compare two Time objects.
     *
     * @param other Time object to compare with
     * @return true if less than
     * @return false otherwise
     */
    constexpr bool operator<(const Time& other) const {
        if (sec < other.sec)
            return true;
        if (sec == other.sec)
            return nsec < other.nsec;
        return false;
    }

    /**
     * @brief Overloading Operator <= to compare two Time objects.
     *
     * @param other Time object to compare with
     * @return true if less than or equal
     * @return false otherwise
     */
    constexpr bool operator<=(const Time& other) const { return *this < other || *this == other; }

    /**
     * @brief Overloading Operator > to compare two Time objects.
     *
     * @param other Time object to compare with
     * @return true if greater than
     * @return false otherwise
     */
    constexpr bool operator>(const Time& other) const { return !(*this <= other); }

    /**
     * @brief Overloading Operator >= to compare two Time objects.
     *
     * @param other Time object to compare with
     * @return true if greater than or equal
     * @return false otherwise
     */
    constexpr bool operator>=(const Time& other) const { return !(*this < other); }

    /**
     * @brief Overloading Operator < to compare a Time object with a double value.
     *
     * @param lhs Time object to compare
     * @param rhs double value to compare with
     * @return true if less than
     * @return false otherwise
     */
    constexpr friend bool operator<(const Time& lhs, double rhs) { return lhs.toSec() < rhs; }

    /**
     * @brief Overloading Operator <= to compare a Time object with a double value.
     *
     * @param lhs Time object to compare
     * @param rhs double value to compare with
     * @return true if less than or equal
     * @return false otherwise
     */
    constexpr friend bool operator<=(const Time& lhs, double rhs) { return lhs.toSec() <= rhs; }

    /**
     * @brief Overloading Operator > to compare a Time object with a double value.
     *
     * @param lhs Time object to compare
     * @param rhs double value to compare with
     * @return true if greater than
     * @return false otherwise
     */
    constexpr friend bool operator>(const Time& lhs, double rhs) { return lhs.toSec() > rhs; }

    /**
     * @brief Overloading Operator >= to compare a Time object with a double value.
     *
     * @param lhs Time object to compare
     * @param rhs double value to compare with
     * @return true if greater than or equal
     * @return false otherwise
     */
    constexpr friend bool operator>=(const Time& lhs, double rhs) { return lhs.toSec() >= rhs; }

    /**
     * @brief Overloading Operator == to compare a Time object with a double value.
     *
     * @param lhs Time object to compare
     * @param rhs double value to compare with
     * @return true if equal
     * @return false otherwise
     */
    constexpr friend bool operator==(const Time& lhs, double rhs) { return lhs.toSec() == rhs; }

    /**
     * @brief Overloading Operator != to compare a Time object with a double value.
     *
     * @param lhs Time object to compare
     * @param rhs double value to compare with
     * @return true if not equal
     * @return false otherwise
     */
    constexpr friend bool operator!=(const Time& lhs, double rhs) { return lhs.toSec() != rhs; }

    /**
     * @brief Overloading Operator < to compare a double value with a Time object.
     *
     * @param lhs double value to compare
     * @param rhs Time object to compare with
     * @return true if less than
     * @return false otherwise
     */
    constexpr friend bool operator<(double lhs, const Time& rhs) { return lhs < rhs.toSec(); }

    /**
     * @brief Overloading Operator <= to compare a double value with a Time object.
     *
     * @param lhs double value to compare
     * @param rhs Time object to compare with
     * @return true if less than or equal
     * @return false otherwise
     */
    constexpr friend bool operator<=(double lhs, const Time& rhs) { return lhs <= rhs.toSec(); }

    /**
     * @brief Overloading Operator > to compare a double value with a Time object.
     *
     * @param lhs double value to compare
     * @param rhs Time object to compare with
     * @return true if greater than
     * @return false otherwise
     */
    constexpr friend bool operator>(double lhs, const Time& rhs) { return lhs > rhs.toSec(); }

    /**
     * @brief Overloading Operator >= to compare a double value with a Time object.
     *
     * @param lhs double value to compare
     * @param rhs Time object to compare with
     * @return true if greater than or equal
     * @return false otherwise
     */
    constexpr friend bool operator>=(double lhs, const Time& rhs) { return lhs >= rhs.toSec(); }

    /**
     * @brief Overloading Operator == to compare a double value with a Time object.
     *
     * @param lhs double value to compare
     * @param rhs Time object to compare with
     * @return true if equal
     * @return false otherwise
     */
    constexpr friend bool operator==(double lhs, const Time& rhs) { return lhs == rhs.toSec(); }

    /**
     * @brief Overloading Operator != to compare a double value with a Time object.
     *
     * @param lhs double value to compare
     * @param rhs Time object to compare with
     * @return true if not equal
     * @return false otherwise
     */
    constexpr bool operator!=(const Time& other) const { return !(*this == other); }

    /**
     * @brief Overloading Operator != to compare a double value with a Time object.
     *
     * @param lhs double value to compare
     * @param rhs Time object to compare with
     * @return true if not equal
     * @return false otherwise
     */
    constexpr friend bool operator!=(double lhs, const Time& rhs) { return lhs != rhs.toSec(); }

    /**
     * @brief Convert a vector of Time objects to a vector of double values.
     *
     * @param time_vec Vector of Time objects to convert
     * @return std::vector<double> Vector of double values
     */
    static std::vector<double> convertTimeToDoubleVector(const std::vector<Time>& time_vec) {
        std::vector<double> double_vec;
        for (auto& time : time_vec) {
            double_vec.push_back(time.toSec());
        }
        return double_vec;
    }

    /**
     * @brief Convert a vector of double values to a vector of Time objects.
     *
     * @param double_vec Vector of double values to convert
     * @return std::vector<Time> Vector of Time objects
     */
    static std::vector<Time> convertDoubleToTimeVector(const std::vector<double>& double_vec) {
        std::vector<Time> time_vec;
        for (auto& value : double_vec) {
            time_vec.push_back(Time(value));
        }
        return time_vec;
    }

// Not in TwinCAT environment
#ifndef TC_VER

    /**
     * @brief Convert Time object to string representation.
     *
     * @return std::string String representation of the Time object in "sec.nsec" format.
     */
    std::string toString() const {
        std::ostringstream oss;
        oss << std::setfill('0') << std::setw(10) << sec << "." << std::setw(9) << nsec;
        return oss.str();
    }

    /**
     * @brief Overloading Operator << to print Time object to an output stream.
     *
     * @param os Output stream
     * @param time Time object to print
     * @return std::ostream& Reference to the output stream
     */
    friend std::ostream& operator<<(std::ostream& os, const Time& time) {
        os << time.toString();
        return os;
    }
#endif

#if defined(__GNUC__) || defined(__GNUG__)
} __attribute__((packed));
#else
};
#endif

/**
 * @brief Helper function to create a Time object from seconds.
 *
 * @param seconds Time in seconds as a double.
 * @return constexpr Time
 */
constexpr Time makeTime(double seconds) {
    return Time(seconds);
}

// Initialize constexpr variable using the helper function
inline constexpr Time Ts_default = makeTime(1e-3);

#if defined(_MSC_VER)
#pragma pack(pop)
#endif

}  // namespace orc

#endif  // TIME_H
