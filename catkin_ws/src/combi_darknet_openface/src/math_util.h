#ifndef COMBI_DARKNET_OPENFACE_MATH_UTIL_H
#define COMBI_DARKNET_OPENFACE_MATH_UTIL_H
#include <cmath>
#include <opencv2/core/core.hpp>

namespace math_util{
    const double PI = 3.14159265358979323846;

    /** round the value to the specified number of digits
     * 
     * \param value the value
     * \param num_digit the specified number of digits
     */
    template<typename VALUE_T, typename SIZE_T>
    inline VALUE_T round(VALUE_T value, SIZE_T num_digit){
        return static_cast<double>(static_cast<int>(value * std::pow(10.0, num_digit) + 0.5)) * std::pow(10.0, -num_digit);
    }

    template<typename T>
    inline T radToDeg(T value){
        return value * 180 / PI;
    }

    template<typename T>
    inline T degToRad(T value){
        return value * PI / 180;
    }

    inline double atan2(const cv::Point& p)
    {
        return std::atan2(p.y, p.x);
    }
};
#endif
