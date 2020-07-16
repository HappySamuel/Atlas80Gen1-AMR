#ifndef __FM_FILTER__H__
#define __FM_FILTER__H__

/// Author: Brice Rebsamen, Feb 2012

#include <stdexcept>

namespace fmutil
{

/// Implement a low pass filter
///
/// Jeong Hwan's implementation, improved.
class LowPassFilter
{
    double y; ///< The filtered value
    double t; ///< The time of the last input
    double tau; ///< The constant of the filter
    bool initialized; ///< Whether the filter has been initialized

public:

    LowPassFilter();

    /// Construct the filter
    /// @arg tau: the time constant of the filter
    LowPassFilter(double tau);

    /// Returns the current value of the filter. Throws a runtime_error if
    /// the filter has not been initialized.
    double value() const;

    /// Sets the current value of the filter (and time to 0)
    void value(double x);

    /// Reset the filter
    void reset();

    /// Add a value to the filter
    /// @arg t: the time of the value in seconds
    /// @arg x: the value to filter
    /// @return the filtered value
    double filter(double t, double x);

    /// Add a value to the filter
    /// @arg dt: the time since the last value in seconds
    /// @arg x: the value to filter
    /// @return the filtered value
    double filter_dt(double dt, double x);

};

} //namespace fmutil


#endif //__FM_FILTER__H__
