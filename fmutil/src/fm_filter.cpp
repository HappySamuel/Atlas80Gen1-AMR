#include <fmutil/fm_filter.h>

namespace fmutil
{


LowPassFilter::LowPassFilter()
{
    this->tau = 0;
    reset();
}


LowPassFilter::LowPassFilter(double tau)
{
    if( tau<=0 )
        throw std::invalid_argument("Filter's time constant (tau) must be > 0");
    this->tau = tau;
    reset();
}


double LowPassFilter::value() const
{
    if( ! this->initialized )
        throw std::runtime_error("Trying to get the value from an uninitialized filter");
    return this->y;
}


void LowPassFilter::value(double x)
{
    this->initialized = true;
    this->t = 0;
    this->y = x;
}


void LowPassFilter::reset()
{
    this->initialized = false;
}


double LowPassFilter::filter(double t, double x)
{
    if( this->tau==0 )
        throw std::runtime_error("Filter's time constant (tau) was not set");

    if( ! this->initialized )
    {
        this->initialized = true;
        this->y = x;
        this->t = t;
        return x;
    }

    double dt = t - this->t;
    this->t = t;

    if( dt > this->tau )
    {
        // filter has not been updated for a long time
        // need to reset it or value can jump very far away
        this->y = x;
        return x;
    }

    this->y += (x - this->y) * dt / this->tau;
    return this->y;
}


double LowPassFilter::filter_dt(double dt, double x)
{
    if( this->tau==0 )
        throw std::runtime_error("Filter's time constant (tau) was not set");

    if( ! this->initialized )
    {
        this->initialized = true;
        this->t = 0;
        this->y = x;
        return x;
    }

    return filter(this->t+dt, x);
}

}