#ifndef __FMUTIL_MATH_H__
#define __FMUTIL_MATH_H__

#include <cmath>

namespace fmutil
{

/** Bounds the argument between a max and a minimum
 *
 * @param m the lower bound
 * @param x the value to be bounded
 * @param M the upper bound
 * @return m if x is lower than m, M if x is greater than M, x otherwise.
 */
template<typename T>
T bound(T m, T x, T M)
{
    return x<m ? m : x>M ? M : x;
}

/** Bounds the argument between -m and m
 *
 * @param x the value to be bounded
 * @param b the upper bound
 * @return -m if x is lower than -m, m if x is greater than m, x otherwise.
 */
template<typename T>
T symbound(T x, T m)
{
  if(m <0) m = -m;
    return x<-m ? -m : x>m ? m : x;
}

/** Bounds the argument between a max and a minimum.
 *
 * @param m the lower bound
 * @param x the value to be bounded
 * @param M the upper bound
 * @return -m if x is lower than -m, m if x is greater than m, x otherwise.
 */
template<typename T>
T bound(T m, T *x, T M)
{
    *x = *x<m ? m : *x>M ? M : *x;
    return *x;
}

/** Bounds the argument between 0 and a max/minimum
 * 
 * @param m the lower/upper bound
 * @param x the value to be bounded
 * @return -m if m given is smaller than zero and x < -m, 
 * 0 if m given is larger than zero and x < 0
 */
 template<typename T>
 T zerobound(T x, T m)
 {
   if(m > 0) return x<0 ? 0 : x>m ? m : x;
   else return x<m ? m : x>0 ? 0 : x;
 }
 
/** Bounds the value between -m and m
 *
 * @param x the value to be bounded
 * @param b the upper bound
 * @return -m if x is lower than -m, m if x is greater than m, x otherwise.
 */
template<typename T>
T symbound(T *x, T m)
{
  if(m<0) m=-m;
    *x = *x<-m ? -m : *x>m ? m : *x;
    return *x;
}

/// Returns the sign of the argument
double sign(double a);

/// Computes the corresponding angle on ]-180,180].
double angMod180(double ang);

/// Computes the corresponding angle on [0,360[.
double angMod360(double ang);

/// Computes the corresponding angle on ]-pi,pi].
double angModPI(double ang);

/// Computes the corresponding angle on ]-pi/2,pi/2].
double angModPI2(double ang);

/// Computes the corresponding angle on ]0,2*pi].
double angMod2PI(double ang);

/**
 * Computes the angular distance between 2 angles: fabs(angModPI(a-b))
 * @param a in radians
 * @param b in radians
 * @return fabs(angModPI(a-b))
 */
double angDist(double a, double b);

///converts degrees to radians
double d2r(double ang);

///converts radians to degrees
double r2d(double ang);

/// returns the euclidian norm (magnitude) of vector [a b (c)]
/// as sqrt(a^2+b^2+c^2)
double mag(double a, double b, double c=0);

/// Saturation function: returns -1 if x < -s, 1 if x > s, x/s otherwise.
double sat(double x, double s);

/// returns the euclidian distance between points (x1,y1) and (x2,y2)
double distance(double x1, double y1, double x2, double y2);

/// returns the euclidian distance between points (p1.x,p1.y) and (p2.x,p2.y)
template<class T>
double distance(const T & p1, const T & p2)
{return mag(p1.x-p2.x, p1.y-p2.y);}

/// returns the angle of the vector from (x1,y1) to (x2,y2)
/// i.e. atan2f(y1-y2,x1-x2)
double angle(double x1, double y1, double x2, double y2);

/// returns the minimum of the 3 numbers. Can also be used with only 2 numbers.
double min(double a, double b, double c=INFINITY);

/// returns the maximum of the 3 numbers. Can also be used with only 2 numbers.
double max(double a, double b, double c=-INFINITY);

/// checks whether x is within min(a,b) and max(a,b)
bool isWithin(double x, double a, double b);

} //namespace fmutil

#endif //__FMUTIL_MATH_H__
