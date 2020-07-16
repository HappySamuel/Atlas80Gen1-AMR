#include <stdlib.h>
#include <stdio.h>
#include <assert.h>
#include <errno.h>
#include <stdarg.h> //For variable number of arguments
#include <string.h>
#include <math.h>

#include <string>
#include <iostream>

#include <fmutil/fm_math.h>

namespace fmutil
{

double sign(double a)
{
    return a>0.0 ? 1.0 : a<0.0 ? -1.0 : 0.0;
}

double angMod180(double ang)
{
  double a = ang;
  while( a <= -180 ) a+=360.0;
  while( a > 180 )   a-=360.0;
  return a;
}

double angMod360(double ang)
{
  double a = ang;
  while( a < 0 ) a+=360.0;
  while( a >= 360 ) a-=360.0;
  return a;
}

double angModPI(double ang)
{
  double a = ang;
  while( a <= -M_PI ) a+=2.0*M_PI;
  while( a > M_PI )   a-=2.0*M_PI;
  return a;
}

double angModPI2(double ang)
{
  double a = ang;
  while( a <= -M_PI_2 ) a+=M_PI;
  while( a > M_PI_2 )   a-=M_PI;
  return a;
}

double angMod2PI(double ang)
{
  double a = ang;
  while( a <= 0 ) a+=2.0*M_PI;
  while( a > 2*M_PI ) a-=2.0*M_PI;
  return a;
}

double angDist(double a, double b)
{
  return fabs(angModPI(a-b));
}

double d2r(double ang)
{
  return ang/180.0*M_PI;
}

double r2d(double ang)
{
  return ang/M_PI*180.0;
}

double mag(double a, double b, double c)
{
  return sqrt(a*a + b*b + c*c);
}

double distance(double x1, double y1, double x2, double y2)
{
  return mag(x1-x2, y1-y2);
}

double angle(double x1, double y1, double x2, double y2)
{
  return atan2(y1-y2, x1-x2);
}

double min(double a, double b, double c)
{
  return a<b ? (a<c ? a : c) : (b<c ? b : c);
}

double max(double a, double b, double c)
{
  return a>b ? (a>c ? a : c) : (b>c ? b : c);
}

bool isWithin(double x, double a, double b)
{
  return min(a,b)<x && x<max(a,b);
}

double sat(double x, double s)
{
    return x<-s ? -1.0 : x>s ? 1.0 : x/s;
}

} //namespace fmutil
