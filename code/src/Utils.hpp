/**
 *@file Utils.hpp
 *@author Erion Plaku 
 *@brief General utilities
 */

#ifndef UTILS_HPP_
#define UTILS_HPP_

#include <vector>
#define _USE_MATH_DEFINES
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <ctime>

/**
 *@name Geometry
 *@{
 */


/**
 *
 */
static inline
double EuclideanDistance(const double x1, const double y1, const double x2, const double y2){
    return sqrt(pow(x1-x2, 2.) + pow(y1-y2, 2.));
}

/**
 *@brief Determine if point <em>p</em> is inside polygon
 *@param p    2D point
 *@param n    number of polygon vertices
 *@param poly 2D polygon vertices
 */
bool IsPointInsidePolygon(const double p[], const int n, const double poly[]);

/**
 *@brief Determine if point <em>(x, y)</em> is inside polygon
 *@param x    x coordinate of 2D point
 *@param y    y coordinate of 2D point
 *@param n    number of polygon vertices
 *@param poly 2D polygon vertices
 */
static inline
bool IsPointInsidePolygon(const double x, const double y, const int n, const double poly[])
{
    const double p[2] = {x, y};        
    return IsPointInsidePolygon(p, n, poly);
}

/**
 * check if a point is inside circle by comparing its distance from circle center with radius of the circle
 */
static inline
bool IsPointInsideCircle(const double x, const double y, const double cx, const double cy, const double r){
    return (EuclideanDistance(x, y, cx, cy) <= r);
}

/**
 *@brief Return true iff segments <p0, p1> and <p2, p3> intersect
 *
 *@param p0 2D point
 *@param p1 2D point
 *@param p2 2D point
 *@param p3 2D point
 */
bool SegmentSegmentIntersection(const double p0[],
				const double p1[],
				const double p2[],
				const double p3[]);

/**
 *@brief Return true iff segment and polygon intersect
 *
 *@param p0 2D point
 *@param p1 2D point
 *@param n    number of polygon vertices
 *@param poly 2D polygon vertices
 */
bool SegmentPolygonIntersection(const double p0[],
				const double p1[],
				const int    n,
				const double poly[]);

static inline
bool SegmentPolygonIntersection(const double x1, const double y1, const double x2, const double y2, const int n, const double poly[]){
    double p0[2] = {x1,y1};
    double p1[2] = {x2,y2};
    return SegmentPolygonIntersection(p0, p1, n, poly);
}
/**
* This method checks whether given segment and circle intersect.
*/
bool SegmentCircleIntersection(const double x1, const double y1, const double x2, const double y2, const double cx, const double cy, const double r);
/**
* This method checks whether given polygon and circle intersect.
*/
bool CirclePolygonIntersection(const double cx, const double cy, const double r, const int n, const double poly[]);
/**
* This method checks whether 2 given circles intersect. Checks whether distance between centers is less than sum of radi or not
*/
static inline
bool CircleCircleIntersection(const double cx1, const double cy1, const double r1, const double cx2, const double cy2, const double r2){
    return (EuclideanDistance(cx1, cy1, cx2, cy2) <= (r1 + r2));
}


/**
 *@brief Return true iff polygons are in collision
 *
 *@param n    number of polygon vertices
 *@param poly 2D polygon vertices
 *@param n2    number of polygon vertices
 *@param poly2 2D polygon vertices
 */
bool PolygonPolygonCollision(const int n,
			     const double poly[],
			     const int n2,
			     const double poly2[]);
/**
 *@}
 */

/**
 *@name Timer
 *@{
 */

/**
 *@cond
 */

/**
 *@brief Check if operating system is Windows
 */
#if defined (WINDOWS) || defined (_WIN32) || defined (__WIN32__)
#define OS_WINDOWS 
#endif

/**
 *@endcond
 */

#ifdef OS_WINDOWS
#include <windows.h>
typedef long long Clock;	
#else
#include <sys/time.h>
#include <sys/resource.h>
typedef struct timeval Clock;	
#endif

/**
 *@brief Start the clock
 *@param c pointer to Clock
 */
static inline void ClockStart(Clock * const c)
{
#ifdef OS_WINDOWS
    QueryPerformanceCounter((LARGE_INTEGER *) c);
#else
    gettimeofday(c, NULL);  
#endif
}

/**
 *@brief Measure elapsed time in seconds since clock started
 *@param c pointer to Clock
 *@par Description
 *  To measure how much time code is taking do the following:
 *  Clock clk;
 *  ClockStart(&clk);
 *  //code you want to measure
 *  const double time = ClockElapsed(&clk);
 *  printf("code took %f seconds\n", time);
 */
static inline double ClockElapsed(Clock * const c)
{
#ifdef OS_WINDOWS
    long long end;
    long long freq;
    QueryPerformanceCounter((LARGE_INTEGER *) &end);
    QueryPerformanceFrequency((LARGE_INTEGER *) &freq);    
    return ((double) (end - (*c))) / freq; 
#else
    struct timeval end;
    gettimeofday(&end, NULL);      
    return  (end.tv_sec  - c->tv_sec ) +  0.000001 * (end.tv_usec - c->tv_usec);    
#endif	    
}

/**
 *@}
 */


/**
 *@name Pseudorandom Number Generation
 *@{
 */

/**
 *@cond
 */

/**
 *@brief Operating system specific definitions 
 */
#ifdef OS_WINDOWS
  #define srandom srand
  #define random rand
  #define RANDOM_MAX RAND_MAX
  static inline int long round(const double x)
  {
    return ((long) floor((x) + 0.5));
  }
#else
  #define RANDOM_MAX 2147483647
#endif

/**
 *@endcond
 */

/**
 *@brief Set the seed for the random number generator 
 *@param s seed value
 */
static inline void PseudoRandomSeed(const unsigned int s)
{
    srandom(s);    
}

/**
 *@brief Generate and set the seed for the random number generator 
 * 
 *@returns
 *  A pseudorandom number read from the file <em>/dev/urandom</em>, 
 *  which is then used to set the seed using <em>srandom</em>
 *  \n
 *  If file <em>/dev/urandom</em> does not exist, then
 *  <em>time(NULL)</em> is used to obtain the seed. 
 */
static inline unsigned int PseudoRandomSeed(void)
{
    FILE        *fp = fopen("/dev/urandom", "r");    
    unsigned int s;
    
    if(fp != NULL)
    {
	    fread(&s, sizeof(unsigned int), 1, fp);    
	    fclose(fp);    	        
    }
    else
	    s = (unsigned int) time(NULL);
    
    printf("using seed: %u\n", s);
    
    PseudoRandomSeed(s);
    
    return s;
}



/**
 *@brief Generate a real number uniformly at random from the interval
 *       <em>[0, 1]</em> 
 */
static inline double PseudoRandomUniformReal(void)
{
    return ((double) random()) / ((double) RANDOM_MAX);
}

/**
 *@brief Generate a real number uniformly at random from the interval
 *       <em>[min, max]</em> 
 *
 *@param min left boundary of the sampling interval
 *@param max right boundary of the sampling interval
 */
static inline double PseudoRandomUniformReal(const double min, const double max)
{
    return min + (max - min) * PseudoRandomUniformReal();
}

/**
 *@brief Generate an integer number uniformly at random from the
 *       interval <em>[min, max]</em> 
 *
 *@param min left boundary of the sampling interval
 *@param max right boundary of the sampling interval
 */
static inline long PseudoRandomUniformInteger(const long min, const long max)
{
    const long x = (long) round(min - 0.5 + (max - min + 1) * PseudoRandomUniformReal()); 
    return x > max ? max : (x < min ? min : x);
}


/**
 *@}
 */

#endif
