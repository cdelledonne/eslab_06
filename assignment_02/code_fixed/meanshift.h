#ifndef MEANSHIFT_H
#define MEANSHIFT_H
#include <iostream>
#include <math.h>
#include "opencv2/core/core.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include "opencv2/highgui/highgui.hpp"


#define B                       16

#define INT2FIXED(N)            ( ((int32_t)N) << B )
#define FIXED2INT(N)            ( (int32_t)(N >> B) )

#define FLOAT2FIXED(N)          ( (int32_t)(N * (1 << B)) )
#define FIXED2FLOAT(N)          ( ((float)N) / (1 << B) )

#define MULT(X,Y)               ( (int32_t)(((int64_t)X * (int64_t)Y) >> B) )
#define DIV(X,Y)                ( (int32_t)((((int64_t)X << B) + (Y/2)) / Y) )

#define MULT64(X,Y)             ( (int64_t)(((int64_t)X * (int64_t)Y) >> B) )
#define DIV64(X,Y)              ( (int64_t)((((int64_t)X << B) + (Y/2)) / Y) )

#define FIXEDSQRT(X)            ( ((int64_t)sqrt((int64_t)X)) << (B/2) )


#define BITSPERLONG 32
#define TOP2BITS(x) ((x & (3L << (BITSPERLONG-2))) >> (BITSPERLONG-2))

struct int_sqrt {
    unsigned sqrt, frac;
};

/* usqrt:
    ENTRY x: unsigned long
    EXIT  returns floor(sqrt(x) * pow(2, BITSPERLONG/2))

    Since the square root never uses more than half the bits
    of the input, we use the other half of the bits to contain
    extra bits of precision after the binary point.

    EXAMPLE
        suppose BITSPERLONG = 32
        then    usqrt(144) = 786432 = 12 * 65536
                usqrt(32) = 370727 = 5.66 * 65536

    NOTES
        (1) change BITSPERLONG to BITSPERLONG/2 if you do not want
            the answer scaled.  Indeed, if you want n bits of
            precision after the binary point, use BITSPERLONG/2+n.
            The code assumes that BITSPERLONG is even.
        (2) This is really better off being written in assembly.
            The line marked below is really a "arithmetic shift left"
            on the double-long value with r in the upper half
            and x in the lower half.  This operation is typically
            expressible in only one or two assembly instructions.
        (3) Unrolling this loop is probably not a bad idea.

    ALGORITHM
        The calculations are the base-two analogue of the square
        root algorithm we all learned in grammar school.  Since we're
        in base 2, there is only one nontrivial trial multiplier.

        Notice that absolutely no multiplications or divisions are performed.
        This means it'll be fast on a wide range of processors.
*/

// void usqrt(unsigned long x, struct int_sqrt *q)
// {
//       unsigned long a = 0L;                   /* accumulator      */
//       unsigned long r = 0L;                   /* remainder        */
//       unsigned long e = 0L;                   /* trial product    */
// 
//       int i;
// 
//       for (i = 0; i < BITSPERLONG; i++)   /* NOTE 1 */
//       {
//             r = (r << 2) + TOP2BITS(x); x <<= 2; /* NOTE 2 */
//             a <<= 1;
//             e = (a << 1) + 1;
//             if (r >= e)
//             {
//                   r -= e;
//                   a++;
//             }
//       }
//       memcpy(q, &a, sizeof(long));
// }


#define PI 3.1415926
class MeanShift
{
 private:
    float bin_width;
    cv::Mat target_model;
    cv::Rect target_Region;

    cv::Mat kernel;
    float kernelSum;

    struct config{
        int num_bins;
        int piexl_range;
        int MaxIter;
    }cfg;

public:
    MeanShift();
    void Init_target_frame(const cv::Mat &frame,const cv::Rect &rect);
    uint32_t Epanechnikov_kernel(cv::Mat &kernel);
    cv::Mat pdf_representation(const cv::Mat &frame,const cv::Rect &rect);
    void CalWeight(const cv::Mat &bgr_planes, cv::Mat &target_model, cv::Mat &target_candidate, cv::Rect &rec);
    cv::Mat CalWeight_opt(const cv::Mat &frame, cv::Mat &target_model, cv::Mat &target_candidate, cv::Rect &rec);
    cv::Rect track(const cv::Mat &next_frame, const cv::Mat &mult);

    int count;

#ifdef TIMING
    int64_t splitCount;
    int64_t pdfCount;
    int64_t weightCount;
    int64_t loopCount;
    int64_t weightLoopCount;
    int64_t accCount;
    int64_t sqrtCount;
#endif
};

#endif // MEANSHIFT_H
