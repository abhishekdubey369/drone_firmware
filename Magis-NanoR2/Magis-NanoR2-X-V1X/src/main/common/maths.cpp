/*
 * This file is part of Cleanflight and Magis.
 *
 * Cleanflight and Magis are free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * Cleanflight and Magis are distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this software.  If not, see <http://www.gnu.org/licenses/>.
 */

#include <stdint.h>
#include <math.h>

#include "axis.h"
#include "maths.h"

// http://lolengine.net/blog/2011/12/21/better-function-approximations
// Chebyshev http://stackoverflow.com/questions/345085/how-do-trigonometric-functions-work/345117#345117
// Thanks for ledvinap for making such accuracy possible! See: https://github.com/cleanflight/cleanflight/issues/940#issuecomment-110323384
// https://github.com/Crashpilot1000/HarakiriWebstore1/blob/master/src/mw.c#L1235
#if defined(FAST_TRIGONOMETRY) || defined(EVEN_FASTER_TRIGONOMETRY)
#if defined(EVEN_FASTER_TRIGONOMETRY)
#define sinPolyCoef3 -1.666568107e-1f
#define sinPolyCoef5  8.312366210e-3f
#define sinPolyCoef7 -1.849218155e-4f
#define sinPolyCoef9  0
#else
#define sinPolyCoef3 -1.666665710e-1f                                          // Double: -1.666665709650470145824129400050267289858e-1
#define sinPolyCoef5  8.333017292e-3f                                          // Double:  8.333017291562218127986291618761571373087e-3
#define sinPolyCoef7 -1.980661520e-4f                                          // Double: -1.980661520135080504411629636078917643846e-4
#define sinPolyCoef9  2.600054768e-6f                                          // Double:  2.600054767890361277123254766503271638682e-6
#endif


float sin_approx(float x)
{
    int32_t xint = x;
    if (xint < -32 || xint > 32)
        return 0.0f;                               // Stop here on error input (5 * 360 Deg)
    while (x > M_PIf)
        x -= (2.0f * M_PIf);                                 // always wrap input angle to -PI..PI
    while (x < -M_PIf)
        x += (2.0f * M_PIf);
    if (x > (0.5f * M_PIf))
        x = (0.5f * M_PIf) - (x - (0.5f * M_PIf));   // We just pick -90..+90 Degree
    else if (x < -(0.5f * M_PIf))
        x = -(0.5f * M_PIf) - ((0.5f * M_PIf) + x);
    float x2 = x * x;
    return x + x * x2 * (sinPolyCoef3 + x2 * (sinPolyCoef5 + x2 * (sinPolyCoef7 + x2 * sinPolyCoef9)));
}  // AdityaLakshmi // This is done to improve accuracy
// https://en.wikipedia.org/wiki/Trigonometric_functions and numerical recipes

float cos_approx(float x)
{
    return sin_approx(x + (0.5f * M_PIf));
}
#endif

int32_t applyDeadband(int32_t value, int32_t deadband)
{
    if (ABS(value) < deadband) {
        value = 0;
    } else if (value > 0) {
        value -= deadband;
    } else if (value < 0) {
        value += deadband;
    }
    return value;
}

int constrain(int amt, int low, int high)
{
    if (amt < low)
        return low;
    else if (amt > high)
        return high;
    else
        return amt;
}

float constrainf(float amt, float low, float high)
{
    if (amt < low)
        return low;
    else if (amt > high)
        return high;
    else
        return amt;
}

void devClear(stdev_t *dev)
{
    dev->m_n = 0;
}

void devPush(stdev_t *dev, float x)
{
    dev->m_n++;
    if (dev->m_n == 1) {
        dev->m_oldM = dev->m_newM = x;
        dev->m_oldS = 0.0f;
    } else {
        dev->m_newM = dev->m_oldM + (x - dev->m_oldM) / dev->m_n;
        dev->m_newS = dev->m_oldS + (x - dev->m_oldM) * (x - dev->m_newM);
        dev->m_oldM = dev->m_newM;
        dev->m_oldS = dev->m_newS;
    }
}

float devVariance(stdev_t *dev)
{
    return ((dev->m_n > 1) ? dev->m_newS / (dev->m_n - 1) : 0.0f);
}

float devStandardDeviation(stdev_t *dev)
{
    return sqrtf(devVariance(dev));
}

float degreesToRadians(int16_t degrees)
{
    return degrees * RAD;
}

int scaleRange(int x, int srcMin, int srcMax, int destMin, int destMax)
{
    long int a = ((long int) destMax - (long int) destMin) * ((long int) x - (long int) srcMin);
    long int b = (long int) srcMax - (long int) srcMin;
    return ((a / b) - (destMax - destMin)) + destMax;
}

// Normalize a vector
void normalizeV(struct fp_vector *src, struct fp_vector *dest)
{
    float length;

    length = sqrtf(src->X * src->X + src->Y * src->Y + src->Z * src->Z);
    if (length != 0) {
        dest->X = src->X / length;
        dest->Y = src->Y / length;
        dest->Z = src->Z / length;
    }
}

void buildRotationMatrix(fp_angles_t *delta, float matrix[3][3])
{
    float cosx, sinx, cosy, siny, cosz, sinz;
    float coszcosx, sinzcosx, coszsinx, sinzsinx;

    cosx = cos_approx(delta->angles.roll);
    sinx = sin_approx(delta->angles.roll);
    cosy = cos_approx(delta->angles.pitch);
    siny = sin_approx(delta->angles.pitch);
    cosz = cos_approx(delta->angles.yaw);
    sinz = sin_approx(delta->angles.yaw);

    coszcosx = cosz * cosx;
    sinzcosx = sinz * cosx;
    coszsinx = sinx * cosz;
    sinzsinx = sinx * sinz;

    matrix[0][X] = cosz * cosy;
    matrix[0][Y] = -cosy * sinz;
    matrix[0][Z] = siny;
    matrix[1][X] = sinzcosx + (coszsinx * siny);
    matrix[1][Y] = coszcosx - (sinzsinx * siny);
    matrix[1][Z] = -sinx * cosy;
    matrix[2][X] = (sinzsinx) - (coszcosx * siny);
    matrix[2][Y] = (coszsinx) + (sinzcosx * siny);
    matrix[2][Z] = cosy * cosx;





}

// Rotate a vector *v by the euler angles defined by the 3-vector *delta.
void rotateV(struct fp_vector *v, fp_angles_t *delta)
{
    struct fp_vector v_tmp = *v;

    float matrix[3][3];

    buildRotationMatrix(delta, matrix);

    v->X = v_tmp.X * matrix[0][X] + v_tmp.Y * matrix[1][X] + v_tmp.Z * matrix[2][X];
    v->Y = v_tmp.X * matrix[0][Y] + v_tmp.Y * matrix[1][Y] + v_tmp.Z * matrix[2][Y];
    v->Z = v_tmp.X * matrix[0][Z] + v_tmp.Y * matrix[1][Z] + v_tmp.Z * matrix[2][Z];
}

// Quick median filter implementation
// (c) N. Devillard - 1998
// http://ndevilla.free.fr/median/median.pdf
#define QMF_SORT(a,b) { if ((a)>(b)) QMF_SWAP((a),(b)); }
#define QMF_SWAP(a,b) { int32_t temp=(a);(a)=(b);(b)=temp; }
#define QMF_COPY(p,v,n) { int32_t i; for (i=0; i<n; i++) p[i]=v[i]; }

int32_t quickMedianFilter3(int32_t * v)
{
    int32_t p[3];
    QMF_COPY(p, v, 3);

    QMF_SORT(p[0], p[1]);
    QMF_SORT(p[1], p[2]);
    QMF_SORT(p[0], p[1]);
    return p[1];
}

int32_t quickMedianFilter5(int32_t * v)
{
    int32_t p[5];
    QMF_COPY(p, v, 5);

    QMF_SORT(p[0], p[1]);
    QMF_SORT(p[3], p[4]);
    QMF_SORT(p[0], p[3]);
    QMF_SORT(p[1], p[4]);
    QMF_SORT(p[1], p[2]);
    QMF_SORT(p[2], p[3]);
    QMF_SORT(p[1], p[2]);
    return p[2];
}

int32_t quickMedianFilter7(int32_t * v)
{
    int32_t p[7];
    QMF_COPY(p, v, 7);

    QMF_SORT(p[0], p[5]);
    QMF_SORT(p[0], p[3]);
    QMF_SORT(p[1], p[6]);
    QMF_SORT(p[2], p[4]);
    QMF_SORT(p[0], p[1]);
    QMF_SORT(p[3], p[5]);
    QMF_SORT(p[2], p[6]);
    QMF_SORT(p[2], p[3]);
    QMF_SORT(p[3], p[6]);
    QMF_SORT(p[4], p[5]);
    QMF_SORT(p[1], p[4]);
    QMF_SORT(p[1], p[3]);
    QMF_SORT(p[3], p[4]);
    return p[3];
}

int32_t quickMedianFilter9(int32_t * v)
{
    int32_t p[9];
    QMF_COPY(p, v, 9);

    QMF_SORT(p[1], p[2]);
    QMF_SORT(p[4], p[5]);
    QMF_SORT(p[7], p[8]);
    QMF_SORT(p[0], p[1]);
    QMF_SORT(p[3], p[4]);
    QMF_SORT(p[6], p[7]);
    QMF_SORT(p[1], p[2]);
    QMF_SORT(p[4], p[5]);
    QMF_SORT(p[7], p[8]);
    QMF_SORT(p[0], p[3]);
    QMF_SORT(p[5], p[8]);
    QMF_SORT(p[4], p[7]);
    QMF_SORT(p[3], p[6]);
    QMF_SORT(p[1], p[4]);
    QMF_SORT(p[2], p[5]);
    QMF_SORT(p[4], p[7]);
    QMF_SORT(p[4], p[2]);
    QMF_SORT(p[6], p[4]);
    QMF_SORT(p[4], p[2]);
    return p[4];
}

void arraySubInt32(int32_t *dest, int32_t *array1, int32_t *array2, int count)
{
    for (int i = 0; i < count; i++) {
        dest[i] = array1[i] - array2[i];
    }
}
uint16_t leastSignificantBit(uint16_t myInt)
{
    uint16_t mask = 1 << 0;
    for (uint16_t bitIndex = 0; bitIndex <= 8; bitIndex++) {
        if ((myInt & mask) != 0) {
            return bitIndex;
        }
        mask <<= 1;
    }
    return -1;
}





/* used for mostly dcm */
// a varient of asin() that checks the input ranges and ensures a
// valid angle as output. If nan is given as input then zero is
// returned.
float safe_asin(float v)
{
    if (isnan(v)) {
        return 0.0;
    }
    if (v >= 1.0f) {
        return M_PIf/2;
    }
    if (v <= -1.0f) {
        return -M_PIf/2;
    }
    return asinf(v);
}

// degrees -> radians
float radians(float deg) {
    return deg * DEG_TO_RAD;
}

// radians -> degrees
float degrees(float rad) {
    return rad * RAD_TO_DEG;
}


bool is_positive(float val)
{
    return val>=FLT_EPSILON;

}
