/* ----------------------------------------------------------------------
 * Project:      CMSIS DSP Library
 * Title:        arm_conv_q15.c
 * Description:  Convolution of Q15 sequences
 *
 * $Date:        18. March 2019
 * $Revision:    V1.6.0
 *
 * Target Processor: Cortex-M cores
 * -------------------------------------------------------------------- */
/*
 * Copyright (C) 2010-2019 ARM Limited or its affiliates. All rights reserved.
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Licensed under the Apache License, Version 2.0 (the License); you may
 * not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 * www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an AS IS BASIS, WITHOUT
 * WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#include "arm_math.h"

/**
  @ingroup groupFilters
 */

/**
  @addtogroup Conv
  @{
 */

/**
  @brief         Convolution of Q15 sequences.
  @param[in]     pSrcA      points to the first input sequence
  @param[in]     srcALen    length of the first input sequence
  @param[in]     pSrcB      points to the second input sequence
  @param[in]     srcBLen    length of the second input sequence
  @param[out]    pDst       points to the location where the output result is written.  Length srcALen+srcBLen-1.
  @return        none

  @par           Scaling and Overflow Behavior
                   The function is implemented using a 64-bit internal accumulator.
                   Both inputs are in 1.15 format and multiplications yield a 2.30 result.
                   The 2.30 intermediate results are accumulated in a 64-bit accumulator in 34.30 format.
                   This approach provides 33 guard bits and there is no risk of overflow.
                   The 34.30 result is then truncated to 34.15 format by discarding the low 15 bits and then saturated to 1.15 format.

  @remark
                   Refer to \ref arm_conv_fast_q15() for a faster but less precise version of this function.
  @remark
                   Refer to \ref arm_conv_opt_q15() for a faster implementation of this function using scratch buffers.
 */
#if defined(ARM_MATH_MVEI)
#include "arm_helium_utils.h"
#include "arm_vec_filtering.h"

void arm_conv_q15(
  const q15_t * pSrcA,
        uint32_t srcALen,
  const q15_t * pSrcB,
        uint32_t srcBLen,
        q15_t * pDst)
{
    const q15_t    *pIn1 = pSrcA;     /* inputA pointer               */
    const q15_t    *pIn2 = pSrcB;     /* inputB pointer               */
    /*
     * Loop to perform MAC operations according to correlation equation
     */
    const q15_t    *pX;
    const q15_t    *pY;
    const q15_t    *pA;
    const q15_t    *pB;
    int32_t   i = 0U, j = 0;    /* loop counters */
    int32_t   block1, block2, block3;


    uint16x8_t decrIdxVec = vddupq_u16(7, 1);


    if (srcALen < srcBLen)
    {
        /*
         * Initialization to inputB pointer
         */
        pIn1 = pSrcB;
        /*
         * Initialization to the end of inputA pointer
         */
        pIn2 = pSrcA;
        /*
         * Swapping the lengths
         */
        j = srcALen;
        srcALen = srcBLen;
        srcBLen = j;
    }

    block1 = srcBLen - 1;
    block2 = srcALen - srcBLen + 1;
    block3 = srcBLen - 1;

    pA = pIn1;
    pB = pIn2 - 7;

    for (i = 0; i <= block1 - 2; i += 2)
    {
        uint32_t  count = i + 1;
        int64_t   acc0 = 0LL;
        int64_t   acc1 = 0LL;

        pX = pA;
        pY = pB;

        MVE_INTR_CONV_DUAL_INC_Y_INC_SIZE_Q15(acc0, acc1, pX, pY, count);
        *pDst++ = (q15_t) acc0;
        *pDst++ = (q15_t) acc1;
        pB += 2;
    }
    for (; i < block1; i++)
    {
        uint32_t  count = i + 1;
        int64_t   acc = 0LL;

        pX = pA;
        pY = pB;

        MVE_INTR_CONV_SINGLE_Q15(acc, pX, pY, count);
        *pDst++ = (q15_t) acc;
        pB++;
    }

    for (i = 0; i <= block2 - 4; i += 4)
    {
        uint32_t  count = srcBLen;
        int64_t   acc0 = 0LL;
        int64_t   acc1 = 0LL;
        int64_t   acc2 = 0LL;
        int64_t   acc3 = 0LL;

        pX = pA;
        pY = pB;
        /*
         * compute 4 accumulators per loop
         * size is fixed for all accumulators
         * X pointer is incrementing for successive accumulators
         */
        MVE_INTR_CONV_QUAD_INC_X_FIXED_SIZE_Q15(acc0, acc1, acc2, acc3, pX, pY, count);
        *pDst++ = (q15_t) acc0;
        *pDst++ = (q15_t) acc1;
        *pDst++ = (q15_t) acc2;
        *pDst++ = (q15_t) acc3;

        pA += 4;
    }
    for (; i <= block2 - 2; i += 2)
    {
        uint32_t  count = srcBLen;
        int64_t   acc0 = 0LL;
        int64_t   acc1 = 0LL;

        pX = pA;
        pY = pB;
        /*
         * compute 2 accumulators per loop
         * size is fixed for all accumulators
         * X pointer is incrementing for successive accumulators
         */
        MVE_INTR_CONV_DUAL_INC_X_FIXED_SIZE_Q15(acc0, acc1, pX, pY, count);
        *pDst++ = (q15_t) acc0;
        *pDst++ = (q15_t) acc1;

        pA += 2;
    }
    if (block2 & 1)
    {
        uint32_t  count = srcBLen;
        int64_t   acc = 0LL;

        pX = pA;
        pY = pB;

        MVE_INTR_CONV_SINGLE_Q15(acc, pX, pY, count);
        *pDst++ = (q15_t) acc;
        pA++;
    }

    for (i = block3; i >= 1; i -= 2)
    {
        uint32_t  count = i;
        int64_t   acc0 = 0LL;
        int64_t   acc1 = 0LL;

        pX = pA;
        pY = pB;

        MVE_INTR_CONV_DUAL_INC_X_DEC_SIZE_Q15(acc0, acc1, pX, pY, count);
        *pDst++ = (q15_t) acc0;
        *pDst++ = (q15_t) acc1;
        pA += 2;
    }
    for (; i >= 1; i--)
    {
        uint32_t  count = i;
        int64_t   acc = 0LL;

        pX = pA;
        pY = pB;

        MVE_INTR_CONV_SINGLE_Q15(acc, pX, pY, count);
        *pDst++ = (q15_t) acc;
        pA++;
    }
}
#else
void arm_conv_q15(
  const q15_t * pSrcA,
        uint32_t srcALen,
  const q15_t * pSrcB,
        uint32_t srcBLen,
        q15_t * pDst)
{

#if defined (ARM_MATH_DSP)

  const q15_t *pIn1;                                   /* InputA pointer */
  const q15_t *pIn2;                                   /* InputB pointer */
        q15_t *pOut = pDst;                            /* Output pointer */
        q63_t sum, acc0, acc1, acc2, acc3;             /* Accumulators */
  const q15_t *px;                                     /* Intermediate inputA pointer */
  const q15_t *py;                                     /* Intermediate inputB pointer */
  const q15_t *pSrc1, *pSrc2;                          /* Intermediate pointers */
        q31_t x0, x1, x2, x3, c0;                      /* Temporary input variables to hold state and coefficient values */
        uint32_t blockSize1, blockSize2, blockSize3;   /* Loop counters */
        uint32_t j, k, count, blkCnt;                  /* Loop counters */

  /* The algorithm implementation is based on the lengths of the inputs. */
  /* srcB is always made to slide across srcA. */
  /* So srcBLen is always considered as shorter or equal to srcALen */
  if (srcALen >= srcBLen)
  {
    /* Initialization of inputA pointer */
    pIn1 = pSrcA;

    /* Initialization of inputB pointer */
    pIn2 = pSrcB;
  }
  else
  {
    /* Initialization of inputA pointer */
    pIn1 = pSrcB;

    /* Initialization of inputB pointer */
    pIn2 = pSrcA;

    /* srcBLen is always considered as shorter or equal to srcALen */
    j = srcBLen;
    srcBLen = srcALen;
    srcALen = j;
  }

  /* conv(x,y) at n = x[n] * y[0] + x[n-1] * y[1] + x[n-2] * y[2] + ...+ x[n-N+1] * y[N -1] */
  /* The function is internally
   * divided into three stages according to the number of multiplications that has to be
   * taken place between inputA samples and inputB samples. In the first stage of the
   * algorithm, the multiplications increase by one for every iteration.
   * In the second stage of the algorithm, srcBLen number of multiplications are done.
   * In the third stage of the algorithm, the multiplications decrease by one
   * for every iteration. */

  /* The algorithm is implemented in three stages.
     The loop counters of each stage is initiated here. */
  blockSize1 = srcBLen - 1U;
  blockSize2 = srcALen - (srcBLen - 1U);

  /* --------------------------
   * Initializations of stage1
   * -------------------------*/

  /* sum = x[0] * y[0]
   * sum = x[0] * y[1] + x[1] * y[0]
   * ....
   * sum = x[0] * y[srcBlen - 1] + x[1] * y[srcBlen - 2] +...+ x[srcBLen - 1] * y[0]
   */

  /* In this stage the MAC operations are increased by 1 for every iteration.
     The count variable holds the number of MAC operations performed */
  count = 1U;

  /* Working pointer of inputA */
  px = pIn1;

  /* Working pointer of inputB */
  py = pIn2;

  /* ------------------------
   * Stage1 process
   * ----------------------*/

  /* For loop unrolling by 4, this stage is divided into two. */
  /* First part of this stage computes the MAC operations less than 4 */
  /* Second part of this stage computes the MAC operations greater than or equal to 4 */

  /* The first part of the stage starts here */
  while ((count < 4U) && (blockSize1 > 0U))
  {
    /* Accumulator is made zero for every iteration */
    sum = 0;

    /* Loop over number of MAC operations between
     * inputA samples and inputB samples */
    k = count;

    while (k > 0U)
    {
      /* Perform the multiply-accumulates */
      sum = __SMLALD(*px++, *py--, sum);

      /* Decrement loop counter */
      k--;
    }

    /* Store the result in the accumulator in the destination buffer. */
    *pOut++ = (q15_t) (__SSAT((sum >> 15), 16));

    /* Update the inputA and inputB pointers for next MAC calculation */
    py = pIn2 + count;
    px = pIn1;

    /* Increment MAC count */
    count++;

    /* Decrement loop counter */
    blockSize1--;
  }

  /* The second part of the stage starts here */
  /* The internal loop, over count, is unrolled by 4 */
  /* To, read the last two inputB samples using SIMD:
   * y[srcBLen] and y[srcBLen-1] coefficients, py is decremented by 1 */
  py = py - 1;

  while (blockSize1 > 0U)
  {
    /* Accumulator is made zero for every iteration */
    sum = 0;

    /* Apply loop unrolling and compute 4 MACs simultaneously. */
    k = count >> 2U;

    /* First part of the processing with loop unrolling.  Compute 4 MACs at a time.
     ** a second loop below computes MACs for the remaining 1 to 3 samples. */
    while (k > 0U)
    {
      /* Perform the multiply-accumulate */
      /* x[0], x[1] are multiplied with y[srcBLen - 1], y[srcBLen - 2] respectively */
      sum = __SMLALDX(read_q15x2_ia ((q15_t **) &px), read_q15x2_da ((q15_t **) &py), sum);
      /* x[2], x[3] are multiplied with y[srcBLen - 3], y[srcBLen - 4] respectively */
      sum = __SMLALDX(read_q15x2_ia ((q15_t **) &px), read_q15x2_da ((q15_t **) &py), sum);

      /* Decrement loop counter */
      k--;
    }

    /* For the next MAC operations, the pointer py is used without SIMD
     * So, py is incremented by 1 */
    py = py + 1U;

    /* If the count is not a multiple of 4, compute any remaining MACs here.
     ** No loop unrolling is used. */
    k = count % 0x4U;

    while (k > 0U)
    {
      /* Perform the multiply-accumulate */
      sum = __SMLALD(*px++, *py--, sum);

      /* Decrement loop counter */
      k--;
    }

    /* Store the result in the accumulator in the destination buffer. */
    *pOut++ = (q15_t) (__SSAT((sum >> 15), 16));

    /* Update the inputA and inputB pointers for next MAC calculation */
    py = pIn2 + (count - 1U);
    px = pIn1;

    /* Increment MAC count */
    count++;

    /* Decrement loop counter */
    blockSize1--;
  }

  /* --------------------------
   * Initializations of stage2
   * ------------------------*/

  /* sum = x[0] * y[srcBLen-1] + x[1] * y[srcBLen-2] +...+ x[srcBLen-1] * y[0]
   * sum = x[1] * y[srcBLen-1] + x[2] * y[srcBLen-2] +...+ x[srcBLen] * y[0]
   * ....
   * sum = x[srcALen-srcBLen-2] * y[srcBLen-1] + x[srcALen] * y[srcBLen-2] +...+ x[srcALen-1] * y[0]
   */

  /* Working pointer of inputA */
  px = pIn1;

  /* Working pointer of inputB */
  pSrc2 = pIn2 + (srcBLen - 1U);
  py = pSrc2;

  /* count is the index by which the pointer pIn1 to be incremented */
  count = 0U;

  /* -------------------
   * Stage2 process
   * ------------------*/

  /* Stage2 depends on srcBLen as in this stage srcBLen number of MACS are performed.
   * So, to loop unroll over blockSize2,
   * srcBLen should be greater than or equal to 4 */
  if (srcBLen >= 4U)
  {
    /* Loop unrolling: Compute 4 outputs at a time */
    blkCnt = blockSize2 >> 2U;

    while (blkCnt > 0U)
    {
      py = py - 1U;

      /* Set all accumulators to zero */
      acc0 = 0;
      acc1 = 0;
      acc2 = 0;
      acc3 = 0;

      /* read x[0], x[1] samples */
      x0 = read_q15x2 ((q15_t *) px);

      /* read x[1], x[2] samples */
      x1 = read_q15x2 ((q15_t *) px + 1);
      px += 2U;

      /* Apply loop unrolling and compute 4 MACs simultaneously. */
      k = srcBLen >> 2U;

      /* First part of the processing with loop unrolling.  Compute 4 MACs at a time.
       ** a second loop below computes MACs for the remaining 1 to 3 samples. */
      do
      {
        /* Read the last two inputB samples using SIMD:
         * y[srcBLen - 1] and y[srcBLen - 2] */
        c0 = read_q15x2_da ((q15_t **) &py);

        /* acc0 +=  x[0] * y[srcBLen - 1] + x[1] * y[srcBLen - 2] */
        acc0 = __SMLALDX(x0, c0, acc0);

        /* acc1 +=  x[1] * y[srcBLen - 1] + x[2] * y[srcBLen - 2] */
        acc1 = __SMLALDX(x1, c0, acc1);

        /* Read x[2], x[3] */
        x2 = read_q15x2 ((q15_t *) px);

        /* Read x[3], x[4] */
        x3 = read_q15x2 ((q15_t *) px + 1);

        /* acc2 +=  x[2] * y[srcBLen - 1] + x[3] * y[srcBLen - 2] */
        acc2 = __SMLALDX(x2, c0, acc2);

        /* acc3 +=  x[3] * y[srcBLen - 1] + x[4] * y[srcBLen - 2] */
        acc3 = __SMLALDX(x3, c0, acc3);

        /* Read y[srcBLen - 3] and y[srcBLen - 4] */
        c0 = read_q15x2_da ((q15_t **) &py);

        /* acc0 +=  x[2] * y[srcBLen - 3] + x[3] * y[srcBLen - 4] */
        acc0 = __SMLALDX(x2, c0, acc0);

        /* acc1 +=  x[3] * y[srcBLen - 3] + x[4] * y[srcBLen - 4] */
        acc1 = __SMLALDX(x3, c0, acc1);

        /* Read x[4], x[5] */
        x0 = read_q15x2 ((q15_t *) px + 2);

        /* Read x[5], x[6] */
        x1 = read_q15x2 ((q15_t *) px + 3);

        px += 4U;

        /* acc2 +=  x[4] * y[srcBLen - 3] + x[5] * y[srcBLen - 4] */
        acc2 = __SMLALDX(x0, c0, acc2);

        /* acc3 +=  x[5] * y[srcBLen - 3] + x[6] * y[srcBLen - 4] */
        acc3 = __SMLALDX(x1, c0, acc3);

      } while (--k);

      /* For the next MAC operations, SIMD is not used
       * So, the 16 bit pointer if inputB, py is updated */

      /* If the srcBLen is not a multiple of 4, compute any remaining MACs here.
       ** No loop unrolling is used. */
      k = srcBLen % 0x4U;

      if (k == 1U)
      {
        /* Read y[srcBLen - 5] */
        c0 = *(py + 1);
#ifdef  ARM_MATH_BIG_ENDIAN
        c0 = c0 << 16U;
#else
        c0 = c0 & 0x0000FFFF;
#endif /* #ifdef  ARM_MATH_BIG_ENDIAN */

        /* Read x[7] */
        x3 = read_q15x2 ((q15_t *) px);
        px++;

        /* Perform the multiply-accumulate */
        acc0 = __SMLALD(x0, c0, acc0);
        acc1 = __SMLALD(x1, c0, acc1);
        acc2 = __SMLALDX(x1, c0, acc2);
        acc3 = __SMLALDX(x3, c0, acc3);
      }

      if (k == 2U)
      {
        /* Read y[srcBLen - 5], y[srcBLen - 6] */
        c0 = read_q15x2 ((q15_t *) py);

        /* Read x[7], x[8] */
        x3 = read_q15x2 ((q15_t *) px);

        /* Read x[9] */
        x2 = read_q15x2 ((q15_t *) px + 1);
        px += 2U;

        /* Perform the multiply-accumulate */
        acc0 = __SMLALDX(x0, c0, acc0);
        acc1 = __SMLALDX(x1, c0, acc1);
        acc2 = __SMLALDX(x3, c0, acc2);
        acc3 = __SMLALDX(x2, c0, acc3);
      }

      if (k == 3U)
      {
        /* Read y[srcBLen - 5], y[srcBLen - 6] */
        c0 = read_q15x2 ((q15_t *) py);

        /* Read x[7], x[8] */
        x3 = read_q15x2 ((q15_t *) px);

        /* Read x[9] */
        x2 = read_q15x2 ((q15_t *) px + 1);

        /* Perform the multiply-accumulate */
        acc0 = __SMLALDX(x0, c0, acc0);
        acc1 = __SMLALDX(x1, c0, acc1);
        acc2 = __SMLALDX(x3, c0, acc2);
        acc3 = __SMLALDX(x2, c0, acc3);

        c0 = *(py-1);
#ifdef  ARM_MATH_BIG_ENDIAN
        c0 = c0 << 16U;
#else
        c0 = c0 & 0x0000FFFF;
#endif /* #ifdef  ARM_MATH_BIG_ENDIAN */

        /* Read x[10] */
        x3 =  read_q15x2 ((q15_t *) px + 2);
        px += 3U;

        /* Perform the multiply-accumulates */
        acc0 = __SMLALDX(x1, c0, acc0);
        acc1 = __SMLALD(x2, c0, acc1);
        acc2 = __SMLALDX(x2, c0, acc2);
        acc3 = __SMLALDX(x3, c0, acc3);
      }

      /* Store the result in the accumulator in the destination buffer. */
#ifndef  ARM_MATH_BIG_ENDIAN
      write_q15x2_ia (&pOut, __PKHBT(__SSAT((acc0 >> 15), 16), __SSAT((acc1 >> 15), 16), 16));
      write_q15x2_ia (&pOut, __PKHBT(__SSAT((acc2 >> 15), 16), __SSAT((acc3 >> 15), 16), 16));
#else
      write_q15x2_ia (&pOut, __PKHBT(__SSAT((acc1 >> 15), 16), __SSAT((acc0 >> 15), 16), 16));
      write_q15x2_ia (&pOut, __PKHBT(__SSAT((acc3 >> 15), 16), __SSAT((acc2 >> 15), 16), 16));
#endif /*      #ifndef  ARM_MATH_BIG_ENDIAN    */

      /* Increment the pointer pIn1 index, count by 4 */
      count += 4U;

      /* Update the inputA and inputB pointers for next MAC calculation */
      px = pIn1 + count;
      py = pSrc2;

      /* Decrement loop counter */
      blkCnt--;
    }

    /* If the blockSize2 is not a multiple of 4, compute any remaining output samples here.
     ** No loop unrolling is used. */
    blkCnt = blockSize2 % 0x4U;

    while (blkCnt > 0U)
    {
      /* Accumulator is made zero for every iteration */
      sum = 0;

      /* Apply loop unrolling and compute 4 MACs simultaneously. */
      k = srcBLen >> 2U;

      /* First part of the processing with loop unrolling.  Compute 4 MACs at a time.
       ** a second loop below computes MACs for the remaining 1 to 3 samples. */
      while (k > 0U)
      {
        /* Perform the multiply-accumulates */
        sum += (q63_t) ((q31_t) *px++ * *py--);
        sum += (q63_t) ((q31_t) *px++ * *py--);
        sum += (q63_t) ((q31_t) *px++ * *py--);
        sum += (q63_t) ((q31_t) *px++ * *py--);

        /* Decrement loop counter */
        k--;
      }

      /* If the srcBLen is not a multiple of 4, compute any remaining MACs here.
       ** No loop unrolling is used. */
      k = srcBLen % 0x4U;

      while (k > 0U)
      {
        /* Perform the multiply-accumulates */
        sum += (q63_t) ((q31_t) *px++ * *py--);

        /* Decrement the loop counter */
        k--;
      }

      /* Store the result in the accumulator in the destination buffer. */
      *pOut++ = (q15_t) (__SSAT(sum >> 15, 16));

      /* Increment the pointer pIn1 index, count by 1 */
      count++;

      /* Update the inputA and inputB pointers for next MAC calculation */
      px = pIn1 + count;
      py = pSrc2;

      /* Decrement the loop counter */
      blkCnt--;
    }
  }
  else
  {
    /* If the srcBLen is not a multiple of 4,
     * the blockSize2 loop cannot be unrolled by 4 */
    blkCnt = blockSize2;

    while (blkCnt > 0U)
    {
      /* Accumulator is made zero for every iteration */
      sum = 0;

      /* srcBLen number of MACS should be performed */
      k = srcBLen;

      while (k > 0U)
      {
        /* Perform the multiply-accumulate */
        sum += (q63_t) ((q31_t) *px++ * *py--);

        /* Decrement the loop counter */
        k--;
      }

      /* Store the result in the accumulator in the destination buffer. */
      *pOut++ = (q15_t) (__SSAT(sum >> 15, 16));

      /* Increment the MAC count */
      count++;

      /* Update the inputA and inputB pointers for next MAC calculation */
      px = pIn1 + count;
      py = pSrc2;

      /* Decrement the loop counter */
      blkCnt--;
    }
  }


  /* --------------------------
   * Initializations of stage3
   * -------------------------*/

  /* sum += x[srcALen-srcBLen+1] * y[srcBLen-1] + x[srcALen-srcBLen+2] * y[srcBLen-2] +...+ x[srcALen-1] * y[1]
   * sum += x[srcALen-srcBLen+2] * y[srcBLen-1] + x[srcALen-srcBLen+3] * y[srcBLen-2] +...+ x[srcALen-1] * y[2]
   * ....
   * sum +=  x[srcALen-2] * y[srcBLen-1] + x[srcALen-1] * y[srcBLen-2]
   * sum +=  x[srcALen-1] * y[srcBLen-1]
   */

  /* In this stage the MAC operations are decreased by 1 for every iteration.
     The blockSize3 variable holds the number of MAC operations performed */
  blockSize3 = srcBLen - 1U;

  /* Working pointer of inputA */
  pSrc1 = (pIn1 + srcALen) - (srcBLen - 1U);
  px = pSrc1;

  /* Working pointer of inputB */
  pSrc2 = pIn2 + (srcBLen - 1U);
  pIn2 = pSrc2 - 1U;
  py = pIn2;

  /* -------------------
   * Stage3 process
   * ------------------*/

  /* For loop unrolling by 4, this stage is divided into two. */
  /* First part of this stage computes the MAC operations greater than 4 */
  /* Second part of this stage computes the MAC operations less than or equal to 4 */

  /* The first part of the stage starts here */
  j = blockSize3 >> 2U;

  while ((j > 0U) && (blockSize3 > 0U))
  {
    /* Accumulator is made zero for every iteration */
    sum = 0;

    /* Apply loop unrolling and compute 4 MACs simultaneously. */
    k = blockSize3 >> 2U;

    /* First part of the processing with loop unrolling.  Compute 4 MACs at a time.
     ** a second loop below computes MACs for the remaining 1 to 3 samples. */
    while (k > 0U)
    {
      /* Perform the multiply-accumulate */
      /* x[srcALen - srcBLen + 1], x[srcALen - srcBLen + 2] are multiplied
       * with y[srcBLen - 1], y[srcBLen - 2] respectively */
      sum = __SMLALDX(read_q15x2_ia ((q15_t **) &px), read_q15x2_da ((q15_t **) &py), sum);
      /* x[srcALen - srcBLen + 3], x[srcALen - srcBLen + 4] are multiplied
       * with y[srcBLen - 3], y[srcBLen - 4] respectively */
      sum = __SMLALDX(read_q15x2_ia ((q15_t **) &px), read_q15x2_da ((q15_t **) &py), sum);

      /* Decrement loop counter */
      k--;
    }

    /* For the next MAC operations, the pointer py is used without SIMD
     * So, py is incremented by 1 */
    py = py + 1U;

    /* If the blockSize3 is not a multiple of 4, compute any remaining MACs here.
     ** No loop unrolling is used. */
    k = blockSize3 % 0x4U;

    while (k > 0U)
    {
      /* sum += x[srcALen - srcBLen + 5] * y[srcBLen - 5] */
      sum = __SMLALD(*px++, *py--, sum);

      /* Decrement loop counter */
      k--;
    }

    /* Store the result in the accumulator in the destination buffer. */
    *pOut++ = (q15_t) (__SSAT((sum >> 15), 16));

    /* Update the inputA and inputB pointers for next MAC calculation */
    px = ++pSrc1;
    py = pIn2;

    /* Decrement loop counter */
    blockSize3--;

    j--;
  }

  /* The second part of the stage starts here */
  /* SIMD is not used for the next MAC operations,
   * so pointer py is updated to read only one sample at a time */
  py = py + 1U;

  while (blockSize3 > 0U)
  {
    /* Accumulator is made zero for every iteration */
    sum = 0;

    /* Apply loop unrolling and compute 4 MACs simultaneously. */
    k = blockSize3;

    while (k > 0U)
    {
      /* Perform the multiply-accumulates */
      /* sum +=  x[srcALen-1] * y[srcBLen-1] */
      sum = __SMLALD(*px++, *py--, sum);

      /* Decrement loop counter */
      k--;
    }

    /* Store the result in the accumulator in the destination buffer. */
    *pOut++ = (q15_t) (__SSAT((sum >> 15), 16));

    /* Update the inputA and inputB pointers for next MAC calculation */
    px = ++pSrc1;
    py = pSrc2;

    /* Decrement loop counter */
    blockSize3--;
  }

#else /* #if defined (ARM_MATH_DSP) */

  const q15_t *pIn1 = pSrcA;                           /* InputA pointer */
  const q15_t *pIn2 = pSrcB;                           /* InputB pointer */
        q63_t sum;                                     /* Accumulator */
        uint32_t i, j;                                 /* Loop counters */

  /* Loop to calculate convolution for output length number of values */
  for (i = 0; i < (srcALen + srcBLen - 1); i++)
  {
    /* Initialize sum with zero to carry on MAC operations */
    sum = 0;

    /* Loop to perform MAC operations according to convolution equation */
    for (j = 0U; j <= i; j++)
    {
      /* Check the array limitations */
      if (((i - j) < srcBLen) && (j < srcALen))
      {
        /* z[i] += x[i-j] * y[j] */
        sum += ((q31_t) pIn1[j] * pIn2[i - j]);
      }
    }

    /* Store the output in the destination buffer */
    pDst[i] = (q15_t) __SSAT((sum >> 15U), 16U);
  }

#endif /* #if defined (ARM_MATH_DSP) */

}
#endif /* defined(ARM_MATH_MVEI) */

/**
  @} end of Conv group
 */
