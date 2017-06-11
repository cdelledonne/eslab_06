/*
 * Based on paper "Kernel-Based Object Tracking"
 * you can find all the formula in the paper
 */
#include <stdint.h>
#include <stdlib.h>
#include <stdio.h>
#include "meanshift.h"
#include "arm_neon.h"
#ifdef DSP /*******************************************************************/
#include "pool_notify.h"
#endif /* DSP *****************************************************************/

#ifdef DSP /*******************************************************************/

/* Rows of the weight matrix to be computed on the DSP */
#define ROWS_FOR_DSP            8

#else

#define ROWS_FOR_DSP            0

#endif /* DSP *****************************************************************/

/* Horizontal shift for the tracking circle area = 0.5 * (columns - rows) */
#define H_SHIFT                 14


MeanShift::MeanShift()
{
    cfg.MaxIter = 8;
    cfg.num_bins = 16;
    cfg.piexl_range = 256;
    bin_width = cfg.piexl_range / cfg.num_bins;
    bin_width_log2 = log2(bin_width);

#ifdef TIMING /****************************************************************/
    splitCount       = 0;
    pdfCount         = 0;
    weightCount      = 0;
    loopCount        = 0;
    weightLoopCount  = 0;
    sqrtCount        = 0;
    writeBufferCount = 0;
    idleCount        = 0;
#endif /* TIMING **************************************************************/
}

void  MeanShift::Init_target_frame(const cv::Mat &frame, const cv::Rect &rect)
{
    target_Region = rect;
    target_model = pdf_representation(frame, target_Region);
}


cv::Mat MeanShift::pdf_representation(const cv::Mat &frame, const cv::Rect &rect)
{
    /* 5 of the 8 initial rows were unused, pdf_model has 3 rows now */
    cv::Mat pdf_model(3, 16, CV_32F, cv::Scalar(1e-10));

    cv::Vec3b curr_pixel_value;
    cv::Vec3b bin_value;

    /**
     * Old version: the pdf_model elements would have different weight depending
     * on the current pixel position (greater weights towards the centre of
     * the ellipse computed in MeanShift::Epanechnikov_kernel).
     *
     * Current version: maximum weight is assigned to one single pixel in the
     * centre of the rectangle (degenerate case of smaller ellipse).
     */
    curr_pixel_value = frame.at<cv::Vec3b>(rect.y + rect.height/2, rect.x + rect.width/2);
    bin_value[0] = (curr_pixel_value[0] >> bin_width_log2);
    bin_value[1] = (curr_pixel_value[1] >> bin_width_log2);
    bin_value[2] = (curr_pixel_value[2] >> bin_width_log2);

#ifdef DSP /*******************************************************************/

    /**
     * Send a notification to the DSP containing the column indices of the
     * pdf_model elements to be assigned maximum probability, with different
     * notification codes depending on the model to be sent (target_model or
     * target_candidate).
     *
     * The 3 indices are 8-bit numbers, so they can be concatenated to fit
     * into the 3 least significant bytes of a single 32-bit notification. 
     * The MSB is used for the notification code.
     */
    static int send_model = 1;
    Uint32 notification = bin_value[0] | (bin_value[1] << 8) | (bin_value[2] << 16);
    if (send_model) {
        notification |= NOTIF_PDF_MODEL;
        notify_DSP(notification);
        send_model = 0;
    }
    else {
        notification |= NOTIF_PDF_CANDIDATE;
        notify_DSP(notification);
    }

#endif /* DSP *****************************************************************/
    
    pdf_model.at<float>(0,bin_value[0]) += 1;
    pdf_model.at<float>(1,bin_value[1]) += 1;
    pdf_model.at<float>(2,bin_value[2]) += 1;

    return pdf_model;
}

cv::Mat MeanShift::CalWeight(const cv::Mat &window, cv::Mat &target_model, 
                    cv::Mat &target_candidate, cv::Rect &rec)
{
    int rows = window.rows;
    int cols = window.cols + 2; // multiple of 4 to use NEON vectorisation

    cv::Mat weight(rows, cols, CV_32F, cv::Scalar(1.0000));
    std::vector<cv::Mat> bgr_planes;

#ifdef TIMING /****************************************************************/
    int64_t ticksStart, ticksEnd;
    ticksStart = cv::getTickCount();
#endif /* TIMING **************************************************************/

    cv::split(window, bgr_planes);

#ifdef TIMING /****************************************************************/
    ticksEnd = cv::getTickCount();
    splitCount += (ticksEnd - ticksStart);
    ticksStart = cv::getTickCount();
#endif /* TIMING **************************************************************/

    for(int k=0; k<3; k++)
    {

#ifdef DSP /*******************************************************************/

    /**
     * Write and read buffer pointers point at the same address, but the read
     * elements must be interpreted as floats, while the written elements must
     * be interpreted as unsigned integers.
     */
    Uint32* write_buffer_pt = (Uint32*) get_pool_buffer_address();
    float*   read_buffer_pt =  (float*) get_pool_buffer_address();

#ifdef TIMING /****************************************************************/
    writeBufferCount -= cv::getTickCount();
#endif /* TIMING **************************************************************/

        /* Fill in pool buffer with k-th BGR plane (only rows for DSP). */
        int row_start = 0;
        for(int i=0; i<ROWS_FOR_DSP; i++)
        {
            for(int j=0; j<rows; j++)
                write_buffer_pt[row_start + j] = 
                    (Uint32) bgr_planes[k].at<uchar>((i+rows-ROWS_FOR_DSP),(j));

            row_start += rows;
        }

        /* Write into the buffer and notify the DSP. */
        Uint32 notification = NOTIF_BGR_PLANE | k;
        write_buffer(4 * rows * ROWS_FOR_DSP);  // 4 is the size of the elements
        notify_DSP(notification);

#ifdef TIMING /****************************************************************/
    writeBufferCount += cv::getTickCount();
#endif /* TIMING **************************************************************/

#endif /* DSP *****************************************************************/

        /* Start (parallel) computation with NEON extension. */
        for(int i=0; i<(rows-ROWS_FOR_DSP); i++)
        {
            /* Fetch and process groups of 4 elements */
            for(int j=0; j<cols; j=j+4)
            {
                uint32x4_t v_curr_pixel = {
                    (bgr_planes[k].at<uchar>(i,j)),
                    (bgr_planes[k].at<uchar>(i,j+1)),
                    (bgr_planes[k].at<uchar>(i,j+2)),
                    (bgr_planes[k].at<uchar>(i,j+3))
                };

                /* Shift right by 4 instead of dividing by 16. */
                uint32x4_t v_bin_value = vshrq_n_u32(v_curr_pixel,4);

                float32x4_t v_target_model = {
                    target_model.at<float>(k,vgetq_lane_u32(v_bin_value,0)),
                    target_model.at<float>(k,vgetq_lane_u32(v_bin_value,1)),
                    target_model.at<float>(k,vgetq_lane_u32(v_bin_value,2)),
                    target_model.at<float>(k,vgetq_lane_u32(v_bin_value,3))
                };

                float32x4_t v_target_candidate = {
                    target_candidate.at<float>(k,vgetq_lane_u32(v_bin_value,0)),
                    target_candidate.at<float>(k,vgetq_lane_u32(v_bin_value,1)),
                    target_candidate.at<float>(k,vgetq_lane_u32(v_bin_value,2)),
                    target_candidate.at<float>(k,vgetq_lane_u32(v_bin_value,3))
                };

                float32x4_t v_weight_new = 
                    vmulq_f32(v_target_model,vrecpeq_f32(v_target_candidate));

                if (k == 0)
                {
                    weight.at<float>(i,j)   = vgetq_lane_f32(v_weight_new,0);
                    weight.at<float>(i,j+1) = vgetq_lane_f32(v_weight_new,1);
                    weight.at<float>(i,j+2) = vgetq_lane_f32(v_weight_new,2);
                    weight.at<float>(i,j+3) = vgetq_lane_f32(v_weight_new,3);
                }
                else
                {
                    float32x4_t v_weight_old = {
                        weight.at<float>(i,j),
                        weight.at<float>(i,j+1),
                        weight.at<float>(i,j+2),
                        weight.at<float>(i,j+3)
                    };

                    float32x4_t v_result = 
                        vmulq_f32(v_weight_old,v_weight_new);

                    weight.at<float>(i,j)   = vgetq_lane_f32(v_result,0);
                    weight.at<float>(i,j+1) = vgetq_lane_f32(v_result,1);
                    weight.at<float>(i,j+2) = vgetq_lane_f32(v_result,2);
                    weight.at<float>(i,j+3) = vgetq_lane_f32(v_result,3);
                }
            }
        }

#ifdef DSP /*******************************************************************/

#ifdef TIMING /****************************************************************/
    idleCount -= cv::getTickCount();
#endif /* TIMING **************************************************************/

        /* Wait until the DSP notifies its completion. */
        wait_for_DSP();

#ifdef TIMING /****************************************************************/
    idleCount += cv::getTickCount();
#endif /* TIMING **************************************************************/

        /* Multiply weight rows received from the DSP with NEON. */
        row_start = 0;
        for (int i=(rows-ROWS_FOR_DSP); i<rows; i++)
        {
            for(int j=0; j<cols; j=j+4)
            {
                float32x4_t v_weight_new = {
                    read_buffer_pt[row_start+j],
                    read_buffer_pt[row_start+j+1],
                    read_buffer_pt[row_start+j+2],
                    read_buffer_pt[row_start+j+3],
                };
                if (k == 0)
                {
                    weight.at<float>(i,j)   = vgetq_lane_f32(v_weight_new,0);
                    weight.at<float>(i,j+1) = vgetq_lane_f32(v_weight_new,1);
                    weight.at<float>(i,j+2) = vgetq_lane_f32(v_weight_new,2);
                    weight.at<float>(i,j+3) = vgetq_lane_f32(v_weight_new,3);
                }
                else
                {
                    float32x4_t v_weight_old = {
                        weight.at<float>(i,j),
                        weight.at<float>(i,j+1),
                        weight.at<float>(i,j+2),
                        weight.at<float>(i,j+3)
                    };

                    float32x4_t v_result = 
                        vmulq_f32(v_weight_old,v_weight_new);

                    weight.at<float>(i,j)   = vgetq_lane_f32(v_result,0);
                    weight.at<float>(i,j+1) = vgetq_lane_f32(v_result,1);
                    weight.at<float>(i,j+2) = vgetq_lane_f32(v_result,2);
                    weight.at<float>(i,j+3) = vgetq_lane_f32(v_result,3);
                }
            }

            row_start += rows;
        }

#endif /* DSP *****************************************************************/

    }

#ifdef TIMING /****************************************************************/
    ticksEnd = cv::getTickCount();
    weightLoopCount += (ticksEnd - ticksStart);
    ticksStart = cv::getTickCount();
#endif /***********************************************************************/

    cv::sqrt(weight, weight);

#ifdef TIMING /****************************************************************/
    ticksEnd = cv::getTickCount();
    sqrtCount += (ticksEnd - ticksStart);
#endif /***********************************************************************/

    return weight;
}

cv::Rect MeanShift::track(const cv::Mat &next_frame, const cv::Mat &mask)
{
    cv::Mat curr_window;

#ifdef TIMING /****************************************************************/
    int64_t ticksStart, ticksEnd;
    ticksStart = cv::getTickCount();
#endif /***********************************************************************/

    /**
     * It is enough to compute the pdf representation once per frame.
     * Technically this could lead to a loss of accuracy, but we did not
     * experience that due to the conditions of the video (mainly slow car).
     */
    cv::Mat target_candidate = pdf_representation(next_frame,target_Region);

#ifdef TIMING /****************************************************************/
    ticksEnd = cv::getTickCount();
    pdfCount += (ticksEnd - ticksStart);
#endif /***********************************************************************/

    /**
     * Inside the rectangle, only the area inside a circle is considered for
     * tracking.
     *
     * centre:       horizontal centre of the circle.
     *
     * icentre:      reciprocal of the centre, used in the delta's update to
     *               spare some divisions and use increments instead.
     *
     * norm_j_shift: the original circle was left-aligned in the rectangle,
     *               now it is horizontally centred for better accuracy
     *               and faster algorithm convergence.
     */
    float centre = static_cast<float>((target_Region.height-1)/2);
    float icentre = static_cast<float>(2.0/(target_Region.height-1));
    float norm_j_shift = H_SHIFT * icentre;

    cv::Rect next_rect;

    for(int iter=0; iter<cfg.MaxIter; iter++)
    {
        /**
         * Instead of having the CalWeight function split the whole frame,
         * only take a square window that circumscribes the interest circle.
         */
        curr_window = cv::Mat(next_frame, 
                              cv::Range(target_Region.y,
                                        target_Region.y + target_Region.height), 
                              cv::Range(target_Region.x + H_SHIFT,
                                        target_Region.x + target_Region.width - H_SHIFT)
                              );
#ifdef TIMING /****************************************************************/
        ticksStart = cv::getTickCount();
#endif /***********************************************************************/

        cv::Mat weight = CalWeight(curr_window, target_model, target_candidate, target_Region);

#ifdef TIMING /****************************************************************/
        ticksEnd = cv::getTickCount();
        weightCount += (ticksEnd - ticksStart);
#endif /***********************************************************************/

        float delta_x = 0.0;
        float sum_wij = 0.0;
        float delta_y = 0.0;

        next_rect.x = target_Region.x;
        next_rect.y = target_Region.y;
        next_rect.width = target_Region.width;
        next_rect.height = target_Region.height;

#ifdef TIMING /****************************************************************/
        ticksStart = cv::getTickCount();
#endif /***********************************************************************/

        float norm_i = -1;
        for(int i=0; i<weight.rows; i++)
        {
            float norm_j = -1 + norm_j_shift;

            /**
             * The weight matrix is slightly wider than a square to accomodate
             * NEON functionalities, this explains the terminating condition
             * of the following for-loop
             */
            for(int j=0; j<weight.cols-2; j++)
            {
                if (mask.at<int>(i,j)) {
                    delta_x += static_cast<float>(norm_j*weight.at<float>(i,j));
                    delta_y += static_cast<float>(norm_i*weight.at<float>(i,j));
                    sum_wij += static_cast<float>(weight.at<float>(i,j));
                }
                norm_j += icentre;
            }
            norm_i += icentre;
        }

#ifdef TIMING /****************************************************************/
        ticksEnd = cv::getTickCount();
        loopCount += (ticksEnd - ticksStart);
#endif /***********************************************************************/

        next_rect.x += static_cast<int>((delta_x/sum_wij)*centre);
        next_rect.y += static_cast<int>((delta_y/sum_wij)*centre);

        if(abs(next_rect.x-target_Region.x)<1 && abs(next_rect.y-target_Region.y)<1)
            break;
        else
        {
            target_Region.x = next_rect.x;
            target_Region.y = next_rect.y;
        }
    }

    return next_rect;
}
