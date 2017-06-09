/*
 * Based on paper "Kernel-Based Object Tracking"
 * you can find all the formula in the paper
 */

#include "arm_neon.h"
#include "pool_notify.h"
#include "meanshift.h"
#include <stdint.h>
#include <stdlib.h>
#include <stdio.h>


#define ROWS_FOR_DSP            8


#define FIXED2FLOAT(N)          ( ((float)N) / (1 << 16) )

MeanShift::MeanShift()
{
    cfg.MaxIter = 8;
    cfg.num_bins = 16;
    cfg.piexl_range = 256;
    bin_width = cfg.piexl_range / cfg.num_bins;

    count = 1;

#ifdef TIMING
    splitCount      = 0;
    pdfCount        = 0;
    weightCount     = 0;
    loopCount       = 0;
    weightLoopCount = 0;
    sqrtCount       = 0;
#endif
}

void  MeanShift::Init_target_frame(const cv::Mat &frame, const cv::Rect &rect)
{
    target_Region = rect;
    target_model = pdf_representation(frame, target_Region);
}


cv::Mat MeanShift::pdf_representation(const cv::Mat &frame, const cv::Rect &rect)
{
    static int send_model = 1;

    cv::Mat pdf_model(8,16,CV_32F,cv::Scalar(1e-10));

    cv::Vec3b curr_pixel_value;
    cv::Vec3b bin_value;

    curr_pixel_value = frame.at<cv::Vec3b>(rect.y + rect.height/2, rect.x + rect.width/2);
    bin_value[0] = (curr_pixel_value[0] >> 4); //bin_width);
    bin_value[1] = (curr_pixel_value[1] >> 4); //bin_width);
    bin_value[2] = (curr_pixel_value[2] >> 4); //bin_width);
    
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
    
    pdf_model.at<float>(0,bin_value[0]) += 1;
    pdf_model.at<float>(1,bin_value[1]) += 1;
    pdf_model.at<float>(2,bin_value[2]) += 1;

    return pdf_model;
}

cv::Mat MeanShift::CalWeight(const cv::Mat &window, cv::Mat &target_model, 
                    cv::Mat &target_candidate, cv::Rect &rec)
{
    int rows = rec.height;
    int cols = rec.width;

    cv::Mat weight(rows, cols, CV_32F, cv::Scalar(1.0000));
    std::vector<cv::Mat> bgr_planes;

    Uint32* write_buffer_pt = (Uint32*) get_pool_buffer_address();
    Int32*   read_buffer_pt =  (Int32*) get_pool_buffer_address();

#ifdef TIMING
    int64_t ticksStart, ticksEnd;
    ticksStart = cv::getTickCount();
#endif

    cv::split(window, bgr_planes);

#ifdef TIMING
    ticksEnd = cv::getTickCount();
    splitCount += (ticksEnd - ticksStart);

    ticksStart = cv::getTickCount();
#endif

    for(int k=0; k<3;  k++)
    {
        int row_start = 0;
        for(int i=0; i<ROWS_FOR_DSP; i++)
        {
            for(int j=0; j<rows; j++)
                write_buffer_pt[row_start + j] = (Uint32) bgr_planes[k].at<uchar>((i+rows-ROWS_FOR_DSP),(j+14));

            row_start += rows;
        }

        Uint32 notification = NOTIF_BGR_PLANE | k;
        write_buffer((4 * rows * ROWS_FOR_DSP));
        notify_DSP(notification);

        for(int i=0; i<(rows-ROWS_FOR_DSP); i++)
        {
            for(int j=0; j<rows; j++)
            {
                int curr_pixel = (bgr_planes[k].at<uchar>(i,j+14));
                int bin_value = curr_pixel >> 4;
                weight.at<float>(i,j) *= static_cast<float>((target_model.at<float>(k, bin_value)/target_candidate.at<float>(k, bin_value)));
            }
        }

        wait_for_DSP();
        row_start = 0;
        for(int i=(rows-ROWS_FOR_DSP); i<rows; i++)
        {
            for(int j=0; j<rows; j++)
            {
                int curr_pixel = (bgr_planes[k].at<uchar>(i,j+14));
                int bin_value = curr_pixel >> 4;
                float division = static_cast<float>((target_model.at<float>(k, bin_value)/target_candidate.at<float>(k, bin_value)));
                
                if (count == 1)
                    std::cout << "from dsp: " << read_buffer_pt[row_start + j]
                              << ", actual: " << curr_pixel << std::endl;

                weight.at<float>(i,j) *= static_cast<float>(read_buffer_pt[row_start+j]);
            }

            row_start += rows;
        }
    }

#ifdef TIMING
    ticksEnd = cv::getTickCount();
    weightLoopCount += (ticksEnd - ticksStart);

    ticksStart = cv::getTickCount();
#endif

    cv::sqrt(weight, weight);

#ifdef TIMING
    ticksEnd = cv::getTickCount();
    sqrtCount += (ticksEnd - ticksStart);
#endif

    return weight;
}

cv::Rect MeanShift::track(const cv::Mat &next_frame, const cv::Mat &mult)
{
    cv::Mat curr_window;

#ifdef TIMING
    int64_t ticksStart, ticksEnd;
    ticksStart = cv::getTickCount();
#endif

    cv::Mat target_candidate = pdf_representation(next_frame,target_Region);

#ifdef TIMING
    ticksEnd = cv::getTickCount();
    pdfCount += (ticksEnd - ticksStart);
#endif

    float centre = static_cast<float>((mult.rows-1)/2);
    float icentre = static_cast<float>(2.0/(mult.rows-1));
    cv::Rect next_rect;

    for(int iter=0; iter<cfg.MaxIter; iter++)
    {
        curr_window = cv::Mat(next_frame, 
                              cv::Range(target_Region.y, target_Region.y + target_Region.height), 
                              cv::Range(target_Region.x, target_Region.x + target_Region.width)
                              );
#ifdef TIMING
        ticksStart = cv::getTickCount();
#endif

        cv::Mat weight = CalWeight(curr_window, target_model, target_candidate, target_Region);

#ifdef TIMING
        ticksEnd = cv::getTickCount();
        weightCount += (ticksEnd - ticksStart);
#endif

        float delta_x = 0.0;
        float sum_wij = 0.0;
        float delta_y = 0.0;

        next_rect.x = target_Region.x;
        next_rect.y = target_Region.y;
        next_rect.width = target_Region.width;
        next_rect.height = target_Region.height;

#ifdef TIMING
        ticksStart = cv::getTickCount();
#endif

        float norm_i = -1;
        for(int i=0; i<weight.rows; i++)
        {
            float norm_j = -1;
            for(int j=0; j<weight.rows; j++)
            {
                if (mult.at<int>(i,j)) {
                    delta_x += static_cast<float>(norm_j*weight.at<float>(i,j));
                    delta_y += static_cast<float>(norm_i*weight.at<float>(i,j));
                    sum_wij += static_cast<float>(weight.at<float>(i,j));
                }
                norm_j += icentre;
            }
            norm_i += icentre;
        }

#ifdef TIMING
        ticksEnd = cv::getTickCount();
        loopCount += (ticksEnd - ticksStart);
#endif

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

    count++;
    return next_rect;
}
