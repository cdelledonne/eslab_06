/*
 * Based on paper "Kernel-Based Object Tracking"
 * you can find all the formula in the paper
 */

#include "meanshift.h"

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
    accCount        = 0;
    sqrtCount       = 0;
#endif
}

void  MeanShift::Init_target_frame(const cv::Mat &frame, const cv::Rect &rect)
{
    target_Region = rect;
    target_model = pdf_representation(frame, target_Region);
}

uint32_t  MeanShift::Epanechnikov_kernel(cv::Mat &kernel)
{
    int h = kernel.rows;
    int w = kernel.cols;

    // float epanechnikov_cd = 0.1*PI*h*w;
    // float kernel_sum = 0.0;

    uint32_t epanechnikov_cd = FLOAT2FIXED(0.1*PI*h*w);
    uint32_t kernel_sum = 0;

    for(int i=0; i<h; i++)
    {
        int32_t x = i+i+h;

        for(int j=0;j<w;j++)
        {
            // float x = static_cast<float>(i - h/2);
            // float y = static_cast<float>(j - w/2);
            // float norm_x = x*x+y*y;
            // float result = norm_x < 1 ? (epanechnikov_cd * (1.0-norm_x)) : 0;
            //////////////////////////////////////////// float norm_x = x*x/(h*h/4) + y*y/(w*w/4);

            int32_t y = j+j+w;
            int32_t norm = x*x + y*y;
            uint32_t result = norm < 4 ? (MULT(epanechnikov_cd, INT2FIXED((4-norm) >> 2))) : 0;
            
            kernel.at<int>(i,j) = (int32_t)result;
            // std::cout << "result: " << result << std::endl;
            kernel_sum += result;
        }
    }
    std::cout << "kernel_sum: " << kernel_sum << std::endl;
    return kernel_sum;
}

cv::Mat MeanShift::pdf_representation(const cv::Mat &frame, const cv::Rect &rect)
{
    // cv::Mat kernel(rect.height,rect.width,CV_32S,cv::Scalar(0));
    // float normalized_C = 1.0 / Epanechnikov_kernel(kernel);
    // uint32_t normalized_C = DIV(INT2FIXED(1, 16), Epanechnikov_kernel(kernel), 16);

    cv::Mat pdf_model(8,16,CV_32S,cv::Scalar(8));

    cv::Vec3b curr_pixel_value;
    cv::Vec3b bin_value;

    // int row_index = rect.y;
    // int col_index = rect.x;

    // for(int i=0;i<rect.height;i++)
    // {
    //     col_index = rect.x;
    //     for(int j=0;j<rect.width;j++)
    //     {
            curr_pixel_value = frame.at<cv::Vec3b>(rect.y + rect.height/2, rect.x + rect.width/2);
            bin_value[0] = (curr_pixel_value[0] >> 4); //bin_width);
            bin_value[1] = (curr_pixel_value[1] >> 4); //bin_width);
            bin_value[2] = (curr_pixel_value[2] >> 4); //bin_width);

            // COLLAPSE 3 MULTIPLICATIONS INTO A SINGLE ONE
            // pdf_model.at<int>(0,bin_value[0]) += MULT((kernel.at<int>(i,j)), normalized_C, 16);
            // pdf_model.at<int>(1,bin_value[1]) += MULT((kernel.at<int>(i,j)), normalized_C, 16);
            // pdf_model.at<int>(2,bin_value[2]) += MULT((kernel.at<int>(i,j)), normalized_C, 16);
            // ***********************************************************************

            pdf_model.at<int>(0,bin_value[0]) += INT2FIXED(1);
            pdf_model.at<int>(1,bin_value[1]) += INT2FIXED(1);
            pdf_model.at<int>(2,bin_value[2]) += INT2FIXED(1);
    //         col_index++;
    //     }
    //     row_index++;
    // }

    return pdf_model;

}

cv::Mat MeanShift::CalWeight(const cv::Mat &window, cv::Mat &target_model, 
                    cv::Mat &target_candidate, cv::Rect &rec)
{
    int rows = rec.height;
    int cols = rec.width;

    cv::Mat weight(rows,cols,CV_32S,cv::Scalar(INT2FIXED(1)));
    std::vector<cv::Mat> bgr_planes;

#ifdef TIMING
    int64_t ticksStart, ticksEnd;
    ticksStart = cv::getTickCount();
#endif

    cv::split(window, bgr_planes);

#ifdef TIMING
    ticksEnd = cv::getTickCount();
    splitCount += (ticksEnd - ticksStart);
    // std::cout << "split: " << (ticksEnd - ticksStart) << ", ";

    ticksStart = cv::getTickCount();
#endif

    for(int k = 0; k < 3;  k++)
    {
        for(int i=0; i<rows; i++)
        {
            for(int j=0; j<cols; j++)
            {
                int curr_pixel = (bgr_planes[k].at<uchar>(i,j));
                // int bin_value = curr_pixel/bin_width;
                int bin_value = curr_pixel >> 4; // base 2 logarithm of bin_width is 4
                // weight.at<float>(i,j) *= static_cast<float>((sqrt(target_model.at<float>(k, bin_value)/target_candidate.at<float>(k, bin_value))));
                int32_t fixed_div = DIV(target_model.at<int>(k, bin_value), target_candidate.at<int>(k, bin_value));
                int32_t fixed_sqrt = FIXEDSQRT(fixed_div);
                weight.at<int>(i,j) = static_cast<int>(MULT(weight.at<int>(i,j), (fixed_sqrt)));
                // if (count == 1)
                //     std::cout << "w: " << weight.at<int>(i,j) 
                //               << ", mod: " << target_model.at<int>(k, bin_value)
                //               << ", cand: " << target_candidate.at<int>(k, bin_value)
                //               << std::endl;
            }
        }
    }

#ifdef TIMING
    ticksEnd = cv::getTickCount();
    weightLoopCount += (ticksEnd - ticksStart);
    // std::cout << "w_loop: " << (ticksEnd - ticksStart) << ", ";

    ticksStart = cv::getTickCount();
#endif

    // cv::sqrt(weight, weight);

#ifdef TIMING
    ticksEnd = cv::getTickCount();
    sqrtCount += (ticksEnd - ticksStart);
    // std::cout << "sqrt: " << (ticksEnd - ticksStart) << ", ";
#endif

    return weight;
}

cv::Rect MeanShift::track(const cv::Mat &next_frame, const cv::Mat &mult)
{
    cv::Mat curr_window;

    std::cout << "Frame: " << count << std::endl;

#ifdef TIMING
    int64_t ticksStart, ticksEnd;
    ticksStart = cv::getTickCount();
#endif

    cv::Mat target_candidate = pdf_representation(next_frame,target_Region);
    // std::cout << "pdf done..." << std::endl;

#ifdef TIMING
    ticksEnd = cv::getTickCount();
    pdfCount += (ticksEnd - ticksStart);
    // std::cout << "pdf: " << (ticksEnd - ticksStart) << std::endl;
#endif

    // float centre = static_cast<float>((mult.rows-1)/2);
    // float icentre = static_cast<float>(2.0/(mult.rows-1));
    int32_t centre = FLOAT2FIXED((mult.rows-1)/2);
    int32_t icentre = FLOAT2FIXED(2.0/(mult.rows-1));
    cv::Rect next_rect;

    for(int iter=0; iter<cfg.MaxIter; iter++)
    {
        curr_window = cv::Mat(next_frame, 
                              cv::Range(target_Region.y, target_Region.y + target_Region.height), 
                              cv::Range(target_Region.x, target_Region.x + target_Region.width)
                              );
        // std::cout << "window extracted..." << std::endl;

#ifdef TIMING
        ticksStart = cv::getTickCount();
#endif

        cv::Mat weight = CalWeight(curr_window, target_model, target_candidate, target_Region);
        // std::cout << "calweight done..." << std::endl;

#ifdef TIMING
        ticksEnd = cv::getTickCount();
        weightCount += (ticksEnd - ticksStart);
        // std::cout << "CalWeight: " << (ticksEnd - ticksStart) << ", ";
#endif

        int32_t delta_x = 0;
        int32_t delta_y = 0;
        int32_t sum_wij = 0;

        next_rect.x = target_Region.x;
        next_rect.y = target_Region.y;
        next_rect.width = target_Region.width;
        next_rect.height = target_Region.height;

#ifdef TIMING
        ticksStart = cv::getTickCount();
#endif

        int32_t norm_i = INT2FIXED(-1);
        for(int i=0; i<weight.rows; i++)
        {
            int32_t norm_j = INT2FIXED(-1);
            for(int j=0; j<weight.rows; j++)
            {
                if (mult.at<int>(i,j)) {
                    // delta_x += static_cast<float>(norm_j*weight.at<float>(i,j));
                    // delta_y += static_cast<float>(norm_i*weight.at<float>(i,j));
                    // sum_wij += static_cast<float>(weight.at<float>(i,j));
                    delta_x += static_cast<int>(MULT(norm_j, weight.at<int>(i,j)));
                    delta_y += static_cast<int>(MULT(norm_i, weight.at<int>(i,j)));
                    sum_wij += static_cast<int>(weight.at<int>(i,j));
                    std::cout << "delta_x: " << delta_x << std::endl;
                }
                norm_j += icentre;
            }
            norm_i += icentre;
        }
        // std::cout << "loop done..." << std::endl;

#ifdef TIMING
        ticksEnd = cv::getTickCount();
        loopCount += (ticksEnd - ticksStart);
        // std::cout << "loop: " << (ticksEnd - ticksStart) << ", ";
#endif

        // next_rect.x += static_cast<int>((delta_x/sum_wij)*centre);
        // next_rect.y += static_cast<int>((delta_y/sum_wij)*centre);
        next_rect.x += FIXED2INT(MULT(DIV(delta_x, sum_wij), centre));
        next_rect.y += FIXED2INT(MULT(DIV(delta_y, sum_wij), centre));
        std::cout << "x = " << next_rect.x << ", y = " << next_rect.y << std::endl;

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
