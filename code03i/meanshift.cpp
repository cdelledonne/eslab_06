/*
 * Based on paper "Kernel-Based Object Tracking"
 * you can find all the formula in the paper
*/

#include "meanshift.h"
#include "arm_neon.h"
#include <iostream>

MeanShift::MeanShift()
{
    cfg.MaxIter = 8;
    cfg.num_bins = 16;
    cfg.piexl_range = 256;
    bin_width = cfg.piexl_range / cfg.num_bins;
    splitCount = 0;
    pdfCount = 0;
    weightCount = 0;
    loopCount = 0;
    weightLoopCount = 0;
    sqrtCount = 0;
}

void  MeanShift::Init_target_frame(const cv::Mat &frame,const cv::Rect &rect)
{
    target_Region = rect;
    target_model = pdf_representation(frame,target_Region);
}

float  MeanShift::Epanechnikov_kernel(cv::Mat &kernel)
{
    int h = kernel.rows;
    int w = kernel.cols;
    float l=h/2;
    float m=w/2;

    float epanechnikov_cd = 0.1*PI*h*w;
    float kernel_sum = 0.0;
    for(int i=0;i<h;i++)
    {
        for(int j=0;j<w;j++)
        {
            float x = static_cast<float>(i - l);
            float  y = static_cast<float> (j - m);
            // float norm_x = x*x/(h*h/4)+y*y/(w*w/4);
            float norm_x = x*x+y*y;
            float result =norm_x<1?(epanechnikov_cd*(1.0-norm_x)):0;
            kernel.at<float>(i,j) = result;
            kernel_sum += result;
        }
    }
    return kernel_sum;
}
cv::Mat MeanShift::pdf_representation(const cv::Mat &frame, const cv::Rect &rect)
{
    cv::Mat kernel(rect.height,rect.width,CV_32F,cv::Scalar(0));
    float normalized_C = 1.0 / Epanechnikov_kernel(kernel);

    cv::Mat pdf_model(8,16,CV_32F,cv::Scalar(1e-10));

    cv::Vec3b curr_pixel_value;
    cv::Vec3b bin_value;

    int row_index = rect.y;
    int clo_index = rect.x;

    for(int i=0;i<rect.height;i++)
    {
        clo_index = rect.x;
        for(int j=0;j<rect.width;j++)
        {
            curr_pixel_value = frame.at<cv::Vec3b>(row_index,clo_index);
            bin_value[0] = (curr_pixel_value[0]>>4);
            bin_value[1] = (curr_pixel_value[1]>>4);
            bin_value[2] = (curr_pixel_value[2]>>4);

            // COLLAPSE 3 MULTIPLICATIONS INTO A SINGLE ONE
            pdf_model.at<float>(0,bin_value[0]) += kernel.at<float>(i,j)*normalized_C;
            pdf_model.at<float>(1,bin_value[1]) += kernel.at<float>(i,j)*normalized_C;
            pdf_model.at<float>(2,bin_value[2]) += kernel.at<float>(i,j)*normalized_C;
            // ***********************************************************************

            clo_index++;
        }
        row_index++;
    }

    return pdf_model;

}

cv::Mat MeanShift::CalWeight(const cv::Mat &window, cv::Mat &target_model,
                    cv::Mat &target_candidate, cv::Rect &rec)
{
    int rows = rec.height;
    int cols = rec.width;
    int row_index = 0;
    int col_index = 0;
    int64_t ticksStart, ticksEnd;

    cv::Mat weight(rows,cols,CV_32F,cv::Scalar(1.0000));
    std::vector<cv::Mat> bgr_planes;

    ticksStart = cv::getTickCount();
    cv::split(window, bgr_planes);
    ticksEnd = cv::getTickCount();
    splitCount += (ticksEnd - ticksStart);
    //std::cout << "split: " << (ticksEnd - ticksStart) << ", ";
     //int32x4_t bin_width1=vdupq_n_f32(vshrq_n_);
     //float32x4_t bin_width2=vdupq_n_f32(14.858);
     //int32x4_t ishu= vcvtq_s32_f32(bin_width2);
     //int hell=vgetq_lane_s32(ishu,0);
     //std::cout<<hell<<"\n";

    ticksStart = cv::getTickCount();


    for(int k = 0; k < 3;  k++)
    {
        row_index = 0;
        for(int i=0;i<rows;i++)
        {
            col_index = 0;
            for(int j=0;j<cols-2;j=j+6)
            {
                uint32x4_t curr_pixel={(bgr_planes[k].at<uchar>(row_index,col_index)),(bgr_planes[k].at<uchar>(row_index,col_index+1)),(bgr_planes[k].at<uchar>(row_index,col_index+2)),(bgr_planes[k].at<uchar>(row_index,col_index+3))};
                uint32x4_t curr_pixel1={(bgr_planes[k].at<uchar>(row_index,col_index+4)),(bgr_planes[k].at<uchar>(row_index,col_index+5)),0,0};
                uint32x4_t bin_value=vshrq_n_u32(curr_pixel,4);
                uint32x4_t bin_value1=vshrq_n_u32(curr_pixel1,4);

                float32x4_t target_m={target_model.at<float>(k,vgetq_lane_u32(bin_value,0)),target_model.at<float>(k, vgetq_lane_u32(bin_value,1)),target_model.at<float>(k, vgetq_lane_u32(bin_value,2)),target_model.at<float>(k, vgetq_lane_u32(bin_value,3))};


                float32x4_t target_m1={target_model.at<float>(k,vgetq_lane_u32(bin_value1,0)),target_model.at<float>(k, vgetq_lane_u32(bin_value1,1)),1,1};


                float32x4_t target_c={target_candidate.at<float>(k, vgetq_lane_u32(bin_value,0)),target_candidate.at<float>(k, vgetq_lane_u32(bin_value,1)),target_candidate.at<float>(k, vgetq_lane_u32(bin_value,2)),target_candidate.at<float>(k, vgetq_lane_u32(bin_value,3))};


                float32x4_t target_c1={target_candidate.at<float>(k, vgetq_lane_u32(bin_value1,0)),target_candidate.at<float>(k, vgetq_lane_u32(bin_value1,1)),1,1};



                float32x4_t milan=vmulq_f32(target_m,vrecpeq_f32(target_c));
                float32x4_t milan1=vmulq_f32(target_m1,vrecpeq_f32(target_c1));



                float32x4_t weight1={weight.at<float>(i,j),weight.at<float>(i,j+1),weight.at<float>(i,j+2),weight.at<float>(i,j+3)};
               float32x4_t weight2={weight.at<float>(i,j+4),weight.at<float>(i,j+5),1,1};

                float32x4_t weight4=vmulq_f32(weight1,milan);
                float32x4_t weight5=vmulq_f32(weight2,milan1);
                
                weight.at<float>(i,j)   = vgetq_lane_f32(weight4,0);
                weight.at<float>(i,j+1) = vgetq_lane_f32(weight4,1);
                weight.at<float>(i,j+2) = vgetq_lane_f32(weight4,2);
                weight.at<float>(i,j+3) = vgetq_lane_f32(weight4,3);
                weight.at<float>(i,j+4) = vgetq_lane_f32(weight5,0);
                weight.at<float>(i,j+5) = vgetq_lane_f32(weight5,1);
                //weight.at<float>(i,j+6) *= vgetq_lane_f32(milan1,2);
                //weight.at<float>(i,j+7) *= vgetq_lane_f32(milan1,3);


                col_index+=6;
            }
            row_index++;
        }
    }

  /*  for(int i=0;i<rows;i++)
    {
        col_index = 0;
        for(int j=0;j<=cols-2;j=j+3)

        {   float32x4_t curr_pixel={(bgr_planes[0].at<uchar>(row_index,col_index)),(bgr_planes[1].at<uchar>(row_index,col_index)),(bgr_planes[2].at<uchar>(row_index,col_index)),0};
            float32x4_t curr_pixel1={(bgr_planes[0].at<uchar>(row_index,col_index+1)),(bgr_planes[1].at<uchar>(row_index,col_index+1)),(bgr_planes[2].at<uchar>(row_index,col_index+1)),0};
            float32x4_t curr_pixel2={(bgr_planes[0].at<uchar>(row_index,col_index+2)),(bgr_planes[1].at<uchar>(row_index,col_index+2)),(bgr_planes[2].at<uchar>(row_index,col_index+2)),0};
            //float32x4_t curr_pixel3={(bgr_planes[0].at<uchar>(row_index,col_index+3)),(bgr_planes[1].at<uchar>(row_index,col_index+3)),(bgr_planes[2].at<uchar>(row_index,col_index+3)),0};


            float32x4_t bin_value=vmulq_n_f32(curr_pixel,bin_width1);
            float32x4_t bin_value1=vmulq_n_f32(curr_pixel1,bin_width1);
            float32x4_t bin_value2=vmulq_n_f32(curr_pixel2,bin_width1);
            //float32x4_t bin_value3=vmulq_f32(curr_pixel3,bin_width1);

            float32x4_t target_m={target_model.at<float>(0,vgetq_lane_f32(bin_value,0)),target_model.at<float>(1, vgetq_lane_f32(bin_value,1)),target_model.at<float>(2, vgetq_lane_f32(bin_value,2)),0};
            float32x4_t target_m1={target_model.at<float>(0,vgetq_lane_f32(bin_value1,0)),target_model.at<float>(1, vgetq_lane_f32(bin_value1,1)),target_model.at<float>(2, vgetq_lane_f32(bin_value1,2)),0};
            float32x4_t target_m2={target_model.at<float>(0,vgetq_lane_f32(bin_value2,0)),target_model.at<float>(1, vgetq_lane_f32(bin_value2,1)),target_model.at<float>(2, vgetq_lane_f32(bin_value2,2)),0};
            //float32x4_t target_m3={target_model.at<float>(0,vgetq_lane_f32(bin_value3,0)),target_model.at<float>(1, vgetq_lane_f32(bin_value3,1)),target_model.at<float>(2, vgetq_lane_f32(bin_value3,2)),0};

            float32x4_t target_c={target_candidate.at<float>(0, vgetq_lane_f32(bin_value,0)),target_candidate.at<float>(1, vgetq_lane_f32(bin_value,1)),target_candidate.at<float>(2, vgetq_lane_f32(bin_value,2)),0};
            float32x4_t target_c1={target_candidate.at<float>(0, vgetq_lane_f32(bin_value1,0)),target_candidate.at<float>(1, vgetq_lane_f32(bin_value1,1)),target_candidate.at<float>(2, vgetq_lane_f32(bin_value1,2)),0};
            float32x4_t target_c2={target_candidate.at<float>(0, vgetq_lane_f32(bin_value2,0)),target_candidate.at<float>(1, vgetq_lane_f32(bin_value2,1)),target_candidate.at<float>(2, vgetq_lane_f32(bin_value2,2)),0};
            //float32x4_t target_c3={target_candidate.at<float>(0, vgetq_lane_f32(bin_value3,0)),target_candidate.at<float>(1, vgetq_lane_f32(bin_value3,1)),target_candidate.at<float>(2, vgetq_lane_f32(bin_value3,2)),0};


            float32x4_t milan=vmulq_f32(target_m,vrecpeq_f32(target_c));
            float32x4_t milan1=vmulq_f32(target_m1,vrecpeq_f32(target_c1));
            float32x4_t milan2=vmulq_f32(target_m2,vrecpeq_f32(target_c2));
            //float32x4_t milan3=vmulq_f32(target_m3,vrecpeq_f32(target_c3));

           //float m2=vgetq_lane_f32(milan,2);
           //std::cout << m2 << '\n';


            weight.at<float>(i,j) *= vgetq_lane_f32(milan,0);
            weight.at<float>(i,j) *= vgetq_lane_f32(milan,1);
            weight.at<float>(i,j) *= vgetq_lane_f32(milan,2);

            weight.at<float>(i,j+1) *= vgetq_lane_f32(milan1,0);
            weight.at<float>(i,j+1) *= vgetq_lane_f32(milan1,1);
            weight.at<float>(i,j+1) *= vgetq_lane_f32(milan1,2);

            weight.at<float>(i,j+2) *= vgetq_lane_f32(milan2,0);
            weight.at<float>(i,j+2) *= vgetq_lane_f32(milan2,1);
            weight.at<float>(i,j+2) *= vgetq_lane_f32(milan2,2);

            //weight.at<float>(i,j+3) *= vgetq_lane_f32(milan3,0);
            //weight.at<float>(i,j+3) *= vgetq_lane_f32(milan3,1);
            //weight.at<float>(i,j+3) *= vgetq_lane_f32(milan3,2);

            col_index+=3;
        }
        row_index++;
    }
*/
    ticksEnd = cv::getTickCount();
    weightLoopCount += (ticksEnd - ticksStart);
    //std::cout << "w_loop: " << (ticksEnd - ticksStart) << ", ";

    ticksStart = cv::getTickCount();
    cv::sqrt(weight, weight);
    ticksEnd = cv::getTickCount();
    sqrtCount += (ticksEnd - ticksStart);
    //std::cout << "sqrt: " << (ticksEnd - ticksStart) << ", ";

    return weight;
}

cv::Rect MeanShift::track(const cv::Mat &next_frame)
{
    int64_t ticksStart, ticksEnd;
    cv::Mat window;

    ticksStart = cv::getTickCount();
    cv::Mat target_candidate = pdf_representation(next_frame,target_Region);
    ticksEnd = cv::getTickCount();
    pdfCount += (ticksEnd - ticksStart);
    //std::cout << "pdf: " << (ticksEnd - ticksStart) << std::endl;

    cv::Rect next_rect;
    for(int iter=0; iter<cfg.MaxIter; iter++)
    {
        // ticksStart = cv::getTickCount();
        // MOVE THE FOLLOWING LINE OUTSIDE THE LOOP
        // cv::Mat target_candidate = pdf_representation(next_frame,target_Region);
        // *********************************************************************
        // ticksEnd = cv::getTickCount();
        // std::cout << "pdf: " << (ticksEnd - ticksStart) << ", ";

        window = cv::Mat(next_frame, cv::Range(target_Region.y, target_Region.y + 58), cv::Range(target_Region.x, target_Region.x + 86));

        ticksStart = cv::getTickCount();
        cv::Mat weight = CalWeight(window,target_model,target_candidate,target_Region);
        ticksEnd = cv::getTickCount();
        weightCount += (ticksEnd - ticksStart);
        //std::cout << "CalWeight: " << (ticksEnd - ticksStart) << ", ";

        float delta_x = 0.0;
        float sum_wij = 0.0;
        float delta_y = 0.0;
        float centre = static_cast<float>((weight.rows-1)/2.0);
        double mult = 0.0;

        next_rect.x = target_Region.x;
        next_rect.y = target_Region.y;
        next_rect.width = target_Region.width;
        next_rect.height = target_Region.height;

        ticksStart = cv::getTickCount();

        for(int i=0; i<weight.rows; i++)
        {
            for(int j=0; j<weight.cols; j++)
            {
                // MOVE THE FOLLOWING LINE ONE LEVEL ABOVE
                float norm_i = static_cast<float>(i-centre)/centre;
                // ************************************************

                float norm_j = static_cast<float>(j-centre)/centre;
                mult = pow(norm_i,2)+pow(norm_j,2)>1.0?0.0:1.0;
                delta_x += static_cast<float>(norm_j*weight.at<float>(i,j)*mult);
                delta_y += static_cast<float>(norm_i*weight.at<float>(i,j)*mult);
                sum_wij += static_cast<float>(weight.at<float>(i,j)*mult);

                // REPLACE THE ABOVE BLOCK WITH THE FOLLOWING
                // if (pow(norm_i,2)+pow(norm_j,2) <= 1.0) {
                //     delta_x += static_cast<float>(norm_j*weight.at<float>(i,j));
                //     delta_y += static_cast<float>(norm_i*weight.at<float>(i,j));
                //     sum_wij += static_cast<float>(weight.at<float>(i,j));
                // }
            }
        }

        ticksEnd = cv::getTickCount();
        loopCount += (ticksEnd - ticksStart);
      //  std::cout << "loop: " << (ticksEnd - ticksStart) << ", ";

        ticksStart = cv::getTickCount();

        next_rect.x += static_cast<int>((delta_x/sum_wij)*centre);
        next_rect.y += static_cast<int>((delta_y/sum_wij)*centre);

        if(abs(next_rect.x-target_Region.x)<1 && abs(next_rect.y-target_Region.y)<1)
        {
            ticksEnd = cv::getTickCount();
            //std::cout << "rest: " << (ticksEnd - ticksStart) << std::endl;
            break;
        }
        else
        {
            target_Region.x = next_rect.x;
            target_Region.y = next_rect.y;
            ticksEnd = cv::getTickCount();
            //std::cout << "rest: " << (ticksEnd - ticksStart) << std::endl;
        }
    }

    return next_rect;
}
