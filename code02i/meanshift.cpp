/*
 * Based on paper "Kernel-Based Object Tracking"
 * you can find all the formula in the paper
*/

#include"meanshift.h"
#include <iostream>
#include "arm_neon.h"


MeanShift::MeanShift()
{
    cfg.MaxIter = 8;
    cfg.num_bins = 16;
    cfg.piexl_range = 256;
    bin_width = cfg.piexl_range / cfg.num_bins;
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
    float l = h/2;
    float m= w/2;
    float32x4_t maxR = {l,m,m,m};
    float32x4_t maxR2 = {m,m,m,m};
    float norm_x[7];

    // h=58;; w=86

    float epanechnikov_cd = 0.1*PI*h*w;
    float kernel_sum = 0.0;
    for(int i=0;i<h;i++)
    {
        for(int j=0;j<w-2;j+=7)
        {

            float32x4_t alpha ={i,j,j+1,j+2};
            float32x4_t alpha2={j+3,j+4,j+5,j+6};
            float32x4_t xyyy=vsubq_f32(alpha,maxR);
            float32x4_t xyyy2=vsubq_f32(alpha2,maxR2);
            float32x4_t normi_x=vmulq_f32(xyyy,xyyy);
            float32x4_t normh_x=vmulq_f32(xyyy2,xyyy2);
            norm_x[0]=vgetq_lane_f32(normi_x, 0)+vgetq_lane_f32(normi_x, 1);
            norm_x[1]=vgetq_lane_f32(normi_x, 0)+vgetq_lane_f32(normi_x, 2);
            norm_x[2]=vgetq_lane_f32(normi_x, 0)+vgetq_lane_f32(normi_x, 3);
            norm_x[3]=vgetq_lane_f32(normi_x, 0)+vgetq_lane_f32(normh_x, 0);
            norm_x[4]=vgetq_lane_f32(normi_x, 0)+vgetq_lane_f32(normh_x, 1);
            norm_x[5]=vgetq_lane_f32(normi_x, 0)+vgetq_lane_f32(normh_x, 2);
            norm_x[6]=vgetq_lane_f32(normi_x, 0)+vgetq_lane_f32(normh_x, 3);
            for(int k=0;k<7;k++)
            {
            float result =norm_x[k]<1?(epanechnikov_cd*(1.0-norm_x[k])):0;
            kernel.at<float>(i,j+k) = result;
            kernel_sum += result;
          }
        }
    }
    return kernel_sum;
}
cv::Mat MeanShift::pdf_representation(const cv::Mat &frame, const cv::Rect &rect)
{
    cv::Mat kernel(rect.height,rect.width,CV_32F,cv::Scalar(0));
    float normalized_C = 1.0 / Epanechnikov_kernel(kernel);

    cv::Mat pdf_model(8,16,CV_32F,cv::Scalar(1e-10));

    cv::Vec3f curr_pixel_value;
    cv::Vec3f bin_value;

    int row_index = rect.y;
    int clo_index = rect.x;

    for(int i=0;i<rect.height;i++)
    {
        clo_index = rect.x;
        for(int j=0;j<rect.width;j++)
        {
            curr_pixel_value = frame.at<cv::Vec3b>(row_index,clo_index);
            //have to vecctorize this
            bin_value[0] = (curr_pixel_value[0]/bin_width);
            bin_value[1] = (curr_pixel_value[1]/bin_width);
            bin_value[2] = (curr_pixel_value[2]/bin_width);
            pdf_model.at<float>(0,bin_value[0]) += kernel.at<float>(i,j)*normalized_C;
            pdf_model.at<float>(1,bin_value[1]) += kernel.at<float>(i,j)*normalized_C;
            pdf_model.at<float>(2,bin_value[2]) += kernel.at<float>(i,j)*normalized_C;
            clo_index++;
        }
        row_index++;
    }

    return pdf_model;

}

cv::Mat MeanShift::CalWeight(const std::vector<cv::Mat> &bgr_planes, cv::Mat &target_model,
                    cv::Mat &target_candidate, cv::Rect &rec)
{
    int rows = rec.height;
    int cols = rec.width;
    int row_index = rec.y;
    int col_index = rec.x;


    // Number of rows=58 and cols=86
    cv::Mat weight(rows,cols,CV_32F,cv::Scalar(1.0000));


    for(int k = 0; k < 3;  k++)
    {
        row_index = rec.y;
        for(int i=0;i<rows;i++)
        {
            col_index = rec.x;
            for(int j=0;j<cols;j++)
            {
                int curr_pixel = (bgr_planes[k].at<uchar>(row_index,col_index));
                int bin_value = curr_pixel/bin_width;
                weight.at<float>(i,j) *= static_cast<float>((sqrt(target_model.at<float>(k, bin_value)/target_candidate.at<float>(k, bin_value))));
                col_index++;
            }
            row_index++;
        }
    }

    return weight;
}

cv::Rect MeanShift::track(const cv::Mat &next_frame)
{
  std::vector<cv::Mat> bgr_planes;
  cv::split(next_frame, bgr_planes);
    cv::Rect next_rect;
      cv::Mat target_candidate = pdf_representation(next_frame,target_Region);
   //Max iterations = 8
    for(int iter=0;iter<cfg.MaxIter;iter++)
    {

        cv::Mat weight = CalWeight(bgr_planes,target_model,target_candidate,target_Region);

        float delta_x = 0.0;
        float sum_wij = 0.0;
        float delta_y = 0.0;
        float centre = static_cast<float>((weight.rows-1)/2.0);

        float mult = 0.0;
        //float32x2_t mult1={3.0,9.0};
        //size of double is 8

        next_rect.x = target_Region.x;
        next_rect.y = target_Region.y;
        next_rect.width = target_Region.width;
        next_rect.height = target_Region.height;
       //std::cout<<next_rect.width<<"\t"<<next_rect.height<<"\n";
        for(int i=0;i<weight.rows;i++)
        {

            for(int j=0;j<weight.cols;j++)
            {


                float norm_i = static_cast<float>(i-centre)/centre;
                float norm_j = static_cast<float>(j-centre)/centre;
                mult = (norm_i*norm_i)+(norm_j*norm_j)>1.0?0.0:1.0;
                delta_x += static_cast<float>(norm_j*weight.at<float>(i,j)*mult);
                delta_y += static_cast<float>(norm_i*weight.at<float>(i,j)*mult);
                sum_wij += static_cast<float>(weight.at<float>(i,j)*mult);
            }
        }

        next_rect.x += static_cast<int>((delta_x/sum_wij)*centre);
        next_rect.y += static_cast<int>((delta_y/sum_wij)*centre);

        if(abs(next_rect.x-target_Region.x)<1 && abs(next_rect.y-target_Region.y)<1)
        {
            break;
        }
        else
        {
            target_Region.x = next_rect.x;
            target_Region.y = next_rect.y;
        }
    }

    return next_rect;
}
