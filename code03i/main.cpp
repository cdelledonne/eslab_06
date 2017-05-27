#include "meanshift.h"
#include "Timer.h"
#include <iostream>

#ifndef ARMCC
#include "markers.h"
#endif

int main(int argc, char ** argv)
{
    Timer totalTimer("Total Time");
    int64_t startCycle, endCycle, readCount, trackCount, rectCount, writeCount;
    readCount = 0;
    trackCount = 0;
    rectCount = 0;
    writeCount = 0;

    cv::VideoCapture frame_capture;
    if(argc<2)
    {
        std::cout <<"specifiy an input video file to track" << std::endl;
        std::cout <<"Usage:  ./" << argv[0] << " car.avi" << std::endl;
        return -1;
    }
    else
    {
        frame_capture = cv::VideoCapture( argv[1] );
    }

    // this is used for testing the car video
    // instead of selection of object of interest using mouse
    cv::Rect rect(228,367,86,58);
    cv::Mat frame;
    frame_capture.read(frame);

    MeanShift ms; // creat meanshift obj
    ms.Init_target_frame(frame,rect); // init the meanshift

    int codec = CV_FOURCC('F', 'L', 'V', '1');
    cv::VideoWriter writer("tracking_result.avi", codec, 20, cv::Size(frame.cols,frame.rows));

    startCycle = cv::getTickCount();
    totalTimer.Start();
    #ifndef ARMCC
    MCPROF_START();
    #endif
    int TotalFrames = 32;
    int fcount;
    for(fcount=0; fcount<TotalFrames; ++fcount)
    {
        readCount -= cv::getTickCount();
        // read a frame
        int status = frame_capture.read(frame);
        if( 0 == status ) break;
        readCount += cv::getTickCount();

        // track object
        #ifndef ARMCC
        // MCPROF_START();
        #endif
        trackCount -= cv::getTickCount();
        cv::Rect ms_rect =  ms.track(frame);
        trackCount += cv::getTickCount();
        #ifndef ARMCC
        // MCPROF_STOP();
        #endif

        // mark the tracked object in frame
        rectCount -= cv::getTickCount();
        cv::rectangle(frame,ms_rect,cv::Scalar(0,0,255),3);
        rectCount += cv::getTickCount();

        // write the frame
        writeCount -= cv::getTickCount();
        writer << frame;
        writeCount += cv::getTickCount();
    }
    #ifndef ARMCC
    MCPROF_STOP();
    #endif
    endCycle = cv::getTickCount();
    totalTimer.Pause();

    std::cout << std::endl;
    totalTimer.Print();

    std::cout << "Processed " << fcount << " frames" << std::endl;
    std::cout << "Time: " << totalTimer.GetTime() <<" sec\nFPS : " << fcount/totalTimer.GetTime() << std::endl;
    std::cout << "Total cycles: " << (endCycle - startCycle) << std::endl;
    std::cout << "read: " << readCount << " (" << ((float)(readCount)*100/(endCycle - startCycle)) << " %)" << std::endl;
    std::cout << "track: " << trackCount << " (" << ((float)(trackCount)*100/(endCycle - startCycle)) << " %)" << std::endl;
    std::cout << " |-> pdf: " << ms.pdfCount << " (" << ((float)(ms.pdfCount)*100/(endCycle - startCycle)) << " %)" << std::endl;
    std::cout << " |-> weight: " << ms.weightCount << " (" << ((float)(ms.weightCount)*100/(endCycle - startCycle)) << " %)" << std::endl;
    std::cout << " |    |-> split: " << ms.splitCount << " (" << ((float)(ms.splitCount)*100/(endCycle - startCycle)) << " %)" << std::endl;
    std::cout << " |    |-> w_loop: " << ms.weightLoopCount << " (" << ((float)(ms.weightLoopCount)*100/(endCycle - startCycle)) << " %)" << std::endl;
    std::cout << " |    |-> sqrt: " << ms.sqrtCount << " (" << ((float)(ms.sqrtCount)*100/(endCycle - startCycle)) << " %)" << std::endl;
    std::cout << " |-> loop: " << ms.loopCount << " (" << ((float)(ms.loopCount)*100/(endCycle - startCycle)) << " %)" << std::endl;
    std::cout << "rect: " << rectCount << " (" << ((float)(rectCount)*100/(endCycle - startCycle)) << " %)" << std::endl;
    std::cout << "write: " << writeCount << " (" << ((float)(writeCount)*100/(endCycle - startCycle)) << " %)" << std::endl;
    // std::cout << "CalWeight() cycle count: " << ms.weightCycles << std::endl;

    return 0;
}
