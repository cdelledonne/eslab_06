/* ------------------------------------ OS Specific Headers ----------------- */
#include <stdio.h>
#include <stdlib.h>
#include <iostream>

/* ------------------------------------ Application Headers ----------------- */
#include "pool_notify.h"
#include "meanshift.h"
#include "Timer.h"

#ifndef ARMCC
#include "markers.h"
#endif

#define BASELINE_TIME_MS        12.341f
#define BASELINE_TRACK_CYCLES   9500000000.0f


int main(int argc, char ** argv)
{
    char dspExecutable[] = "pool_notify.out";
    char strBufferSize[] = "128";

    Timer totalTimer("Total Time");
    int64_t startCycle, endCycle;

#ifdef TIMING
    int64_t matCount, readCount, trackCount, writeCount;
    matCount   = 0;
    readCount  = 0;
    trackCount = 0;
    writeCount = 0;
#endif

    cv::VideoCapture frame_capture;

    if(argc<2)
    {
        printf ("Usage : %s car.avi\n", argv [0]);
        return -1;
    }
    else
    {
        frame_capture = cv::VideoCapture(argv[1]);
    }

    pool_notify_Init(dspExecutable, strBufferSize);

    // this is used for testing the car video
    // instead of selection of object of interest using mouse
    cv::Rect rect(228,367,86,58);
    cv::Mat frame;
    frame_capture.read(frame);
    
    MeanShift ms; // creat meanshift obj
    ms.Init_target_frame(frame, rect); // init the meanshift

    int codec = CV_FOURCC('F', 'L', 'V', '1');
    cv::VideoWriter writer("tracking_result.avi", codec, 20, cv::Size(frame.cols,frame.rows));

    startCycle = cv::getTickCount();
    totalTimer.Start();
    #ifndef ARMCC
    MCPROF_START();
    #endif
    int TotalFrames = 32;
    int fcount;

    int minSize = (rect.height < rect.width) ? rect.height : rect.width;
    cv::Mat mult(minSize, minSize, CV_32S, cv::Scalar(1));

#ifdef TIMING
    matCount -= cv::getTickCount();
#endif

    float icentre = static_cast<float>(2.0/(minSize-1));
    float norm_i = -1;
    for (int i=0; i<minSize; i++) {
        float norm_j = -1;
        for (int j=0; j<minSize; j++) {
            if (pow(norm_i,2)+pow(norm_j,2) > 1.0)
                mult.at<int>(i,j) = 0;
            norm_j += icentre;
        }
        norm_i += icentre;
    }

#ifdef TIMING
    matCount += cv::getTickCount();
#endif

    for(fcount=0; fcount<TotalFrames; ++fcount)
    {

#ifdef TIMING
        readCount -= cv::getTickCount();
#endif

        // read a frame
        int status = frame_capture.read(frame);
        if( 0 == status ) break;

#ifdef TIMING
        readCount += cv::getTickCount();
#endif

        #ifndef ARMCC
        // MCPROF_START();
        #endif

#ifdef TIMING
        trackCount -= cv::getTickCount();
#endif

        // track object
        cv::Rect ms_rect =  ms.track(frame, mult);

#ifdef TIMING
        trackCount += cv::getTickCount();
#endif

        #ifndef ARMCC
        // MCPROF_STOP();
        #endif
        
        // mark the tracked object in frame
        cv::rectangle(frame,ms_rect,cv::Scalar(0,0,255),3);

#ifdef TIMING
        writeCount -= cv::getTickCount();
#endif

        // write the frame
        writer << frame;

#ifdef TIMING
        writeCount += cv::getTickCount();
#endif

    }

    #ifndef ARMCC
    MCPROF_STOP();
    #endif
    endCycle = cv::getTickCount();
    totalTimer.Pause();

    pool_notify_Delete(0);

    std::cout << std::endl;
    totalTimer.Print();

    std::cout << "Processed " << fcount << " frames" << std::endl;
    std::cout << "Time: " << totalTimer.GetTime() << " sec" << std::endl;
    std::cout << "FPS: " << fcount/totalTimer.GetTime() << std::endl;
    std::cout << "Total cycles: " << (endCycle - startCycle) << std::endl;

#ifdef TIMING
    std::cout << "matrix: " << matCount << " (" << ((float)(matCount)*100/(endCycle - startCycle)) << " %)" << std::endl;
    std::cout << "read: " << readCount << " (" << ((float)(readCount)*100/(endCycle - startCycle)) << " %)" << std::endl;
    std::cout << "track: " << trackCount << " (" << ((float)(trackCount)*100/(endCycle - startCycle)) << " %)" << std::endl;
    std::cout << "├─> pdf: " << ms.pdfCount << " (" << ((float)(ms.pdfCount)*100/(endCycle - startCycle)) << " %)" << std::endl;
    std::cout << "├─> weight: " << ms.weightCount << " (" << ((float)(ms.weightCount)*100/(endCycle - startCycle)) << " %)" << std::endl;
    std::cout << "│   ├─> split: " << ms.splitCount << " (" << ((float)(ms.splitCount)*100/(endCycle - startCycle)) << " %)" << std::endl;
    std::cout << "│   ├─> loop: " << ms.weightLoopCount << " (" << ((float)(ms.weightLoopCount)*100/(endCycle - startCycle)) << " %)" << std::endl;
    std::cout << "│   │   └─> acc_count: " << ms.accCount << " (" << ((float)(ms.accCount)*100/(endCycle - startCycle)) << " %)" << std::endl;
    std::cout << "│   └─> sqrt: " << ms.sqrtCount << " (" << ((float)(ms.sqrtCount)*100/(endCycle - startCycle)) << " %)" << std::endl;
    std::cout << "└─> loop: " << ms.loopCount << " (" << ((float)(ms.loopCount)*100/(endCycle - startCycle)) << " %)" << std::endl;
    std::cout << "write: " << writeCount << " (" << ((float)(writeCount)*100/(endCycle - startCycle)) << " %)" << std::endl;
    std::cout << std::endl;
    std::cout << "Application speedup: " << BASELINE_TIME_MS/totalTimer.GetTime() << "x" << std::endl;
    std::cout << "Function speedup: " << BASELINE_TRACK_CYCLES/trackCount << "x" << std::endl;
    std::cout << "Clock frequency: " <<  cv::getTickFrequency() << std::endl;
#endif

    return 0;
}

