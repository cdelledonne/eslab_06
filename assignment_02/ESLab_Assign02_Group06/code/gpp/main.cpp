/* ------------------------------------ OS Specific Headers ----------------- */
#include <stdio.h>
#include <stdlib.h>
#include <iostream>

/* ------------------------------------ Application Headers ----------------- */
#include "meanshift.h"
#include "Timer.h"
#ifdef DSP /*******************************************************************/
#include "pool_notify.h"
#endif /* DSP *****************************************************************/

#ifndef ARMCC
#include "markers.h"
#endif

/* Baseline total time and track cycles for final comparison */
#define BASELINE_TIME_S                 12.341f
#define BASELINE_TRACK_CYCLES           9500000000.0f


int main(int argc, char ** argv)
{
    Timer totalTimer("Total Time");
    int64_t startCycle, endCycle, trackCount = 0;

#ifdef TIMING /****************************************************************/
    int64_t maskCount, readCount, writeCount;
    maskCount   = 0;
    readCount  = 0;
    writeCount = 0;
#endif /* TIMING **************************************************************/

    cv::VideoCapture frame_capture;

    /* Verify command line arguments */
    if (argc<2) {
        printf ("Usage: %s car.avi\n", argv[0]);
        return -1;
    }
    else
        frame_capture = cv::VideoCapture(argv[1]);

#ifdef DSP /*******************************************************************/
    char dspExecutable[] = "pool_notify.out";
    char strBufferSize[] = "65536";
    pool_notify_Init(dspExecutable, strBufferSize);
#endif /* DSP *****************************************************************/

    /**
     * This is used for testing the car video
     * instead of selection of object of interest using mouse
     *
     * The top-left corner horizontal coordinate was changed from 228 to 214
     * for a better inclusion of the car in the starting rectangle
     */
    cv::Rect rect(214,367,86,58);
    cv::Mat frame;
    frame_capture.read(frame);
    
    MeanShift ms; // creat meanshift obj
    ms.Init_target_frame(frame, rect); // init the meanshift

    int codec = CV_FOURCC('F', 'L', 'V', '1');
    cv::VideoWriter writer("tracking_result.avi", codec, 20, cv::Size(frame.cols,frame.rows));

    startCycle = cv::getTickCount();
    totalTimer.Start();

    int TotalFrames = 32;
    int fcount;

    /* Generate mask matrix for tracking area, used in MeanShift::track */
    int minSize = (rect.height < rect.width) ? rect.height : rect.width;
    cv::Mat mask(minSize, minSize, CV_32S, cv::Scalar(1));

#ifdef TIMING /****************************************************************/
    maskCount -= cv::getTickCount();
#endif /* TIMING **************************************************************/

    float icentre = static_cast<float>(2.0/(minSize-1)); // reciprocal of centre
    float norm_i = -1;
    for (int i=0; i<minSize; i++) {
        float norm_j = -1;
        for (int j=0; j<minSize; j++) {
            if (pow(norm_i,2)+pow(norm_j,2) > 1.0)
                mask.at<int>(i,j) = 0;
            norm_j += icentre;
        }
        norm_i += icentre;
    }

#ifdef TIMING /****************************************************************/
    maskCount += cv::getTickCount();
#endif /* TIMING **************************************************************/

    /* Loop through the frames */
    for(fcount=0; fcount<TotalFrames; ++fcount)
    {

#ifdef TIMING /****************************************************************/
        readCount -= cv::getTickCount();
#endif /* TIMING **************************************************************/

        // read a frame
        int status = frame_capture.read(frame);
        if( 0 == status ) break;

#ifdef TIMING /****************************************************************/
        readCount += cv::getTickCount();
#endif /* TIMING **************************************************************/

        trackCount -= cv::getTickCount();

        // track object
        cv::Rect ms_rect = ms.track(frame, mask);

        trackCount += cv::getTickCount();

        // mark the tracked object in frame
        cv::rectangle(frame, ms_rect, cv::Scalar(0,0,255), 3);

#ifdef TIMING /****************************************************************/
        writeCount -= cv::getTickCount();
#endif /* TIMING **************************************************************/

        // write the frame
        writer << frame;

#ifdef TIMING /****************************************************************/
        writeCount += cv::getTickCount();
#endif /* TIMING **************************************************************/

    }

    endCycle = cv::getTickCount();
    totalTimer.Pause();

#ifdef DSP /*******************************************************************/

    /* Inform the DSP it is not needed any longer and clear the pool */
    notify_DSP(0);
    pool_notify_Delete(0);

#endif /* DSP *****************************************************************/

    std::cout << std::endl;
    totalTimer.Print();

    /* Print timing information */

    std::cout << "Processed " << fcount << " frames" << std::endl;
    std::cout << "Time: " << totalTimer.GetTime() << " sec" << std::endl;
    std::cout << "FPS: " << fcount/totalTimer.GetTime() << std::endl;
    std::cout << "Total cycles: " << (endCycle - startCycle) << std::endl;

#ifdef TIMING /****************************************************************/
    std::cout << std::endl;
    std::cout << "matrix: " << maskCount << " (" << ((float)(maskCount)*100/(endCycle - startCycle)) << " %)" << std::endl;
    std::cout << "read: " << readCount << " (" << ((float)(readCount)*100/(endCycle - startCycle)) << " %)" << std::endl;
    std::cout << "track: " << trackCount << " (" << ((float)(trackCount)*100/(endCycle - startCycle)) << " %)" << std::endl;
    std::cout << "├─> pdf: " << ms.pdfCount << " (" << ((float)(ms.pdfCount)*100/(endCycle - startCycle)) << " %)" << std::endl;
    std::cout << "├─> weight: " << ms.weightCount << " (" << ((float)(ms.weightCount)*100/(endCycle - startCycle)) << " %)" << std::endl;
    std::cout << "│   ├─> split: " << ms.splitCount << " (" << ((float)(ms.splitCount)*100/(endCycle - startCycle)) << " %)" << std::endl;
    std::cout << "│   ├─> loop: " << ms.weightLoopCount << " (" << ((float)(ms.weightLoopCount)*100/(endCycle - startCycle)) << " %)" << std::endl;
#ifdef DSP /*******************************************************************/
    std::cout << "│   │   ├─> write to buffer: " << ms.writeBufferCount << " (" << ((float)(ms.writeBufferCount)*100/(endCycle - startCycle)) << " %)" << std::endl;
    std::cout << "│   │   └─> idle: " << ms.idleCount << " (" << ((float)(ms.idleCount)*100/(endCycle - startCycle)) << " %)" << std::endl;
#endif /* DSP *****************************************************************/
    std::cout << "│   └─> sqrt: " << ms.sqrtCount << " (" << ((float)(ms.sqrtCount)*100/(endCycle - startCycle)) << " %)" << std::endl;
    std::cout << "└─> loop: " << ms.loopCount << " (" << ((float)(ms.loopCount)*100/(endCycle - startCycle)) << " %)" << std::endl;
    std::cout << "write: " << writeCount << " (" << ((float)(writeCount)*100/(endCycle - startCycle)) << " %)" << std::endl;
#endif /* TIMING **************************************************************/
    std::cout << std::endl;
    std::cout << "Application speedup: " << BASELINE_TIME_S/totalTimer.GetTime() << "x" << std::endl;
    std::cout << "Function speedup: " << BASELINE_TRACK_CYCLES/trackCount << "x" << std::endl;

    return 0;
}

