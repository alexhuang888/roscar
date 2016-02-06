#include "clinefollowernavigatorengine3.h"
#include "stdio.h"
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
//#include <gtest/gtest.h>

int main(int argc, char * argv[])
{
    if (argc != 2)
    {
        //cout <<" Usage: %s " << endl;
        printf("Usage: %s imagefile\n", argv[0]);
        return -1;
    }
	// Open the video file
    cv::VideoCapture capture(argv[1]);
	// check if video successfully opened
	if (!capture.isOpened())
		return 1;

	// Get the frame rate
	double rate = 30;//capture.get(CV_CAP_PROP_FPS);
	double delay = 1000 / rate;
	bool stop(false);
	cv::Mat frame; // current video frame

	// for all frames in video
	while (!stop) 
	{	
		// read next frame if any
		if (!capture.read(frame))
			break;

		cv::imshow("Extracted Frame",frame);

	    yisys_roswheels::CLineFollowerNavigatorEngine3 lfn2;
	
	    IplImage acopy = frame;
	    IplImage* new_image = &acopy;
	    float fAngle = 0;
	    CvPoint vanishingPoint;
	
	    lfn2.ProcessImage(new_image, true, fAngle, vanishingPoint);

		// introduce a delay
		// or press key to stop
		//if (cv::waitKey(delay) >= 0)
		//	stop = true;
		cv::waitKey();
	}

	// Close the video file
	capture.release();

	cv::waitKey();               // Show our image inside it.                                        // Wait for a keystroke in the window
    return 0;

}
