#include "clinefollowernavigatorengine.h"
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
    //testing::InitGoogleTest(&argc, argv);
    cv::Mat image;
    image = cv::imread(argv[1], CV_LOAD_IMAGE_COLOR);   // Read the file

    if (!image.data)                              // Check for invalid input
    {
        printf("Could not open or find the image: %s\n", argv[1]);
        return -1;
    }

    cv::namedWindow( "source image", cv::WINDOW_AUTOSIZE );// Create a window for display.
    cv::imshow( "source image", image );                   // Show our image inside it.

    yisys_roswheels::CLineFollowerNavigatorEngine lfn;

    IplImage acopy = image;
    IplImage* new_image = &acopy;
    float fAngle = 0;
    CvPoint vanishingPoint;

    lfn.ProcessImage(new_image, true, fAngle, vanishingPoint);

    cv::waitKey(0);                                          // Wait for a keystroke in the window
    return 0;

}
