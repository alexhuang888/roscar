#include "clanedetectornavigatorengine.h"
#include "vector"
#include<opencv2/core/core.hpp>
#include<opencv2/highgui/highgui.hpp>
#include "myutil.h"

namespace yisys_roswheels
{
CLaneDetectorNavigatorEngine::CLaneDetectorNavigatorEngine()
{
	m_bPaused = true;
	m_pHoughStorage = NULL;
	m_FrameSize.width = 0;
	m_FrameSize.height = 0;
	m_HalfFrameSize.width = 0;
	m_HalfFrameSize.height = 0;

	cvInitFont(&m_Font, CV_FONT_VECTOR0, 0.5f, 0.5f);
	m_pWorkingImage = NULL;
	m_pGreyImage = NULL;
	m_pEdgesImage = NULL;

	m_bShowLine = false;

	m_fTurnAngle = 0.f;
	m_VanishingPoint.x = 0;
	m_VanishingPoint.y = 0;
}

CLaneDetectorNavigatorEngine::~CLaneDetectorNavigatorEngine()
{
	if (m_pHoughStorage != NULL)
		cvReleaseMemStorage(&m_pHoughStorage);

	if (m_pGreyImage != NULL)
		cvReleaseImage(&m_pGreyImage);

	if (m_pEdgesImage != NULL)
		cvReleaseImage(&m_pEdgesImage);

	if (m_pWorkingImage != NULL)
		cvReleaseImage(&m_pWorkingImage);
}

int32_t CLaneDetectorNavigatorEngine::Init(void)
{
	return 1;
}

int32_t CLaneDetectorNavigatorEngine::Start(void)
{
	m_bPaused = false;
	return 1;
}

int32_t CLaneDetectorNavigatorEngine::Pause(void)
{
	m_bPaused = true;
	return 1;
}

void CLaneDetectorNavigatorEngine::FindResponses(IplImage *pImage, int startX, int endX, int y, std::vector<int>& list)
{
    // scans for single response: /^\_
	const int row = y * pImage->width * pImage->nChannels;
	unsigned char* ptr = (unsigned char*)pImage->imageData;

    int step = (endX < startX) ? -1: 1;
    int range = (endX > startX) ? endX - startX + 1 : startX - endX + 1;

    for (int x = startX; range > 0; x += step, range--)
    {
        if (ptr[row + x] <= BW_TRESHOLD)
			continue; // skip black: loop until white pixels show up

        // first response found
        int idx = x + step;

        // skip same response(white) pixels
        while (range > 0 && ptr[row+idx] > BW_TRESHOLD)
        {
            idx += step;
            range--;
        }

		// reached black again
        if (ptr[row + idx] <= BW_TRESHOLD)
        {
            list.push_back(x);
        }

        x = idx; // begin from new pos
    }
}
void CLaneDetectorNavigatorEngine::ProcessSide(std::vector<CLaneInfo> lanes, IplImage *pEdges, bool bRightLane)
{
	CLaneStatus *pLaneStatus = bRightLane ? &m_LaneR : &m_LaneL;

	// response search
	int w = pEdges->width;
	int h = pEdges->height;
	const int BEGINY = 0;
	const int ENDY = h - 1;
	const int ENDX = bRightLane ? (w - BORDERX) : BORDERX;
	int midx = w / 2;
	int midy = pEdges->height / 2;
	unsigned char* ptr = (unsigned char*)pEdges->imageData;

	// show responses
	int* votes = new int[lanes.size()];

	for (int i = 0; i < lanes.size(); i++)
		votes[i++] = 0;

	for (int y = ENDY; y >= BEGINY; y -= SCAN_STEP)
	{
		std::vector<int> rsp;
		FindResponses(pEdges, midx, ENDX, y, rsp);

		if (rsp.size() > 0)
		{
			int response_x = rsp[0]; // use first reponse (closest to screen center)

			float dmin = 9999999;
			float xmin = 9999999;
			int match = -1;

			for (int j = 0; j < lanes.size(); j++)
			{
				// compute response point distance to current line
				float d = dist2line(
							cvPoint2D32f(lanes[j].m_p0.x, lanes[j].m_p0.y),
							cvPoint2D32f(lanes[j].m_p1.x, lanes[j].m_p1.y),
							cvPoint2D32f(response_x, y));

				// point on line at current y line
				int xline = (y - lanes[j].m_b) / lanes[j].m_k;
				int dist_mid = abs(midx - xline); // distance to midpoint

				// pick the best closest match to line & to screen center
				if (match == -1 || (d <= dmin && dist_mid < xmin))
				{
					dmin = d;
					match = j;
					xmin = dist_mid;
					break;
				}
			}

			// vote for each line
			if (match != -1)
			{
				votes[match] += 1;
			}
		}
	}

	int bestMatch = -1;
	int mini = 9999999;

	for (int i = 0; i < lanes.size(); i++)
	{
		int xline = (midy - lanes[i].m_b) / lanes[i].m_k;
		int dist = abs(midx - xline); // distance to midpoint

		if (bestMatch == -1 || (votes[i] > votes[bestMatch] && dist < mini))
		{
			bestMatch = i;
			mini = dist;
		}
	}

	if (bestMatch != -1)
	{
		CLaneInfo* best = &lanes[bestMatch];
		float k_diff = fabs(best->m_k - pLaneStatus->k.get());
		float b_diff = fabs(best->m_b - pLaneStatus->b.get());

		bool update_ok = (k_diff <= K_VARY_FACTOR && b_diff <= B_VARY_FACTOR) || pLaneStatus->reset;

		myprintf(2, 1, "pLaneStatus: %s, k vary: %-9.2f, b vary: %-9.2f, lost: %s\n",
			(bRightLane ? "RIGHT":"LEFT "), k_diff, b_diff, (update_ok?"no ":"yes"));

		if (update_ok)
		{
			// update is in valid bounds
			pLaneStatus->k.add(best->m_k);
			pLaneStatus->b.add(best->m_b);
			pLaneStatus->reset = false;
			pLaneStatus->lost = 0;
		}
		else
		{
			// can't update, lanes flicker periodically, start counter for partial reset!
			pLaneStatus->lost++;
			if (pLaneStatus->lost >= MAX_LOST_FRAMES && !pLaneStatus->reset)
			{
				pLaneStatus->reset = true;
			}
		}

	}
	else
	{
		myprintf(2, 1, "no lanes detected - lane tracking lost! counter increased\n");
		pLaneStatus->lost++;
		if (pLaneStatus->lost >= MAX_LOST_FRAMES && !pLaneStatus->reset)
		{
			// do full reset when lost for more than N frames
			pLaneStatus->reset = true;
			pLaneStatus->k.clear();
			pLaneStatus->b.clear();
		}
	}

	delete[] votes;
}

void CLaneDetectorNavigatorEngine::ProcessLanes(CvSeq* lines, IplImage* pEdges, IplImage *pWorkingImage, bool bShowHoughLine, bool bDisplayImage)
{
	// classify lines to left/right pLaneStatus
	std::vector<CLaneInfo> left, right;

	for (int i = 0; i < lines->total; i++)
    {
        CvPoint* line = (CvPoint*)cvGetSeqElem(lines, i);
		int dx = line[1].x - line[0].x;
		int dy = line[1].y - line[0].y;
		float angle = atan2f(dy, dx) * 180 / CV_PI;

		if (fabs(angle) <= LINE_REJECT_DEGREES)
		{ // reject near horizontal lines
			continue;
		}

		// assume that vanishing point is close to the image horizontal center
		// calculate line parameters: y = kx + b;
		dx = (dx == 0) ? 1 : dx; // prevent DIV/0!
		float k = dy / (float)dx;
		float b = line[0].y - k * line[0].x;

		// assign lane's pLaneStatus based by its midpoint position
		int midx = (line[0].x + line[1].x) / 2;

		if (midx < pWorkingImage->width / 2)
		{
			left.push_back(CLaneInfo(line[0], line[1], angle, k, b));
		}
		else
		{
			right.push_back(CLaneInfo(line[0], line[1], angle, k, b));
		}
    }
	if (bShowHoughLine)
	{
		// show Hough lines
		for	(int i=0; i<right.size(); i++)
		{
			cvLine(pWorkingImage, right[i].m_p0, right[i].m_p1, CV_RGB(0, 0, 255), 2);
		}

		for	(int i=0; i<left.size(); i++)
		{
			cvLine(pWorkingImage, left[i].m_p0, left[i].m_p1, CV_RGB(255, 0, 0), 2);
		}
	}
	ProcessSide(left, pEdges, false);
	ProcessSide(right, pEdges, true);

	// show computed lanes

	CvPoint lt, lb, rt, rb;

	rt.x = pWorkingImage->width * 0.65f;//(rt.y - laneR.b.get()) / laneR.k.get();//temp_frame->width * 0.65f;
	rt.y = m_LaneR.k.get() * rt.x + m_LaneR.b.get();

	rb.x = pWorkingImage->width;//(rb.y - laneR.b.get()) / laneR.k.get();//temp_frame->width;
	rb.y = m_LaneR.k.get() * rb.x + m_LaneR.b.get();

	lt.x = (rt.y - m_LaneL.b.get()) / m_LaneL.k.get();//temp_frame->width * 0;
	lt.y = rt.y;//laneL.k.get() * lt.x + laneL.b.get();

	lb.x = (rb.y - m_LaneL.b.get()) / m_LaneL.k.get();//temp_frame->width * 0.45f;
	lb.y = rb.y;//laneL.k.get() * lb.x + laneL.b.get();
	if (bDisplayImage)
	{
		cvLine(pWorkingImage, rt, rb, CV_RGB(255, 0, 255), 2);
		cvLine(pWorkingImage, lt, lb, CV_RGB(255, 0, 255), 2);
		cvLine(pWorkingImage, lt, rt, CV_RGB(128, 0, 128), 2);
		cvLine(pWorkingImage, lb, rb, CV_RGB(128, 0, 128), 2);
	}
	//CvPoint vanishingPoint;

	m_VanishingPoint.x = -(m_LaneR.b.get() - m_LaneL.b.get()) / (m_LaneR.k.get() - m_LaneL.k.get());	// x coordinate
	m_VanishingPoint.y = m_LaneR.k.get() * m_VanishingPoint.x + m_LaneR.b.get();
	if (bDisplayImage)
	{
		cvLine(pWorkingImage, cvPoint(m_VanishingPoint.x, 0),
							cvPoint(m_VanishingPoint.x, pWorkingImage->height), CV_RGB(255, 255, 255), 2);
	}
	// find out the turn angle
	CvPoint pivotT, pivotB;

	pivotT.x = (lt.x + rt.x) / 2;
	pivotT.y = (lt.y + rt.y) / 2;
	pivotB.x = (lb.x + rb.x) / 2;
	pivotB.y = (lb.y + rb.y) / 2;
	if (bDisplayImage)
	{
		cvLine(pWorkingImage, pivotT, pivotB, CV_RGB(0, 255, 255), 2);
	}
	// find the angle

	m_fTurnAngle = 90 - atan2(pivotB.y - pivotT.y, pivotB.x - pivotT.x) * 180 / CV_PI;
	myprintf(3, 1, "Angle: %4.2f\n", m_fTurnAngle);
#if DEBUGIMG
	char szOut[100];

	sprintf(szOut, "Angle: %4.2f", m_fTurnAngle);
	cvPutText(pWorkingImage, szOut, pivotT, &m_Font, CV_RGB(0, 255, 255));
#endif
}
int32_t CLaneDetectorNavigatorEngine::ProcessImage(IplImage *pFrame, bool bDisplayImage, float &fAngle, CvPoint &vanishingPoint)
//int32_t CLaneDetectorNavigatorEngine::ProcessFrame(IplImage *pFrame, float &fAngle, CvPoint &vanishingPoint)
{
	int32_t nRet = -1;
	double rho = 1;
	double theta = CV_PI / 180;
	CvSeq* pLines = NULL;

	if (pFrame == NULL)
	{
		goto err_out;
	}
	if (m_FrameSize.width == 0 || m_FrameSize.height == 0)
	{
		m_FrameSize.width = pFrame->width;
		m_FrameSize.height = pFrame->height;

		m_HalfFrameSize.width = pFrame->width;
		m_HalfFrameSize.height = pFrame->height / 2;

		m_ROI = cvRect(0, m_HalfFrameSize.height, m_HalfFrameSize.width, m_HalfFrameSize.height);
	}
	else
	{
		if (m_FrameSize.width != pFrame->width || m_FrameSize.height != pFrame->height)
		{
			goto err_out;
		}
	}

	if (m_pHoughStorage == NULL)
	{
		m_pHoughStorage = cvCreateMemStorage(0);
	}

	if (m_pWorkingImage == NULL)
	{
		m_pWorkingImage = cvCreateImage(m_HalfFrameSize, IPL_DEPTH_8U, 3);
	}

	if (m_pGreyImage == NULL)
	{
		m_pGreyImage = cvCreateImage(m_HalfFrameSize, IPL_DEPTH_8U, 1);
	}

	if (m_pEdgesImage == NULL)
	{
		m_pEdgesImage = cvCreateImage(m_HalfFrameSize, IPL_DEPTH_8U, 1);
	}

	// we're interested only in road below horizont - so crop top image portion off
	crop(pFrame, m_pWorkingImage, m_ROI);
	cvCvtColor(m_pWorkingImage, m_pGreyImage, CV_BGR2GRAY); // convert to grayscale

	// Perform a Gaussian blur ( Convolving with 5 X 5 Gaussian) & detect edges
	cvSmooth(m_pGreyImage, m_pGreyImage, CV_GAUSSIAN, 5, 5);
    // till here, it consume 8/30 frame
    {
        CvScalar mu, sigma;
        cvAvgSdv(m_pGreyImage, &mu, &sigma);
        //double otsu_thresh_val = cvThreshold(m_pGreyImage, m_pOTSU, 0, 255, CV_THRESH_BINARY | CV_THRESH_OTSU);

        //cvCanny(m_pGreyImage, m_pGreyImage, L2_CANNY_MIN_TRESHOLD, L2_CANNY_MAX_TRESHOLD, 3);  // 15/30 (src=dest is faster 17/30)
        cvCanny(m_pGreyImage, m_pGreyImage, mu.val[0] - sigma.val[0], mu.val[0] + sigma.val[0], 3);  // 15/30 (src=dest is faster 17/30)
        //cvCanny(m_pGreyImage, m_pGreyImage, otsu_thresh_val * 0.5, otsu_thresh_val, 3);
    }
	//cvCanny(m_pGreyImage, m_pEdgesImage, CANNY_MIN_TRESHOLD, CANNY_MAX_TRESHOLD);

	// do Hough transform to find lanes
	pLines = cvHoughLines2(m_pGreyImage, m_pHoughStorage, CV_HOUGH_PROBABILISTIC,
									rho, theta, HOUGH_TRESHOLD, HOUGH_MIN_LINE_LENGTH, HOUGH_MAX_LINE_GAP);

	ProcessLanes(pLines, m_pGreyImage, m_pWorkingImage, m_bShowLine, bDisplayImage);

	fAngle = m_fTurnAngle;
	vanishingPoint = m_VanishingPoint;

	if (bDisplayImage)
	{
		cvShowImage("Lane-Detector::Edges", m_pGreyImage);
		cvShowImage("Lane-Detector::ColorImage", m_pWorkingImage);

        //PublishDebugImage("mono8", m_pGreyImage);

		PublishDebugImage("bgr8", m_pWorkingImage);
	}
	else
	{
		cvDestroyWindow("Lane-Detector::Edges");
		cvDestroyWindow("Lane-Detector::ColorImage");
	}
#if DEBUGIMG
	cvShowImage("Grey", m_pGreyImage);
	cvShowImage("Edges", m_pEdgesImage);
	cvShowImage("Color", m_pWorkingImage);

	cvMoveWindow("Grey", 0, 0);
	cvMoveWindow("Edges", 0, m_HalfFrameSize.height + 25);
	cvMoveWindow("Color", 0, 2*(m_HalfFrameSize.height + 25));
#endif
err_out:
	return nRet;
}

}
