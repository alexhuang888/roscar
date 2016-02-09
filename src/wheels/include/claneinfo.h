#ifndef __CLANEINFO_H__
#define __CLANEINFO_H__
#include "opencv2/imgproc/imgproc.hpp"
namespace yisys_roswheels
{

class CExpMovingAverage
{
private:
	double alpha; // [0;1] less = more stable, more = less stable
    double oldValue;
	bool unset;
public:
    CExpMovingAverage()
    {
        this->alpha = 0.2;
		unset = true;
    }

	void clear(void)
	{
		unset = true;
	}

    void add(double value)
    {
        if (unset)
        {
            oldValue = value;
			unset = false;
        }
        double newValue = oldValue + alpha * (value - oldValue);
        oldValue = newValue;
    }

	double get(void)
	{
		return oldValue;
	}
};

class CLaneStatus
{
public:
	CLaneStatus():reset(true),lost(0){}
	CExpMovingAverage k, b;
	bool reset;
	int lost;
};

class CLaneInfo
{
public:
	CLaneInfo()
	{

	}
	CLaneInfo(CvPoint a, CvPoint b, float angle, float kl, float bl): m_p0(a), m_p1(b), m_angle(angle),
		m_votes(0), m_visited(false), m_found(false), m_k(kl), m_b(bl)
	{
	}

	CvPoint m_p0, m_p1;
	int m_votes;
	bool m_visited, m_found;
	float m_angle, m_k, m_b;
};

class CMyLine2DBase
{
public:

    CMyLine2DBase()
    {
        m_bVertical = false;
    };
	CMyLine2DBase(cv::Point p1, cv::Point p2)
	{
		CvPoint xp1, xp2;

		xp1.x = p1.x;
		xp1.y = p1.y;

		xp2.x = p2.x;
		xp2.y = p2.y;

		SetMyLine2DBase(xp1, xp2);
	}


    CMyLine2DBase(CvPoint p1, CvPoint p2)
    {
		SetMyLine2DBase(p1, p2);
    };

    float FindX(float fY)
    {
        if (m_bVertical)
        {
            return m_Point1.x;
        }
        else
        {
            return m_Slope != 0 ? (fY - m_B) / m_Slope : 0;
        }
        return 0;
    }
	float Angle(void) { return m_fAngle; };
	float Length(void) { return m_fLength; }
	float Slope(void) { return m_Slope; }
	float B(void) { return m_B; };
    CvPoint m_Point1;
    CvPoint m_Point2;
    float m_fLength;
    float m_fAngle;
    float m_Slope, m_B;
    bool m_bVertical;
private:
	void SetMyLine2DBase(CvPoint p1, CvPoint p2)
    {
		m_Point1 = p1;
		m_Point2 = p2;

		// Compute the line parameters
		float diffX = p2.x - p1.x;
		float diffY = p2.y - p1.y;

        if (diffY < 0)
        {
            diffX = p1.x - p2.x;
            diffY = p1.y - p2.y;
        }
		if (diffX != 0)
		{
			m_Slope = diffY / diffX; // Slope
			m_B = p1.y - m_Slope * p1.x; // Intercept
			m_bVertical = false;
        }
		else
		{
            m_bVertical = true;
			m_Slope = 0;
            m_B = p1.y - m_Slope * p1.x; // Intercept
		}

		m_fLength = sqrt(diffX * diffX + diffY * diffY);
        m_fAngle = atan2(diffY, diffX) * 180 / CV_PI - 90;
    };
};
};

#endif
