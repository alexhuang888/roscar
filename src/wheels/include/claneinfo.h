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
};

#endif
