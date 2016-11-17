#ifndef BEZIER_HPP
#define BEZIER_HPP

#include <opencv2/core.hpp>
#include "math.h"
#include <stdio.h>

using namespace std;
using namespace cv;

//#define PI 3.14159;

class QuadraticBezier {
public:
	Point2f controlPoints[3];

	QuadraticBezier(Point2f p1, Point2f p2, Point2f p3){
		this->controlPoints[0]=p1;
		this->controlPoints[1]=p2;
		this->controlPoints[2]=p3;
	}

	Point2f GetPoint(float t) {
		float xa = getPt(this->controlPoints[0].x, this->controlPoints[1].x,t);
		float ya = getPt(this->controlPoints[0].y, this->controlPoints[1].y,t);
		float xb = getPt(this->controlPoints[1].x, this->controlPoints[2].x,t);
		float yb = getPt(this->controlPoints[1].y, this->controlPoints[2].y,t);

		float x = getPt(xa, xb, t);
		float y = getPt(ya, yb, t);

		return Point2f(y,x);
	}
private:

	float getPt(float n1, float n2, float t) {
		float diff = n2 - n1;
		return n1 + diff*t;
	}

};

class CubicBezier {
public:
	Point2f controlPoints[4];

	CubicBezier(Point2f p1, Point2f p2, Point2f p3, Point2f p4){

		//cout << "P1: " << p1 << " P2: " << p2 << " P3: " << p3 << " P4: " << p4 << endl;
		//cout << "Making a curve starting at [" << p1.x << "," << p1.y << "]cm and ending at [" << p2.x << "," << p2.y << "]" << endl;

		this->controlPoints[0]=p1;
		this->controlPoints[1]=p2;
		this->controlPoints[2]=p3;
		this->controlPoints[3]=p4;
	}

	CubicBezier(float p1x, float p1y, float p1theta, float p4x, float p4y, float p4theta, float curveFactor){
		//cout << "p1theta: " << p1theta << " p4theta: " << p4theta << endl;
		cout << "#2 Making a curve starting at [" << p1x << "," << p1y << "]cm and ending at [" << p4x << "," << p4y << "]" << endl;
		float intermediateOffset = curveFactor * sqrt(pow(p4x-p1x,2) + pow(p4y-p1y,2));

		float p2x = p1x + intermediateOffset * cos(p1theta*M_PI/180);
		float p2y = p1y + intermediateOffset * sin(p1theta*M_PI/180);

		float p3x = p4x - intermediateOffset * cos(p4theta*M_PI/180);
		float p3y = p4y - intermediateOffset * sin(p4theta*M_PI/180);

		//CubicBezier(Point2f(p1y,p1x),Point2f(p2y,p2x),Point2f(p3y,p3x),Point2f(p4y,p4x));

		this->controlPoints[0]=Point2f(p1y,p1x);
		this->controlPoints[1]=Point2f(p2y,p2x);
		this->controlPoints[2]=Point2f(p3y,p3x);
		this->controlPoints[3]=Point2f(p4y,p4x);
	}

	Point2f GetPoint(float t) {
		float xa = getPt(this->controlPoints[0].x, this->controlPoints[1].x,t);
		float ya = getPt(this->controlPoints[0].y, this->controlPoints[1].y,t);
		float xb = getPt(this->controlPoints[1].x, this->controlPoints[2].x,t);
		float yb = getPt(this->controlPoints[1].y, this->controlPoints[2].y,t);
		float xc = getPt(this->controlPoints[2].x, this->controlPoints[3].x,t);
		float yc = getPt(this->controlPoints[2].y, this->controlPoints[3].y,t);

		float xab = getPt(xa, xb, t);
		float yab = getPt(ya, yb, t);
		float xbc = getPt(xb, xc, t);
		float ybc = getPt(yb, yc, t);

		float x = getPt(xab, xbc, t);
		float y = getPt(yab, ybc, t);

		return Point2f(y,x);
	}
private:

	float getPt(float n1, float n2, float t) {
		float diff = n2 - n1;
		return n1 + diff*t;
	}

};



#endif