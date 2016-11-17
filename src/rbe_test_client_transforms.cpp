#include <iostream>
#include <string>
#include "rbe510.hpp"
#include "rectification.hpp"
#include "bezier.hpp"
#include "Eigen/Dense"

/*
#include <opencv2/core.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/features2d.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/calib3d.hpp>
#include <opencv2/xfeatures2d.hpp>
*/

using namespace std;
using namespace Eigen;
using namespace cv;
using std::vector;

/*
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
*/
/*
class CubicBezier {
public:
	Point2f controlPoints[4];

	QuadraticBezier(Point2f p1, Point2f p2, Point2f p3, Point2f p4){
		this->controlPoints[0]=p1;
		this->controlPoints[1]=p2;
		this->controlPoints[2]=p3;
		this->controlPoints[3]=p4;
	}

	Point2f GetPoint(float t) {
		float xa = getPt(this->controlPoints[0].x, this->controlPoints[1].x,t);
		float ya = getPt(this->controlPoints[0].y, this->controlPoints[1].y,t);
		float xb = getPt(this->controlPoints[1].x, this->controlPoints[2].x,t);
		float yb = getPt(this->controlPoints[1].y, this->controlPoints[2].y,t);
		float xc = getPt(this->controlPoints[2].x, this->controlPoints[3].x,t);
		float yc = getPt(this->controlPoints[2].y, this->controlPoints[3].y,t);

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
*/


int main(int argc, char *argv[])
{
	// Connect to the field server
	if(argc != 2){
		printf("\n Usage: %s <ip of server> \n",argv[0]);
		return 1;
	}

	string ip = string(argv[1]);
	FieldComputer fc(ip);
	fc.enableVerbose();
	FieldData data = fc.getFieldData();

	std::vector<Entity> corners;
	double fieldWidth = 231.14;
	double fieldHeight = 109.86;
	Eigen::Matrix<double, 4, 2> p1;
	Eigen::Matrix<double, 4, 2> p2;

	// Find all four corner markers
	while(corners.size() < 4){
		for(unsigned i = 0; i < data.entities.size(); i++){
			if (data.entities[i].id() >= 200 && data.entities[i].id() <= 203) {
				corners.push_back(data.entities[i]);
			}
		}
		if (corners.size() < 4){
			corners.clear();
		}	
	}

	// Get the coordinates of each of the corners
	for(unsigned i = 0; i < corners.size(); i++) {
		cout << "Corner fount at ID: " << corners[i].id() << endl;

		if (corners[i].id() == 200) {
			// upper left
			p1(1,0) = corners[i].x();
			p1(1,1) = corners[i].y();
		}
		else if (corners[i].id() == 201) {
			// lower left
			p1(0,0) = corners[i].x();
			p1(0,1) = corners[i].y();
		}
		else if (corners[i].id() == 202) {
			// upper right
			p1(2,0) = corners[i].x();
			p1(2,1) = corners[i].y();
		}
		else if (corners[i].id() == 203) {
			// lower right
			p1(3,0) = corners[i].x();
			p1(3,1) = corners[i].y();
		}

	}

	// Define the real-world positions corrensponding to each corner
	p2 << 0, 0, 0, fieldHeight, fieldWidth, fieldHeight, fieldWidth, 0;

	PerspectiveCorrection correction(p1, p2);

	//cout << "Point 2: " << correction.correctPerspectiveMetric(correction.pointsToMatrix(p1(1,0),p1(1,1))) << endl;
	cout << "Point 2: " << correction.correctPerspectiveMetric(p1(1,0),p1(1,1)) << endl;
	cout << "Test: " << correction.correctPerspectivePixels(0,0) << endl;

	//Eigen::Matrix<double, 8, 9> A;



	//QuadraticBezier trajectory(Point2f(0,0),Point2f(50,50),Point2f(100,100));
	CubicBezier trajectory(Point2f(0,0),Point2f(0,50),Point2f(100,50),Point2f(100,100));

	Point2f last(0,0);
	for(float i = 0; i <= 1; i += 0.005) {
		Point2f currentPoint = trajectory.GetPoint(i);
		cout << "X: " << currentPoint.x << " Y: " << currentPoint.y << endl;
		float delta = sqrt(pow(currentPoint.x - last.x,2) + pow(currentPoint.y - last.y,2));
		//cout << "Delta: " << delta << endl;
		last = currentPoint;
	}
	


	for(unsigned i = 0; i < data.robots.size(); i++){
		cout << " - Robot id: " << data.robots[i].id() << endl;
		cout << "x: " << data.robots[i].x() << endl;
		cout << "y: " << data.robots[i].y() << endl;
		Matrix<double,1,3> metricPosition = correction.correctPerspectiveMetric((double)data.robots[i].x(),(double)data.robots[i].y());
		cout << "X (cm): " << metricPosition(0) << endl;
		cout << "Y (cm): " << metricPosition(1) << endl;

		cout << "theta: " << data.robots[i].theta() << endl;
		//cout << "width: " << data.robots[i].width() << endl;
		//cout << "height: " << data.robots[i].height() << endl;
		//fc.arcadeDrive(data.robots[i].id(), 1.0, -1.0);
		//sleep(1);
		//fc.arcadeDrive(data.robots[i].id(), 0.0, 0.0);
	}


	return 0;
}