#ifndef RECTIFICATION_HPP
#define RECTIFICATION_HPP

#include <iostream>
#include <string>
//#include "rbe510.hpp"
#include "Eigen/Dense"
#include <opencv2/core.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/features2d.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/calib3d.hpp>
#include <opencv2/xfeatures2d.hpp>

using namespace std;
using namespace Eigen;
using namespace cv;
using std::vector;

class PerspectiveCorrection {
	private:
		Matrix<double, 3, 3> transform;
		Matrix<double, 3, 3> invtransform;

	public:
		PerspectiveCorrection(Matrix<double, 4, 2> p1, Matrix<double, 4, 2> p2) {
			Point2f inputQuad[4];
			Point2f outputQuad[4];

			// Convert matrices to Point2f arrays for OpenCV
			inputQuad[0] = Point2f((float)p1(0,0),(float)p1(0,1));
			inputQuad[1] = Point2f((float)p1(1,0),(float)p1(1,1));
			inputQuad[2] = Point2f((float)p1(2,0),(float)p1(2,1));
			inputQuad[3] = Point2f((float)p1(3,0),(float)p1(3,1));

			outputQuad[0] = Point2f((float)p2(0,0),(float)p2(0,1));
			outputQuad[1] = Point2f((float)p2(1,0),(float)p2(1,1));
			outputQuad[2] = Point2f((float)p2(2,0),(float)p2(2,1));
			outputQuad[3] = Point2f((float)p2(3,0),(float)p2(3,1));

			// Calculate the 3x3 perspective transform that maps the points in p1 to the points in p2
			Mat tform(3,3,CV_32FC1);
			tform = getPerspectiveTransform(inputQuad, outputQuad);

			// Convert from OpenCV-style matrix to Eigen-style matrix
			Matrix<double, 3, 3> tform2;
			tform2 << tform.at<double>(0,0), tform.at<double>(0,1), tform.at<double>(0,2),
			tform.at<double>(1,0), tform.at<double>(1,1), tform.at<double>(1,2),
			tform.at<double>(2,0), tform.at<double>(2,1), tform.at<double>(2,2);

			this->transform=tform2;
			this->invtransform=tform2.inverse();
		}

		Matrix<double, 1, 3> correctPerspectiveMetric(Matrix<double, 1, 3> point) {
			// To transform a pixel coordinate to a rectified metric coordinate, multiply the coordinate matrix by the transform and
			// scale the entire matrix by the last element in the matrix such that the last element equals 1.
			// If the transform was correctly constructed, then transforming the points in p1 will produce the corresponding points in p2.
			Matrix<double,1,3> pointtransform = point*this->transform.transpose();
			Matrix<double,1,3> pointtransformScaled = pointtransform*(1/pointtransform(0,2));
			return pointtransformScaled;
		}

		Matrix<double, 1, 3> pointsToMatrix(double x, double y) {
			Matrix<double, 1, 3> points;
			points << x, y, 1;
			return points;
		}

		Matrix<double, 1, 3> correctPerspectiveMetric(double x, double y) {
			return this->correctPerspectiveMetric(this->pointsToMatrix(x,y));
		}

		Matrix<double,1,3> correctPerspectivePixels(Matrix<double,1,3> point) {
			// Apply inverse transform to convert from centimeters to pixels
			Matrix<double,1,3> pointtransform = point*this->invtransform.transpose();
			Matrix<double,1,3> pointtransformScaled = pointtransform*(1/pointtransform(0,2));
			return pointtransformScaled;
		}

		Matrix<double,1,3> correctPerspectivePixels(double x, double y) {
			return this->correctPerspectivePixels(this->pointsToMatrix(x,y));
		}

};
#endif