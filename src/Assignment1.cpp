#include <iostream>
#include <sstream>
#include <string>
#include <stdio.h>
#include <stdlib.h>
#include "rbe510.hpp"
#include "planner.h"
#include "rectification.hpp"
#include "Eigen/Eigen"

using namespace std;

const double Radius=40; // Radius from center of rotation to center of the box (maybe compute based on criteria)
const double nBoxLength=25; //length of box in cm

const double WayDistance=2; // Way point every 10 cm
const int NUM_ROBOTS = 2;


int main(int argc, char *argv[])
{
    //if(argc != 2){
      // printf("\n Usage: %s <ip of server> \n",argv[0]);
       //return 1;
    //}

    string ip = "127.0.0.1";
    FieldComputer fc(ip);
    fc.enableVerbose();
    FieldData data = fc.getFieldData();

	std::vector<pair<double,double>> truePathLeft;
    std::vector<pair<double,double>> truePathRight;


	std::vector<Entity> corners;
    std::vector<Entity> longBox;
    std::vector<Robot> robots;

	double fieldWidth = 231.14;
	double fieldHeight = 109.86;
	Eigen::Matrix<double, 4, 2> p1;
	Eigen::Matrix<double, 4, 2> p2;

    
    double x1,x2,y1,y2;
    int x1p,y1p, x2p, y2p;



    cout << "Connected" << endl;
	while(corners.size() < 4){
        data = fc.getFieldData();
		for(unsigned i = 0; i < data.entities.size(); i++){
			if (data.entities[i].id() >= 200 && data.entities[i].id() <= 203) {
				corners.push_back(data.entities[i]);
			}
		}
		if (corners.size() < 4){
			corners.clear();
		}
	}

cout << "Found all corners" << endl;

    while(longBox.size() < 2){
        cout << "Looking for long box..." << endl;
        data = fc.getFieldData();
        for(unsigned i = 0; i < data.entities.size(); i++){
           if (data.entities[i].id() == 101){
                cout << "Found marker 101" << endl;
                longBox.push_back(data.entities[i]);
            } else if (data.entities[i].id()==102){
                cout << "Found marker 102" << endl;
                longBox.push_back(data.entities[i]);
            }
        }
        if (longBox.size() < 2){
            longBox.clear();
        }   
    }

	int leftStartPosX, leftStartPosY, rightStartPosX, rightStartPosY;

    while(robots.size() < 2) {
        data = fc.getFieldData();
	   for (unsigned i = 0; i < data.robots.size(); i++){
		  if (data.robots[i].id() == 0){
            robots.push_back(data.robots[i]);
			// Robot 1 (left)
			leftStartPosX = (int)data.robots[i].x();
			leftStartPosY = (int)data.robots[i].y();
            cout << "Left robot must return to: X:"<<leftStartPosX<<" Y:"<<leftStartPosY<<endl;
		  }
		  if (data.robots[i].id() == 3){
			// Robot 4 (right)
            robots.push_back(data.robots[i]);
			rightStartPosX = (int)data.robots[i].x();
			rightStartPosY = (int)data.robots[i].y();
            cout << "Right robot must return to: X:"<<rightStartPosX<<" Y:"<<rightStartPosY<<endl;
		  }
        }
        if (robots.size() < 2) {
            robots.clear();
        }
	}

    

    cout << "Found box position" << endl;

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

    x1p = 0.5*(longBox[0].x() + longBox[1].x());
    y1p = 0.5*(longBox[0].y() + longBox[1].y());

   
    
 

    cout<<"Box Center (px) X: " << x1p << " Y: " << y1p << endl;
 
    Matrix<double, 1, 3> Startposition= correction.correctPerspectiveMetric((double)x1p,(double)y1p);
    x1=Startposition(0,0);
    y1=Startposition(0,1) + 10;
    cout<<"Box Center Start (cm) X: " << x1 << " Y: " << y1 << endl;

    x2 = x1 + 60;
    y2 = y1 - 30 - 10;
    //theta2=90;
    Matrix<double, 1, 3> EndpositionPx= correction.correctPerspectivePixels((double)x2,(double)y2);
    cout<<"Box Center End (px) X: " << (int)EndpositionPx(0,0) << " Y: " << (int)EndpositionPx(0,1)  << endl;
    cout<<"Box Center End (cm) X: " << x2 << " Y: " << y2 << endl;
    
    double dbDist=sqrt(pow(x1-x2,2)+pow(y1-y2,2));
    double dbTheta=2*asin(dbDist/(2*Radius));
    double dbArcLength=Radius*dbTheta;
    
    double dbNumberWayPoints=floor(dbArcLength/WayDistance);
    
    double dbAlpha;
    double dbCenterRotX;
    double dbCenterRotY;
    double dbGamma;
    
    if (x1<x2 && y1<y2) {
        dbAlpha=atan((y2-y1)/(x2-x1));
        dbCenterRotX=x1-Radius*cos(PI/2+dbTheta/2-dbAlpha);
        dbCenterRotY=y1+Radius*sin(PI/2+dbTheta/2-dbAlpha);
        
        dbGamma=2*PI+atan((dbCenterRotY-y1)/(dbCenterRotX-x1));
    }
    else if (x1>x2 && y1<y2){
        dbAlpha=-atan((y2-y1)/(x2-x1));
        dbCenterRotX=x1-Radius*cos(PI/2-dbTheta/2-dbAlpha);
        dbCenterRotY=y1-Radius*sin(PI/2-dbTheta/2-dbAlpha);
        dbGamma=PI/2-dbTheta/2-dbAlpha;
    }
    else if (x1>x2 && y1>y2){
        dbAlpha=atan((y2-y1)/(x2-x1));
        dbCenterRotX=x1+Radius*sin(-dbTheta/2+dbAlpha);
        dbCenterRotY=y1-Radius*cos(-dbTheta/2+dbAlpha);
        dbGamma=PI/2+dbAlpha-dbTheta/2;
    }
    else if (x1<x2 && y1>y2){
        dbAlpha=-atan((y2-y1)/(x2-x1));
        dbCenterRotX=x1+Radius*cos(PI/2-dbTheta/2-dbAlpha);
        dbCenterRotY=y1+Radius*sin(PI/2-dbTheta/2-dbAlpha);
        dbGamma=3*PI/2-dbTheta/2-dbAlpha;
    }
    
    double nX1Left = dbCenterRotX+(Radius-nBoxLength/2)*cos(dbGamma);
    double nY1Left = dbCenterRotY+(Radius-nBoxLength/2)*sin(dbGamma);
    
    double nX2Left = dbCenterRotX+(Radius-nBoxLength/2)*cos(dbGamma+dbTheta);
    double nY2Left = dbCenterRotY+(Radius-nBoxLength/2)*sin(dbGamma+dbTheta);
    
    double nX1Right = dbCenterRotX+(Radius+nBoxLength/2)*cos(dbGamma);
    double nY1Right = dbCenterRotY+(Radius+nBoxLength/2)*sin(dbGamma);
    
    double nX2Right = dbCenterRotX+(Radius+nBoxLength/2)*cos(dbGamma+dbTheta);
    double nY2Right = dbCenterRotY+(Radius+nBoxLength/2)*sin(dbGamma+dbTheta);
    
    Planner P;
    
    Path tCenterBox=P.CircleArc(make_pair(x1,y1),make_pair(x2,y2),Radius,dbNumberWayPoints);
    Path tLeftWheel=P.CircleArc(make_pair(nX1Left,nY1Left),make_pair(nX2Left,nY2Left),Radius-nBoxLength/2,dbNumberWayPoints);
    Path tRightWheel=P.CircleArc(make_pair(nX1Right,nY1Right),make_pair(nX2Right,nY2Right),Radius+nBoxLength/2,dbNumberWayPoints);
	
	
	cout << "Moving to Box..." << endl;
	


for (Path::iterator itl=tLeftWheel.begin() , itr=tRightWheel.begin(); itl !=tLeftWheel.end() && itr !=tRightWheel.end(); ++itl, ++itr){
    data = fc.getFieldData();
    std::ostringstream leftCommand;
    std::ostringstream rightCommand;

    cout<<"Left Robot: " << itl->first<<'\t'<<itl->second<<endl;
    cout<<"Right Robot: " << itr->first<<'\t'<<itr->second<<endl;

    Matrix<double,1,3> rightCoordinate = correction.correctPerspectivePixels(itr->first,itr->second);
    rightCommand << "./g2p 127.0.0.1 3 " << (int)rightCoordinate(0,0) << " " << (int)rightCoordinate(0,1);
    cout << rightCommand.str() << endl;
    const char *rc = rightCommand.str().c_str();
    std::system(rc);

	
    Matrix<double,1,3> leftCoordinate = correction.correctPerspectivePixels(itl->first,itl->second);
    leftCommand << "./g2p 127.0.0.1 0 " << (int)leftCoordinate(0,0) << " " << (int)leftCoordinate(0,1);
    cout << leftCommand.str() << endl;
    const char *lc = leftCommand.str().c_str();
    std::system(lc);

    cout<<endl;
	
	for(unsigned i = 0; i < data.robots.size(); i++){
		if (data.robots[i].id() == 0) {
            Matrix<double,1,3> leftPosActualCm = correction.correctPerspectiveMetric(data.robots[i].x(),data.robots[i].y());
            cout << "Left Robot (Actual): " << leftPosActualCm(0,0) <<'\t' <<leftPosActualCm(0,1) <<endl;
			truePathLeft.push_back(make_pair(leftPosActualCm(0,0),leftPosActualCm(0,1)));
		}
        if (data.robots[i].id() == 3) {
            Matrix<double,1,3> rightPosActualCm = correction.correctPerspectiveMetric(data.robots[i].x(),data.robots[i].y());
            cout << "Right Robot (Actual): " << rightPosActualCm(0,0) <<'\t' <<rightPosActualCm(0,1) <<endl;
            truePathRight.push_back(make_pair(rightPosActualCm(0,0),rightPosActualCm(0,1)));
        }
	}
}

std::ostringstream leftCommand;
std::ostringstream rightCommand;

cout << "Box Destination Reached" << endl;


cout << "Returning to Start" << endl;

leftCommand << "./g2p 127.0.0.1 0 " << (int)leftStartPosX << " " << (int)leftStartPosY;
cout << leftCommand.str() << endl;
const char *lc = leftCommand.str().c_str();
std::system(lc);

rightCommand << "./g2p 127.0.0.1 3 " << (int)rightStartPosX << " " << (int)rightStartPosY;
cout << rightCommand.str() << endl;
const char *rc = rightCommand.str().c_str();
std::system(rc);

cout<<endl;


cout << "All done!" << endl;

cout << "Precomputed path from start position to end position :" << endl;	
cout << endl;
cout << "     Center of the box :" << endl;	
for (Path::iterator it=tCenterBox.begin();it!=tCenterBox.end(); ++it) {
    cout<<it->first<<'\t'<<it->second<<endl;   //display path
}
    
cout<<endl;
cout << "     Left robot :" << endl;	    
for (Path::iterator it=tLeftWheel.begin();it!=tLeftWheel.end(); ++it) {
    cout<<it->first<<'\t'<<it->second<<endl;   //display path
}
    
cout<<endl;
cout << "     Right robot :" << endl;	     
for (Path::iterator it=tRightWheel.begin();it!=tRightWheel.end(); ++it) {
    cout<<it->first<<'\t'<<it->second<<endl;   //display path
}

cout << "Path followed by the robots :" << endl;
cout<<endl;	
cout << "Left robot :" << endl;		
for (std::vector<pair<double,double>>::iterator it=truePathLeft.begin();it!=truePathLeft.end(); ++it) {
        cout<<it->first<<'\t'<<it->second<<endl;   //display path
}
    
cout<<endl;
cout << "Right robot :" << endl;    
for (std::vector<pair<double,double>>::iterator it=truePathRight.begin();it!=truePathRight.end(); ++it) {
        cout<<it->first<<'\t'<<it->second<<endl;   //display path
}
    
cout<<endl;
}
