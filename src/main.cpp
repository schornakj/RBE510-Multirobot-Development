
#include "planner.h"
#include "rbe510.hpp"
#include "rectification.hpp"
#include "Eigen/Dense"

using namespace std;

const double Radius=200; // Radius from center of rotation to center of the box (maybe compute based on criteria)
const double nBoxLength=100; //length of box in cm
const double WayDistance=10; // Way point every 10 cm

int main(int argc, char *argv[])
{
    //if(argc != 2){
    //    printf("\n Usage: %s <ip of server> \n",argv[0]);
    //    return 1;
    //}

    string ip = "127.0.0.1";
    //string ip = string(argv[1]);
	FieldComputer fc(ip);
	fc.enableVerbose();
	FieldData data = fc.getFieldData();

	std::vector<Entity> corners;
	double fieldWidth = 231.14;
	double fieldHeight = 109.86;
	Eigen::Matrix<double, 4, 2> p1;
	Eigen::Matrix<double, 4, 2> p2;
	const double nBoxLength=100; //length of box in pixels
    double x1,x2,y1,y2,x1p,y1p;
    double theta1,theta2;


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



   
    
    x1p=0.5*(data.entities[101].x()+data.entities[102].x()); // Pixels
    y1p=0.5*(data.entities[101].y()+data.entities[102].y()); // Pixels
    Matrix<double, 1, 3> Startposition= correction.correctPerspectiveMetric(x1p,y1p);
    x1=Startposition(0,0);
    y1=Startposition(0,1);

    theta1=0.5*(data.entities[101].theta()+data.entities[102].theta());
    x2=x1+(30.5*2);
    y2=y1+30.5;
    theta2=60;
//    cout<<"Start Location x y for center of box in pixels :";
//    cin>>x1>>y1;
//    cout<<"Start Orientation theta w.r.t. horizontal in degrees (in ]-90;90[ ):";
//    cin>>theta1;
//    cout<<"Goal Location :";
//    cin>>x2>>y2;
//    cout<<"Goal Orientation theta w.r.t. horizontal in degrees:";
//    cin>>theta2;
    
    theta1*=PI/180;
    theta2*=PI/180;
    
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

for (Path::iterator itl=tLeftWheel.begin() , itr=tRightWheel.begin(); itl !=tLeftWheel.end() && itr !=tRightWheel.end(); ++itl, ++itr){
    cout<<itl->first<<'\t'<<itl->second<<endl;
    cout<<itr->first<<'\t'<<itr->second<<endl;
    cout<<endl;
}
 cout<<endl;

 
//     for (Path::iterator it=tCenterBox.begin();it!=tCenterBox.end(); ++it) {
//         cout<<it->first<<'\t'<<it->second<<endl;   //display path

//     }
    
//     cout<<endl;
    
//    for (Path::iterator it=tLeftWheel.begin();it!=tLeftWheel.end(); ++it) {
//        cout<<it->first<<'\t'<<it->second<<endl;   //display path
//    }
   
//    cout<<endl;
   
//    for (Path::iterator it=tRightWheel.begin();it!=tRightWheel.end(); ++it) {
//        cout<<it->first<<'\t'<<it->second<<endl;   //display path
//    }
   
//    cout<<endl;
//     //Reorient(theta2); to be written (basically not care about orientation until the end and rotate until OK)
}
