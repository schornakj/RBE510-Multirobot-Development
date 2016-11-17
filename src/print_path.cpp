
#include "planner.h"

using namespace std;

const double Radius=200; // Radius from center of rotation to center of the box (maybe compute based on criteria)
const double nBoxLength=100; //length of box in cm
const double WayDistance=10; // Way point every 10 cm

int main(int argc, char *argv[])
{
    double x1,x2,y1,y2;
    double theta1,theta2;
    
    x1=260;
    y1=100;
    theta1=0;
    x2=100;
    y2=260;
    theta2=60;

    
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
    
    for (Path::iterator it=tCenterBox.begin();it!=tCenterBox.end(); ++it) {
        cout<<it->first<<'\t'<<it->second<<endl;   //display path
    }
    
    cout<<endl;
    
    for (Path::iterator it=tLeftWheel.begin();it!=tLeftWheel.end(); ++it) {
        cout<<it->first<<'\t'<<it->second<<endl;   //display path
    }
    
    cout<<endl;
    
    for (Path::iterator it=tRightWheel.begin();it!=tRightWheel.end(); ++it) {
        cout<<it->first<<'\t'<<it->second<<endl;   //display path
    }
    
    cout<<endl;
}
