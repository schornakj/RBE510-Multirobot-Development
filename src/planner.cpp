#include "planner.h"

Path Planner::CircleArc(Location t_start, Location t_goal, double n_Radius, double db_NumberWayPoints)
{
    double nX1=t_start.first;
    double nX2=t_goal.first;
    double nY1=t_start.second;
    double nY2=t_goal.second;
    
    double dbDist=sqrt(pow(nX1-nX2,2)+pow(nY1-nY2,2));
    double dbTheta=2*asin(dbDist/(2*n_Radius));
    
    double dbAlpha;
    double dbCenterRotX;
    double dbCenterRotY;
    double dbGamma;
    Path tArc;
    
    tArc.push_back(make_pair(nX1,nY1));
    
    if (nX1<nX2 && nY1<nY2) {
        dbAlpha=atan((nY2-nY1)/(nX2-nX1));
        dbCenterRotX=nX1-n_Radius*cos(PI/2+dbTheta/2-dbAlpha);
        dbCenterRotY=nY1+n_Radius*sin(PI/2+dbTheta/2-dbAlpha);
        
        dbGamma=2*PI+atan((dbCenterRotY-nY1)/(dbCenterRotX-nX1));
        
        for (double i=0; i<db_NumberWayPoints; ++i) {
            tArc.push_back(make_pair((dbCenterRotX+n_Radius*cos(dbGamma+dbTheta*i/db_NumberWayPoints)),(dbCenterRotY+n_Radius*sin(dbGamma+dbTheta*i/db_NumberWayPoints))));
        }
    }
    else if (nX1>nX2 && nY1<nY2){
        dbAlpha=-atan((nY2-nY1)/(nX2-nX1));
        dbCenterRotX=nX1-n_Radius*cos(PI/2-dbTheta/2-dbAlpha);
        dbCenterRotY=nY1-n_Radius*sin(PI/2-dbTheta/2-dbAlpha);
        
        dbGamma=PI/2-dbTheta/2-dbAlpha;
        
        for (double i=0; i<db_NumberWayPoints; ++i) {
            tArc.push_back(make_pair((dbCenterRotX+n_Radius*cos(dbGamma+dbTheta*i/db_NumberWayPoints)),(dbCenterRotY+n_Radius*sin(dbGamma+dbTheta*i/db_NumberWayPoints))));
        }
    }
    else if (nX1>nX2 && nY1>nY2){
        dbAlpha=atan((nY2-nY1)/(nX2-nX1));
        dbCenterRotX=nX1+n_Radius*sin(-dbTheta/2+dbAlpha);
        dbCenterRotY=nY1-n_Radius*cos(-dbTheta/2+dbAlpha);
        
        dbGamma=PI/2+dbAlpha-dbTheta/2;
        
        for (double i=0; i<db_NumberWayPoints; ++i) {
            tArc.push_back(make_pair((dbCenterRotX+n_Radius*cos(dbGamma+dbTheta*i/db_NumberWayPoints)),(dbCenterRotY+n_Radius*sin(dbGamma+dbTheta*i/db_NumberWayPoints))));
        }
    }
    else if (nX1<nX2 && nY1>nY2){
        dbAlpha=-atan((nY2-nY1)/(nX2-nX1));
        dbCenterRotX=nX1+n_Radius*cos(PI/2-dbTheta/2-dbAlpha);
        dbCenterRotY=nY1+n_Radius*sin(PI/2-dbTheta/2-dbAlpha);
        
        dbGamma=3*PI/2-dbTheta/2-dbAlpha;
        
        for (double i=0; i<db_NumberWayPoints; ++i) {
            tArc.push_back(make_pair((dbCenterRotX+n_Radius*cos(dbGamma+dbTheta*i/db_NumberWayPoints)),(dbCenterRotY+n_Radius*sin(dbGamma+dbTheta*i/db_NumberWayPoints))));
        }
    }
    tArc.push_back(make_pair(nX2,nY2));
    return tArc;

}


void Planner::SetGrid(Location t_start,Location t_goal)
{
    int nX1=t_start.first;
    int nX2=t_goal.first;
    int nY1=t_start.second;
    int nY2=t_goal.second;
    
    int nXStep=Subdivide(nX1,nX2,ArenaWidth/MinSubdiv);
    int nYStep=Subdivide(nY1,nY2,ArenaLength/MinSubdiv);
    
    int nNumberXStepsOrig=min(nX1,nX2)/nXStep; // Euclidian division
    int nNumberYStepsOrig=min(nY1,nY2)/nYStep; // Euclidian division
    
    int nFirstX=min(nX1,nX2)-nNumberYStepsOrig*nXStep;
    int nFirstY=min(nY1,nY2)-nNumberYStepsOrig*nYStep;
    
    int nNumberXSteps=(ArenaWidth-nNumberXStepsOrig)/nXStep;
    int nNumberYSteps=(ArenaWidth-nNumberYStepsOrig)/nYStep;
    
    for (int i=0; i<nNumberXSteps; ++i) {
        for (int j=0; j<nNumberYSteps; ++j) {
            m_tGrid.insert(make_pair(nFirstX+i*nXStep,nFirstY+j*nYStep));
        }
    }
    
}

int Planner::Subdivide(int n_x1, int n_x2, int n_MaxSubdiv)
{
    int nLength=abs(n_x1-n_x2);
    if(nLength%2!=0) nLength-=1;
    int nRemainder=0;
    int nStep=0;
    for (int i=n_MaxSubdiv; i>0; --i) {
        nRemainder=nLength%i;
        nStep=nLength/i; // Euclidian division
        if(nStep>=100){
            if(nRemainder==0) break;
            else {
                nStep+=nRemainder;
                break;
            }
        }
    }
    return nStep;
}

Planner::Planner(){
    
}

