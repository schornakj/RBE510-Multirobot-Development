
#include "planner.h"

TVecCoord Planner::CircleArc(Coordinate t_start, Coordinate t_goal, double n_Radius, double db_NumberWayPoints)
{
    float nX1=t_start.first;
    float nX2=t_goal.first;
    float nY1=t_start.second;
    float nY2=t_goal.second;
    
    float dbDist=sqrt(pow(nX1-nX2,2)+pow(nY1-nY2,2));
    float dbTheta=2*asin(dbDist/(2*n_Radius));
    
    float dbAlpha;
    float dbCenterRotX;
    float dbCenterRotY;
    float dbGamma;
    TVecCoord tArc;
    
    tArc.push_back(make_pair(nX1,nY1));
    
    if (nX1<nX2 && nY1<nY2) {
        dbAlpha=atan((nY2-nY1)/(nX2-nX1));
        dbCenterRotX=nX1-n_Radius*cos(PI/2+dbTheta/2-dbAlpha);
        dbCenterRotY=nY1+n_Radius*sin(PI/2+dbTheta/2-dbAlpha);
        
        dbGamma=2*PI+atan((dbCenterRotY-nY1)/(dbCenterRotX-nX1));
        
        for (float i=0; i<db_NumberWayPoints; ++i) {
            tArc.push_back(make_pair((dbCenterRotX+n_Radius*cos(dbGamma+dbTheta*i/db_NumberWayPoints)),(dbCenterRotY+n_Radius*sin(dbGamma+dbTheta*i/db_NumberWayPoints))));
        }
    }
    else if (nX1>nX2 && nY1<nY2){
        dbAlpha=-atan((nY2-nY1)/(nX2-nX1));
        dbCenterRotX=nX1-n_Radius*cos(PI/2-dbTheta/2-dbAlpha);
        dbCenterRotY=nY1-n_Radius*sin(PI/2-dbTheta/2-dbAlpha);
        
        dbGamma=PI/2-dbTheta/2-dbAlpha;
        
        for (float i=0; i<db_NumberWayPoints; ++i) {
            tArc.push_back(make_pair((dbCenterRotX+n_Radius*cos(dbGamma+dbTheta*i/db_NumberWayPoints)),(dbCenterRotY+n_Radius*sin(dbGamma+dbTheta*i/db_NumberWayPoints))));
        }
    }
    else if (nX1>nX2 && nY1>nY2){
        dbAlpha=atan((nY2-nY1)/(nX2-nX1));
        dbCenterRotX=nX1+n_Radius*sin(-dbTheta/2+dbAlpha);
        dbCenterRotY=nY1-n_Radius*cos(-dbTheta/2+dbAlpha);
        
        dbGamma=PI/2+dbAlpha-dbTheta/2;
        
        for (float i=0; i<db_NumberWayPoints; ++i) {
            tArc.push_back(make_pair((dbCenterRotX+n_Radius*cos(dbGamma+dbTheta*i/db_NumberWayPoints)),(dbCenterRotY+n_Radius*sin(dbGamma+dbTheta*i/db_NumberWayPoints))));
        }
    }
    else if (nX1<nX2 && nY1>nY2){
        dbAlpha=-atan((nY2-nY1)/(nX2-nX1));
        dbCenterRotX=nX1+n_Radius*cos(PI/2-dbTheta/2-dbAlpha);
        dbCenterRotY=nY1+n_Radius*sin(PI/2-dbTheta/2-dbAlpha);
        
        dbGamma=3*PI/2-dbTheta/2-dbAlpha;
        
        for (float i=0; i<db_NumberWayPoints; ++i) {
            tArc.push_back(make_pair((dbCenterRotX+n_Radius*cos(dbGamma+dbTheta*i/db_NumberWayPoints)),(dbCenterRotY+n_Radius*sin(dbGamma+dbTheta*i/db_NumberWayPoints))));
        }
    }
    tArc.push_back(make_pair(nX2,nY2));
    return tArc;

}

/////////////////////////////////////////////////
/////////////////////////////////////////////////

void Planner::SetGrid()
{
    float nNumberStepsX=ceil(ArenaWidth/MaxSubdiv);
    m_fStep=ArenaWidth/nNumberStepsX;
    float nNumberStepsY=floor(ArenaDepth/MaxSubdiv);
    
    for (int i=0; i<nNumberStepsX; ++i) {
        for (int j=0; j<nNumberStepsY; ++j) {
            m_tGrid.push_back(make_pair(i*m_fStep,j*m_fStep));
        }
    }
}

/////////////////////////////////////////////////
/////////////////////////////////////////////////

TVecCoord Planner::MapObstacles(TVecCoord t_centerBoxes){
    
    TVecCoord tObstacles;
    float fCenterX;
    float fCenterY;
    float fXMax;
    float fXMin;
    float fYMax;
    float fYMin;
    Coordinate tMin;
    Coordinate tMax;
    
    float fRadius=sqrt(2)/2*BoxSize; //circumsquare of box
    
    for (TVecCoord::iterator it=t_centerBoxes.begin(); it!=t_centerBoxes.end(); ++it) {
        fCenterX=it->first;
        fCenterY=it->second;
        
        fXMax=fCenterX+fRadius;
        fXMin=fCenterX-fRadius;
        fYMax=fCenterY+fRadius;
        fYMin=fCenterY-fRadius;
        
        for (TVecCoord::iterator it1=m_tGrid.begin(); it1!=m_tGrid.end(); ++it1) {
            if ( (it1->first<fXMax) && (it1->second<fYMax) && (it1->first>fXMin) && (it1->second>fYMin) ) {
                tObstacles.push_back(*it1);
            }
        }
    }
    return tObstacles;
}

/////////////////////////////////////////////////
/////////////////////////////////////////////////


TVecCoord Planner::AStarSearch(Coordinate t_start, Coordinate t_goal, TVecCoord t_obstacles)
{

    TVecNode tOpen;
    TVecNode tNeighbors;
    TVecNode tClosed;
    TVecCoord tPath;

    float fXGoal=t_goal.first;
    float fYGoal=t_goal.second;
    bool bGoalReached=false;
    
    TVecCoord::iterator it5=min_element(m_tGrid.begin(),m_tGrid.end(),Distance(t_start));
    
    Node tStart;
    tStart.X=it5->first;
    tStart.Y=it5->second;
    
    tOpen.push_back(tStart);
    
    while (!tOpen.empty() && !bGoalReached) {
        
        tNeighbors.clear();
        TVecNode::iterator it=min_element(tOpen.begin(),tOpen.end(),LessThanByCost());
        Node tCurrent=*it;
        
        tOpen.erase(it);
        
        for (int i=-1; i<2; i++) {
            for (int j=-1; j<2; j++) {
                if (!(j==0 && i==0)) {
                    float fTempX=tCurrent.X+i*m_fStep;
                    float fTempY=tCurrent.Y+j*m_fStep;
                    if (fTempX>0 && fTempY>0 && fTempX<ArenaWidth && fTempY<ArenaDepth) {
                        TVecCoord::iterator it4=find_if(t_obstacles.begin(),t_obstacles.end(),CompareXandYCoord(fTempX,fTempY));
                        if (it4==t_obstacles.end()) {
                            Node tTemp;
                            tTemp.X=fTempX;
                            tTemp.Y=fTempY;
                            tTemp.g=tCurrent.g + pow(fTempX-tCurrent.X,2) + pow(fTempY-tCurrent.Y,2);
                            tTemp.h=pow(fXGoal-fTempX,2) + pow(fYGoal-fTempY,2);
                            tTemp.f=tTemp.g+tTemp.h;
                            tNeighbors.push_back(tTemp);
                        }
                    }
                }
            }
        }

        for (TVecNode::iterator it1=tNeighbors.begin(); it1!=tNeighbors.end(); ++it1) {
            float fNeighborX=it1->X;
            float fNeighborY=it1->Y;
            float fNeighborCost=it1->f;
            if ((abs(fNeighborX-fXGoal)<m_fStep/2) && (abs(fNeighborY-fYGoal)<m_fStep/2)) {
                bGoalReached=true;
                tClosed.push_back(tCurrent);
                tClosed.push_back(*it1);
                break;
            }
            else
            {
                bool bState=true;
                TVecNode::iterator it2=find_if(tOpen.begin(),tOpen.end(),CompareXandY(fNeighborX,fNeighborY));
                TVecNode::iterator it3=find_if(tClosed.begin(),tClosed.end(),CompareXandY(fNeighborX,fNeighborY));
                
                if ( (it2!=tOpen.end()) && ((it2->f)<fNeighborCost) ) {
                    bState=false;
                    
                }
                else if (it3!=tClosed.end() && ((it3->f)<fNeighborCost)) {
                    bState=false;
                }
                if (bState) {
                    tOpen.push_back(*it1);
                }
            }
        }
        if (!bGoalReached) {
            tClosed.push_back(tCurrent);
        }
    }
    
    for (TVecNode::iterator it=tClosed.begin(); it!=tClosed.end(); ++it) {
        tPath.push_back(make_pair(it->X,it->Y));
    }
    
    return tPath;
}

/////////////////////////////////////////////////
/////////////////////////////////////////////////

TVecCoord Planner::SamplePath(TVecCoord t_AStarPath){ // if delta x close to zero => NaN, change that !
    
    TVecCoord tPath;
    float fPrevSlope=0;
    float fCurrSlope;
    
    for (int i=1; i<t_AStarPath.size(); ++i) {
        fCurrSlope=(t_AStarPath[i-1].second-t_AStarPath[i].second)/(t_AStarPath[i-1].first-t_AStarPath[i].first);
        if (fCurrSlope!=fPrevSlope) {
            tPath.push_back(make_pair(t_AStarPath[i-1].first,t_AStarPath[i-1].second));
        }
        fPrevSlope=fCurrSlope;
    }
    tPath.push_back(make_pair(t_AStarPath.back().first,t_AStarPath.back().second));
    
    return tPath;
}

/////////////////////////////////////////////////
/////////////////////////////////////////////////

Path Planner::GetWaypointsAndHeadings(TVecCoord t_sampledPath, float f_initialOrientation, float f_finalOrientation){
    
    Path tLocation;
    Location temp;
    temp.Orientation=f_finalOrientation;
    
    for (int i=(t_sampledPath.size()-1); i>0; --i) {
        temp.X=t_sampledPath[i].first;
        temp.Y=t_sampledPath[i].second;
        tLocation.insert(tLocation.begin(),temp);
        if (abs(temp.X-t_sampledPath[i-1].first)<Epsilon) {
            if (temp.Y>t_sampledPath[i-1].first) {
                temp.Orientation=270;
            }
            else {
                temp.Orientation=90;
            }
        }
        else if(abs(temp.Y-t_sampledPath[i-1].second)<Epsilon){
            if (temp.X>t_sampledPath[i-1].first) {
                temp.Orientation=180;
            }
            else {
                temp.Orientation=0;
            }
        }
        else {
            temp.Orientation=atan((temp.Y-t_sampledPath[i-1].second)/(temp.X-t_sampledPath[i-1].first))/PI*180;
        }
    }
    temp.Orientation=f_initialOrientation;
    tLocation.insert(tLocation.begin(),temp);
    return tLocation;
}


/////////////////////////////////////////////////
/////////////////////////////////////////////////

Planner::Planner(){
    
}



