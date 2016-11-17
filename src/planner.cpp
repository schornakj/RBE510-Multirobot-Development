
#include "planner.h"

PathXY Planner::CircleArc(Coordinate t_start, Coordinate t_goal, double n_Radius, double db_NumberWayPoints)
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
    PathXY tArc;
    
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
    m_nNumberStepsX=ceil(ArenaWidth/MaxSubdiv);
    m_fStepX=ArenaWidth/m_nNumberStepsX;
    m_nNumberStepsY=ceil(ArenaDepth/MaxSubdiv);
    m_fStepY=ArenaDepth/m_nNumberStepsY;
    for (int i=0; i<m_nNumberStepsX; ++i) {
        for (int j=0; j<m_nNumberStepsY; ++j) {
            Node temp;
            temp.X=i*m_fStepX;
            temp.Y=j*m_fStepY;
            m_tGrid.push_back(temp);
        }
    }

}

/////////////////////////////////////////////////
/////////////////////////////////////////////////


PathXY Planner::AStarSearch(Coordinate t_start, Coordinate t_goal ,float f_stepX ,float f_stepY)
{

    Grid tOpen;
    Grid tSuccessors;
    Grid tClosed;
    PathXY tPath;
    

    float fXGoal=t_goal.first;
    float fYGoal=t_goal.second;
    bool bGoalReached=false;
    
    Node tStart;
    tStart.X=t_start.first;
    tStart.Y=t_start.second;
    tOpen.push_back(tStart);
    
    while (!tOpen.empty() && !bGoalReached) {
        
        tSuccessors.clear();
        Grid::iterator it=min_element(tOpen.begin(),tOpen.end(),LessThanByCost());
        Node q=*it;
        
        tOpen.erase(it);
        
        for (int i=-1; i<2; ++i) {
            for (int j=-1; j<2; ++j) {
                if (j!=0) {
                    float tempX=q.X+i*f_stepX;
                    float tempY=q.Y+j*f_stepY;
                    if (tempX>0 && tempY>0) {
                        Node temp;
                        temp.X=tempX;
                        temp.Y=tempY;
                        temp.g=q.g + pow(tempX-q.X,2) + pow(tempY-q.Y,2);
                        temp.h=pow(fXGoal-tempX,2) + pow(fYGoal-tempY,2);
                        temp.f=temp.g+temp.h;
                        tSuccessors.push_back(temp);
                    }
                }
            }
        }

        for (Grid::iterator it1=tSuccessors.begin(); it1!=tSuccessors.end(); ++it1) {
            float SuccX=it1->X;
            float SuccY=it1->Y;
            float Succf=it1->f;
            if ((abs(SuccX-fXGoal)<f_stepX/2) && (abs(SuccY-fYGoal)<f_stepY/2)) { //break in previous loop?
                bGoalReached=true;
                //tClosed.push_back(*it1);
                break;
            }
            else
            {
                bool bState=true;
                Grid::iterator it2=find_if(tOpen.begin(),tOpen.end(),CompareXandY(SuccX,SuccY));
                Grid::iterator it3=find_if(tClosed.begin(),tClosed.end(),CompareXandY(SuccX,SuccY));
                
                if ( (it2!=tOpen.end()) && ((it2->f)<Succf) ) {
                    bState=false;
                    
                }
                else if (it3!=tClosed.end() && ((it3->f)<Succf)) {
                    bState=false;
                }
                if (bState) {
                    tOpen.push_back(*it1);
                }
            }
        }
        tClosed.push_back(q);
    }
    
    for (Grid::iterator it=tClosed.begin(); it!=tClosed.end(); ++it) {
        tPath.push_back(make_pair(it->X,it->Y));
    }
    
    return tPath;
}

/////////////////////////////////////////////////
/////////////////////////////////////////////////

//bool Planner::Comp(const float& f1,const float& f2){
//    return f1
//}


Planner::Planner(){
    
}


//void planner::a_star_search
//(const Graph& graph,
// typename Graph::Location start,
// typename Graph::Location goal,
// unordered_map<typename Graph::Location, typename Graph::Location>& came_from,
// unordered_map<typename Graph::Location, double>& cost_so_far)
//{
//    PriorityQueue<Location, double> frontier;
//    frontier.put(start, 0);
//
//    came_from[start] = start;
//    cost_so_far[start] = 0;
//
//    while (!frontier.empty()) {
//        auto current = frontier.get();
//
//        if (current == goal) {
//            break;
//        }
//
//        for (auto next : graph.neighbors(current)) {
//            double new_cost = cost_so_far[current] + graph.cost(current, next);
//            if (!cost_so_far.count(next) || new_cost < cost_so_far[next]) {
//                cost_so_far[next] = new_cost;
//                double priority = new_cost + heuristic(next, goal);
//                frontier.put(next, priority);
//                came_from[next] = current;
//            }
//        }
//    }
//}


//int nLength=abs(n_x1-n_x2);
//if(nLength%2!=0) nLength-=1;
//int nRemainder=0;
//int nStep=0;
//
//for (int i=n_MaxSubdiv; i>0; --i) {
//    nRemainder=nLength%i;
//    nStep=nLength/i; // Euclidian division
//    if(nStep>=100){
//        if(nRemainder==0) break;
//        else {
//            nStep+=nRemainder;
//            break;
//        }
//    }
//}
//
//int nNumberXStepsOrig=min(nX1,nX2)/nXStep; // Euclidian division
//int nNumberYStepsOrig=min(nY1,nY2)/nYStep; // Euclidian division
//
//int nFirstX=min(nX1,nX2)-nNumberYStepsOrig*nXStep;
//int nFirstY=min(nY1,nY2)-nNumberYStepsOrig*nYStep;
//
//int nNumberXSteps=(ArenaWidth-nNumberXStepsOrig)/nXStep;
//int nNumberYSteps=(ArenaWidth-nNumberYStepsOrig)/nYStep;
//
//
//float fStep=fLength/fSubdiv;
//
//for (int i=0; i<nNumberXSteps; ++i) {
//    for (int j=0; j<nNumberYSteps; ++j) {
//        m_tGrid.insert(make_pair(nFirstX+i*nXStep,nFirstY+j*nYStep));
//    }
//}
//

