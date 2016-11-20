#include "bezier.hpp"
#include "planner.h"

using namespace std;

int main(int argc, char *argv[])
{
    Planner P;
    P.SetGrid();
//    TVecCoord G = P.GetGrid();
//    for (TVecCoord::iterator it=G.begin(); it!=G.end(); ++it) {
//        cout<<it->first<<'\t'<<it->second<<endl;
//    }
    
    Coordinate tStart=pair<float,float>(10,55); // A
    Coordinate tGoal=pair<float,float>(100,100);
    
    TVecCoord tCenterBoxes;
    tCenterBoxes.push_back(make_pair(50,50));
    TVecCoord tObstacles=P.MapObstacles(tCenterBoxes);
    TVecCoord tAStarPath=P.AStarSearch(tStart,tGoal,tObstacles);
    TVecCoord tPath=P.SamplePath(tAStarPath);
    
    Location tPushStart=Location(10,55,0);
    
    Path vecWaypointsHeadings=GetWaypointsAndHeadings(tPath,tPushStart.Orientation);
    vector<CubicBezier> tTrajectory= TrajectoryFromWaypointsAndHeadings(vecWaypointsHeadings);
    
    for (TVecCoord::iterator it=tObstacles.begin(); it!=tObstacles.end(); ++it) {
        cout<<it->first<<'\t'<<it->second<<endl;
    }
    
    cout<<endl;
    
    for (TVecCoord::iterator it=tAStarPath.begin(); it!=tAStarPath.end(); ++it) {
        cout<<it->first<<'\t'<<it->second<<endl;
    }
    
    cout<<endl;
    
    for (TVecCoord::iterator it=tPath.begin(); it!=tPath.end(); ++it) {
        cout<<it->first<<'\t'<<it->second<<endl;
    }
    
}
