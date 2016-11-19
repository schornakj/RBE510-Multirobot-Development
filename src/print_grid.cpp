
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
    
    Coordinate tStart=pair<float,float>(10,10);
    Coordinate tGoal=pair<float,float>(100,100);
    
    TVecCoord tCenterBoxes;
    tCenterBoxes.push_back(make_pair(50,50));
    TVecCoord tObstacles=P.MapObstacles(tCenterBoxes);
    TVecCoord tAStarPath=P.AStarSearch(tStart,tGoal,tObstacles);
    TVecCoord tPath=P.SamplePath(tAStarPath);
    
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
