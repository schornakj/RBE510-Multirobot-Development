
#include "planner.h"

using namespace std;

const double Radius=200; // Radius from center of rotation to center of the box (maybe compute based on criteria)
const double nBoxLength=100; //length of box in cm
const double WayDistance=10; // Way point every 10 cm

int main(int argc, char *argv[])
{
    Planner P;
    P.SetGrid();
//    TVecCoord G = P.GetGrid();
//    for (TVecCoord::iterator it=G.begin(); it!=G.end(); ++it) {
//        cout<<it->first<<'\t'<<it->second<<endl;
//    }
    
    Coordinate tStart=pair<float,float>(10,10);
    Coordinate tGoal=pair<float,float>(110,110);
    
    TVecCoord tCenterBoxes;
    tCenterBoxes.push_back(make_pair(50,50));
    TVecCoord tObstacles=P.MapObstacles(tCenterBoxes);
    
    for (TVecCoord::iterator it=tObstacles.begin(); it!=tObstacles.end(); ++it) {
        cout<<it->first<<'\t'<<it->second<<endl;
    }
    
    cout<<endl;
    
    TVecCoord tPath=P.AStarSearch(tStart,tGoal,tObstacles);
    for (TVecCoord::iterator it=tPath.begin(); it!=tPath.end(); ++it) {
        cout<<it->first<<'\t'<<it->second<<endl;
    }
    
}
