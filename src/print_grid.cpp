
#include "planner.h"

using namespace std;

const double Radius=200; // Radius from center of rotation to center of the box (maybe compute based on criteria)
const double nBoxLength=100; //length of box in cm
const double WayDistance=10; // Way point every 10 cm

int main(int argc, char *argv[])
{
    Planner P;
    P.SetGrid();
    Grid G = P.GetGrid();
//    for (Grid::iterator it=G.begin(); it!=G.end(); ++it) {
//        cout<<it->X<<'\t'<<it->Y<<endl;
//    }
    Coordinate tStart=pair<float,float>(10,10);
    Coordinate tGoal=pair<float,float>(110,110);
    PathXY tPath=P.AStarSearch(tStart,tGoal,P.m_fStepX,P.m_fStepY);
    for (PathXY::iterator it=tPath.begin(); it!=tPath.end(); ++it) {
        cout<<it->first<<'\t'<<it->second<<endl;
    }
}
