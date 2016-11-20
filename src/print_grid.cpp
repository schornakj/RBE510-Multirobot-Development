#include "bezier.hpp"
#include "planner.h"

#define BOX_OFFSET 10

using namespace std;

vector<CubicBezier> TrajectoryFromWaypointsAndHeadings(vector<Location> input) {
    // first in pair is Point containing metric coordinates
    // second in pair is heading at waypoint in degrees
    // Returns series of cubic bezier curves that provide a continuous path intersecting all waypoints at specified headings
    
    vector<CubicBezier> output;
    
    for (int i = 0; i < input.size() - 1; i++) {
        output.push_back(CubicBezier(input[i].X, input[i].Y, input[i].Orientation, input[i+1].X, input[i+1].Y, input[i+1].Orientation, 0.5));
    }
    return output;
}


int main(int argc, char *argv[])
{
    Planner P;
    P.SetGrid();
//    TVecCoord G = P.GetGrid();
//    for (TVecCoord::iterator it=G.begin(); it!=G.end(); ++it) {
//        cout<<it->first<<'\t'<<it->second<<endl;
//    }
    
    Coordinate tStart=pair<float,float>(10,55); // A
    Coordinate tGoal=pair<float,float>(130-BOX_OFFSET,40); // B
    
    TVecCoord tCenterBoxes;
    tCenterBoxes.push_back(make_pair(40,40));
    tCenterBoxes.push_back(make_pair(90,80));
    tCenterBoxes.push_back(make_pair(140,40));
    tCenterBoxes.push_back(make_pair(200,80));
    
    TVecCoord tObstacles=P.MapObstacles(tCenterBoxes);
    TVecCoord tAStarPath=P.AStarSearch(tStart,tGoal,tObstacles);
    TVecCoord tPath=P.SamplePath(tAStarPath);
    
    Location tPushStart=Location(10,55,0);
    
    Path vecWaypointsHeadings=P.GetWaypointsAndHeadings(tPath,0,tPushStart.Orientation);
    vector<CubicBezier> tTrajectory= TrajectoryFromWaypointsAndHeadings(vecWaypointsHeadings);
    
    cout<<"Obstacles:"<<endl;
    
    for (TVecCoord::iterator it=tObstacles.begin(); it!=tObstacles.end(); ++it) {
        cout<<it->first<<'\t'<<it->second<<endl;
    }
    
    cout<<endl<<"AStar:"<<endl;
    
    for (TVecCoord::iterator it=tAStarPath.begin(); it!=tAStarPath.end(); ++it) {
        cout<<it->first<<'\t'<<it->second<<endl;
    }
    
    cout<<endl<<"Sampled Path:"<<endl;
    
    for (TVecCoord::iterator it=tPath.begin(); it!=tPath.end(); ++it) {
        cout<<it->first<<'\t'<<it->second<<endl;
    }
    
    cout<<endl<<"Bezier:"<<endl;
    
    for (int i=0; i < tTrajectory.size(); i++) {
        //cout << "Current segment: " << i << endl;
        //cout << "Making a curve starting at [" << current.controlPoints[0].x << "," << current.controlPoints[0].y << "]cm and ending at [" << current.controlPoints[3].x << "," << current.controlPoints[3].y << "]" << endl;
        
        CubicBezier current = tTrajectory[i];
        float stepSize = 0.05;
        for (float j = 0.2; j < 1 + stepSize; j += stepSize){
            cout<<current.GetPoint(j).x<<'\t'<<current.GetPoint(j).y<<endl;
            //cout << "Current Target: #" << j << " " << coordinate << "px " << '\t' << current.GetPoint(j).x << "cmX " << current.GetPoint(j).y << "cmY " << endl;
        }
        cout<<endl;
    }
    
}
