#ifndef PLANNER_H
#define PLANNER_H

#include <map>
#include <vector>
//#include <queue>
#include <iostream>
#include <string>
#include <cmath>

using namespace std;

const float PI = 3.1415927;
const int ArenaWidth=1000; //in X direction
const int ArenaLength=1000; //in Y direction
const int MinSubdiv=100;

typedef std::map< int, int > Grid;
typedef pair< double ,double> Location;
typedef vector<Location > Path;

class Planner {
    
public:
    
    Planner();
    Path CircleArc(Location t_start, Location t_goal, double n_Radius, double db_NumberWayPoints);//case x1=x2 or y1=y2
    
    void SetGrid(Location t_start, Location t_goal);
    int Subdivide(int n_x1, int n_x2,int n_MaxSubdiv);
    
    inline Grid GetGrid(){
        return m_tGrid;
    }
    
    private:
    
    Grid m_tGrid;
    
};

#endif
