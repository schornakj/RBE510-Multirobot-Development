#ifndef PLANNER_H
#define PLANNER_H

#include <map>
#include <vector>
//#include <list>
#include <iostream>
#include <string>
#include <cmath>
#include <algorithm>

using namespace std;

const float PI = 3.1415927;
const float ArenaWidth=231.14; //in X direction
const float ArenaDepth=109.86; //in Y direction
const float MaxSubdiv=7.63;
const float Epsilon=0.001;

struct Location {
    float X,Y,Orientation;
};

struct Node {
    
    float X;
    float Y;
    float f,g,h;
    Node(): X(0),Y(0),f(0),g(0),h(0) {}
};

typedef pair<float, float> Coordinate;
typedef vector<Node> Grid;
typedef vector<Coordinate> PathXY; // to be removed, kept to be able to use old functions
typedef vector<Location > Path;

struct LessThanByCost {
    LessThanByCost(){}
    bool operator()(Node const & n1,Node const & n2){return n1.f<n2.f;};
};

struct CompareXandY {
    float x,y;
    CompareXandY(float x,float y): x(x), y(y){}
    bool operator()(const Node& obj){return abs(obj.X-x)<Epsilon && abs(obj.Y-y)<Epsilon;}
};



class Planner {

public:
    
    Planner();
    PathXY CircleArc(Coordinate t_start, Coordinate t_goal, double n_Radius, double db_NumberWayPoints);
    
    void SetGrid();

    PathXY AStarSearch(Coordinate t_start, Coordinate t_goal ,float f_stepX ,float f_stepY); //, Grid t_obstacle);
    //Path PathConstraints(Path t_UnconstrainedPath);

    void SetGrid(Location t_start, Location t_goal);
    int Subdivide(int n_x1, int n_x2,int n_MaxSubdiv);
    
    inline Grid GetGrid(){
        return m_tGrid;
    }
    
    int m_nNumberStepsX;
    int m_nNumberStepsY;
    float m_fStepX;
    float m_fStepY;
    
    private:
    
    Grid m_tGrid;
    
};

#endif
