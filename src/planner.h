#ifndef PLANNER_H
#define PLANNER_H

#include <vector>
#include <iostream>
#include <string>
#include <cmath>
#include <algorithm>

using namespace std;

const float PI = 3.1415927;
const float ArenaWidth=231.14; //in X direction
const float ArenaDepth=109.86; //in Y direction
const float BoxSize=20;
const float MaxSubdiv=7.63;
const float Epsilon=0.001;

struct Location {
    float X,Y,Orientation;
    Location(float X,float Y, float Orientation): X(X), Y(Y), Orientation(Orientation) {}
};

struct Node {
    
    float X;
    float Y;
    float f,g,h;
    Node(): X(0),Y(0),f(0),g(0),h(0) {}
};

typedef pair<float, float> Coordinate;
typedef vector<Coordinate> TVecCoord;
typedef vector<Node> TVecNode;
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

struct Distance {
    pair<float,float> temp;
    Distance(Coordinate point): temp(point) {}
    bool operator()(const Coordinate& obj1,const Coordinate& obj2){return abs(obj1.first-temp.first)< abs(obj2.first-temp.first) && abs(obj1.second-temp.second)< abs(obj2.second-temp.second);}
};

struct CompareXandYCoord {
    float x,y;
    CompareXandYCoord(float x,float y): x(x), y(y){}
    bool operator()(const Coordinate& obj){return abs(obj.first-x)<Epsilon && abs(obj.second-y)<Epsilon;}
};


class Planner {

public:
    
    Planner();
    TVecCoord CircleArc(Coordinate t_start, Coordinate t_goal, double n_Radius, double db_NumberWayPoints);
    
    void SetGrid();
    
    TVecCoord MapObstacles(TVecCoord t_centerBoxes);
    
    TVecCoord AStarSearch(Coordinate t_start, Coordinate t_goal ,TVecCoord t_obstacle);
    
    TVecCoord SamplePath(TVecCoord t_AStarPath);
    
    inline TVecCoord GetGrid(){
        return m_tGrid;
    }
    
    private:
    
    TVecCoord m_tGrid;
    float m_fStep;
    
};

#endif
