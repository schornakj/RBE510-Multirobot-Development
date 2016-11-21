#include <iostream>
#include <sstream>
#include <string>
#include <stdio.h>
#include <stdlib.h>
#include <thread>
#include <cmath>

#include "planner.h"
#include "rbe510.hpp"
#include <opencv2/core.hpp>
#include "rectification.hpp"
#include "bezier.hpp"
#include "Eigen/Eigen"

using namespace std;
using namespace cv;

#define ALLOWED_ANGLE_ERROR 1

#define ALLOWED_DISTANCE 10 // Allowed distance from target is in pixels.

#define BOX_OFFSET 10 // Distance in cm that the robot should stop driving from the box when preparing to push it

#define RED_ID 5
#define GREEN_ID 0
#define BLUE_ID 4

//////////////// REMOVE AS ALREADY DEFINED IN PLANNER.H /////////////////

// Width and height of rectangle marked by corner codes in cm
double fieldWidth = 231.14;
double fieldHeight = 109.86;

//  Dimension of box/entity
float boxdim = 7.63;

const float t_BR2RR=68;
const float t_BL2RL=65;
const float t_RR2RRE=63;
const float t_RL2RLE=20;
const float t_P2BR=140;
const float t_P2BL=107;
const float t_BRy=26;
const float t_BLy=74;
const float t_BR2BRE=80;
const float t_BL2BLE=37;
/*
RED_RIGHT=(138,27)
RED_LEFT=(167,70)
BLUE_RIGHT=(70,26)
BLUE_LEFT=(102,74)
*/
//////////////////////////////////////////////////////////////////////////


enum State {WRONG_SIDE, CORRECT_SIDE, PARKING, DONE};
// East = X-positive
// North = Y-positive
// West = X-negative
// South = Y-negative (shouldn't ever have to push south)
enum Direction {EAST, NORTH, WEST, SOUTH};
enum Color {RED, BLUE, GREEN};
enum Position {RED_RIGHT, RED_RIGHT_END, RED_LEFT, RED_LEFT_END, BLUE_RIGHT, BLUE_RIGHT_END, BLUE_LEFT, BLUE_LEFT_END};

/* */
class Bot{
private :
    PerspectiveCorrection correction;
public:
    Entity m_tBot;
    Color m_tBotColor;
    float m_fXCm;
    float m_fYCm;

    
    Bot(Entity t_Bot, Color t_Color, PerspectiveCorrection t_Correction){
        m_tBot = t_Bot;
        m_tBotColor = t_Color;
        correction = t_Correction;
        
        // Perform perspective correction to get metric position
        Matrix<double, 1, 3> position= t_Correction.correctPerspectiveMetric(t_Bot.x(),t_Bot.y());
        
        m_fXCm = position(0,0);
        m_fYCm = position(0,1);
    }

};

/* Discretizing boxes*/
class Box {
    
private:
    /* Variable to reference the perspective correction to get the position of the box in cm */
    PerspectiveCorrection correction;
public:
    
    /* Box variables */
    Entity box;
    Color boxColor;
    State currentStatus; // box in right/wrong color zone or parking space or in end zone (four choices)
    Position currentPosition; // box in the left/right and red/blue zone or left/right and red/blue end zone (eight choices)
    bool isHighPriority; //could be removed since our code is not general enough to switch
    float xCm;
    float yCm;
    
    /* Constructor */
    
    Box(Entity inputBox, Color inputColor, PerspectiveCorrection inputCorrection){
        this->box = inputBox;
        this->boxColor = inputColor;
        this->correction = inputCorrection;
        
        // Perform perspecitve correction to get metric position
        Matrix<double, 1, 3> position= correction.correctPerspectiveMetric(box.x(),box.y());
        
        this->xCm = position(0,0);
        this->yCm = position(0,1);
        
        // Update box position based on field zone rules
        updateBoxStatus();
        if (boxColor == RED) {
            isHighPriority = true;
        } else {
            isHighPriority = false;
        }
    }
    
    /* Update box state based on its position on the field.
     Red boxes in the red zone (west of centerline) and blue boxes in the blue zone (east of centerline) are on the correct side.
     Boxes in the opposite color's zone are on the wrong side.
     Boxes in the parking zone east of the red zone are parking (only low-priority (i.e. blue) boxes should end up here)
     Boxes in the appropriate color zone and close to the north edge of the field are done being moved*/
    
    void updateBoxStatus() {
        State output;
        if (boxColor == RED) {
            if (xCm > fieldWidth/2) {
                output = CORRECT_SIDE;
                if (yCm > fieldHeight*0.75) {
                    output = DONE;
                }
            } else {
                output = WRONG_SIDE;
            }
        } else if (boxColor == BLUE) {
            if (xCm <= fieldWidth/2) {
                output = CORRECT_SIDE;
                if (yCm > fieldHeight*0.75) {
                    output = DONE;
                }
            } else if (xCm >= fieldWidth*0.8) {
                output = PARKING;
            } else {
                output = WRONG_SIDE;
            }
        }
        currentStatus=output;
    }
    
    /* Get robot position and orientation needed to start pushing the box in the requested direction */
    Location getPushStartPosition(Direction pushDirection) {
        Location output;
        if (pushDirection == EAST) {
            output.Orientation = 0;
            output.Y = yCm;
            output.X = xCm - BOX_OFFSET;
        } else if (pushDirection == NORTH) {
            output.Orientation = 90;
            output.X = xCm;
            output.Y = yCm - BOX_OFFSET;
        } else if (pushDirection == WEST) {
            output.Orientation = 180;
            output.Y = yCm;
            output.X = xCm + BOX_OFFSET;
        } else if (pushDirection == SOUTH) {
            output.Orientation = 270;
            output.Y = xCm;
            output.X = yCm + BOX_OFFSET;
        }
        return output;
    }
    
    
void getDiscretePosition(){
        /* Checks if a box is red/blue. If red or blue, this will check if the discrete position is the right,
           left or in the left/right end positions. Additionally, if the entities are not in the end position, it will
           display the available discrete positoin for each entity. 
            RED_RIGHT(138,27)
            RED_LEFT(167,70)
            BLUE_RIGHT(70,26)
            BLUE_LEFT(102,74)
        */
        Position p_output;
        if (boxColor == RED) {
            if (yCm<fieldHeight*0.25) {
                p_output=RED_RIGHT;
            }
            else if (yCm > fieldHeight*0.25 && yCm<fieldHeight*0.75){
                p_output=RED_LEFT;
            }
            else if (yCm > fieldHeight*0.75){
                if (xCm < ((0.5*fieldWidth)+boxdim)) {
                    p_output=RED_RIGHT_END;
                }
                else{
                    p_output=RED_LEFT_END;
                }
            }
        }
        else if (boxColor ==BLUE){
            if (xCm < fieldWidth/2){
                if (yCm < fieldHeight*0.25){
                    p_output=BLUE_RIGHT;
                }
                else if (yCm > fieldHeight*0.25 && yCm < fieldHeight*0.75){
                    p_output=BLUE_LEFT;
                }
                else if (yCm > fieldHeight*0.75){
                    if (xCm >= ((0.5*fieldWidth)-boxdim)){
                        p_output=BLUE_LEFT_END;
                    }
                    else{
                        p_output=BLUE_RIGHT_END;
                    }
                }
            }
        }
        currentPosition=p_output;
    }
};



class PID {
public:
    // Integrated Error is updated each time class gets an update and error entered is
    // is added to Integrated Error.
    float I_error;
    float error;
    float KP;
    float KI;
    float KD;
    float delta_pos;

    // Initializes the desired value and K values from the arguements.
    PID(float KP, float KI, float KD){
        this->KP = KP;
        this->KI = KI;
        this->KD = KD;
        I_error = 0;
    }

    // Updates the PID with a new error and calculates the pid output
    float update(float error){
        float delta_error = this->error - error;
        this->error = error;
        I_error += error;
        delta_pos = (KP * error) + (KI * I_error) + (KD * delta_error);
        if(delta_pos < 0) return -delta_pos;
        return delta_pos;
    }

    // Sets the I_error to 0
    void reset(){
        I_error = 0;
    }
};

Robot getRobot(int id, FieldComputer fc){
    // Request update from the field computer.
    FieldData data = fc.getFieldData();
    Robot robot(id);
    bool found = false;
    for(unsigned i = 0; i < data.robots.size(); i++){
        if(data.robots[i].id() == robot.id()) {
            robot = data.robots[i];
            return robot;
        }
    }
    return Robot(-1);
}

double angleError(double diff){
    double difference = diff;
    while (difference < -180) difference += 360;
    while (difference > 180) difference -= 360;
    return difference;
}


void SendToPoint(int id, FieldComputer fc, int targetx, int targety) {
    /*
    target_reached is set to false to indicate
    that the robot has not reached the target coordinates.
    */
    bool target_reached = false;

    while(!target_reached){
        Robot robot = getRobot(id, fc);
        if(robot.id() == id){
            PID pid(0.01, 0.005, 0.00005);
            while(true){
                robot = getRobot(id, fc);
                if(robot.id() == id){
                    float target_angle = 180.0 - atan2(robot.y() - targety, robot.x() - targetx) * 180.0 / M_PI;
                    float angle_difference = target_angle - robot.theta();
                    float angle_error = angleError(angle_difference);
                    if(angle_error < ALLOWED_ANGLE_ERROR && angle_error > -ALLOWED_ANGLE_ERROR) break;
                    if(angle_error < 0) fc.arcadeDrive(id, pid.update(angle_error), 1.0);
                    else fc.arcadeDrive(id, pid.update(angle_error), -1.0);
                    usleep(50000);
                }
            }
            fc.arcadeDrive(id, 1.0, 0.0);
            if(sqrt(pow(robot.x() - targetx,2.0) + pow(robot.y() - targety,2.0)) < ALLOWED_DISTANCE) target_reached = true;
        }
        usleep(50000);
    }
    fc.arcadeDrive(id, 0.0, 0.0);
    //return 0;
}

void FollowTrajectory(int id, FieldComputer fc, PerspectiveCorrection correction, vector<CubicBezier> trajectory) {
	for (int i=0; i < trajectory.size(); i++) {
		//cout << "Current segment: " << i << endl;
        //cout << "Making a curve starting at [" << current.controlPoints[0].x << "," << current.controlPoints[0].y << "]cm and ending at [" << current.controlPoints[3].x << "," << current.controlPoints[3].y << "]" << endl;

		CubicBezier current = trajectory[i];
        float stepSize = 0.2;//0.05;
		for (float j = 0.2; j < 1 + stepSize; j += stepSize){
			Matrix<double,1,3> coordinate = correction.correctPerspectivePixels(current.GetPoint(j).x,current.GetPoint(j).y);
			//cout << "Current Target: #" << j << " " << coordinate << "px " << '\t' << current.GetPoint(j).x << "cmX " << current.GetPoint(j).y << "cmY " << endl;
			cout<<current.GetPoint(j).x<<'\t' << current.GetPoint(j).y<<endl;
            SendToPoint(id, fc, (int)coordinate(0,0),(int)coordinate(0,1));
		}
	}
}

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

/*
void TrackRobot(int inputId, PerspectiveCorrection correction, FieldComputer fc) {
    data = fc.getFieldData();
    while(true){
        for (unsigned i = 0; i < data.robots.size(); i++){
            if (data.robots[i].id() == inputId){
                Matrix<double, 1, 3> robotPos= correction.correctPerspectiveMetric((double)data.robots[i].x(),(double)data.robots[i].y());

                cout << "Actual robot position: " << robotPos(0,0) << " " << robotPos(0,1) << endl;
                //botRed = data.robots[i];
                //cout << "Found Red" << endl;
            }

        }
        usleep(1000000);
    }   
}
*/

int main(int argc, char *argv[])
{
    
    ////////////////////////// INITIALIZATION ///////////////////////////
    
    string ip = "127.0.0.1";
    FieldComputer fc(ip);
    fc = FieldComputer(ip);
    fc.disableVerbose();
    FieldData data = fc.getFieldData();

    double fieldWidth = 231.14;
	double fieldHeight = 109.86;

	Eigen::Matrix<double, 4, 2> p1;
	Eigen::Matrix<double, 4, 2> p2;
	p2 << 0, 0, 0, fieldHeight, fieldWidth, fieldHeight, fieldWidth, 0;

	vector<Entity> corners;
	vector<Entity> boxes;

    vector<Box> vecBoxes;
    vector<Bot> vecBots;
    

	while(corners.size() < 4 || boxes.size() < 4){
        data = fc.getFieldData();
		for(unsigned i = 0; i < data.entities.size(); i++){
			if (data.entities[i].id() >= 200 && data.entities[i].id() <= 203) {
				corners.push_back(data.entities[i]);
			}
			if (data.entities[i].id() >= 101 && data.entities[i].id() <= 112) {
				boxes.push_back(data.entities[i]);
			}
            
		}
		if (corners.size() < 4 || boxes.size() < 4){
			corners.clear();
			boxes.clear();
		}
	}
	cout << "Found all corners and boxes" << endl;

	// Get the coordinates of each of the corners
	for(unsigned i = 0; i < corners.size(); i++) {
		cout << "Corner fount at ID: " << corners[i].id() << endl;

		if (corners[i].id() == 200) {
			// upper left
			p1(1,0) = corners[i].x();
			p1(1,1) = corners[i].y();
		}
		else if (corners[i].id() == 201) {
			// lower left
			p1(0,0) = corners[i].x();
			p1(0,1) = corners[i].y();
		}
		else if (corners[i].id() == 202) {
			// upper right
			p1(2,0) = corners[i].x();
			p1(2,1) = corners[i].y();
		}
		else if (corners[i].id() == 203) {
			// lower right
			p1(3,0) = corners[i].x();
			p1(3,1) = corners[i].y();
		}
	}
    
    PerspectiveCorrection correction(p1, p2);
    
    Matrix<double, 1, 3> origin= correction.correctPerspectiveMetric((double)p1(0,0),(double)p1(0,1));
    Matrix<double, 1, 3> upperRight= correction.correctPerspectiveMetric((double)p1(2,0),(double)p1(2,1));
    cout << "CORRECTION SANITY CHECK:" << endl << "Origin: [" << origin(0,0) << "," << origin(0,1) << "]" << endl << "URC: [" << upperRight(0,0) << "," << upperRight(0,1) << "]" << endl;
    
    for (unsigned i = 0; i < boxes.size(); i++) {
        if (boxes[i].id() == 102) {
            // Red 1
            vecBoxes.push_back(Box(boxes[i],RED,correction));
        } else if (boxes[i].id() == 112) {
            // Red 2
            vecBoxes.push_back(Box(boxes[i],RED,correction));
        } else if (boxes[i].id() == 101) {
            // Blue 1
            vecBoxes.push_back(Box(boxes[i],BLUE,correction));
        } else if (boxes[i].id() == 103) {
            // Blue 2
            vecBoxes.push_back(Box(boxes[i],BLUE,correction));
        }
    }
    
    for (unsigned i = 0; i < data.robots.size(); i++){
        if (data.robots[i].id() == RED_ID){
            vecBots.push_back(Bot(data.robots[i],RED,correction));
            cout << "Found Red" << endl;
        }
        else if (data.robots[i].id() == GREEN_ID){
            vecBots.push_back(Bot(data.robots[i],GREEN,correction));
            cout << "Found Green" << endl;
        }
        else if (data.robots[i].id() == BLUE_ID){
            vecBots.push_back(Bot(data.robots[i],BLUE,correction));
            cout << "Found Blue" << endl;
        }
    }
    
    Planner P;
    ////////////////////////////////////////////////////
    
    P.SetGrid();
    
    /* Main routine */
    
    //NOTE : OUR CODE WILL NOT WORK IF BOTH BLUE BOXES (BOTH RED BOXES) START AT THE SAME Y VALUE !
    
    /* STEP 1 : Move blue boxes in Red Zone to the Parking Space */
    
    /* Get centers of the boxes in cm and put them in a TVecCoord */
    TVecCoord tCenterBoxes1;
    for (unsigned i = 0; i < vecBoxes.size(); i++) {
        tCenterBoxes1.push_back(make_pair(vecBoxes[i].xCm,vecBoxes[i].yCm));
    }
    /* Map obstacles */
    TVecCoord tObstacles=P.MapObstacles(tCenterBoxes1);
    
    /* Push blue (least priority) boxes which are in the red zone into the parking space*/
    for (unsigned i = 0; i < vecBoxes.size(); i++) {
        
        if (!vecBoxes[i].isHighPriority && vecBoxes[i].currentStatus==WRONG_SIDE) {
            /* Move blue robot from its initial position to blue box */
            Location tPushStart=vecBoxes[i].getPushStartPosition(EAST);
            Location tRobotStart;
            Coordinate tGoal=pair<float,float>(tPushStart.X,tPushStart.Y); //
            int nPusherId;
            
            for (vector<Bot>::iterator it=vecBots.begin(); it!=vecBots.end(); ++it) {
                if (it->m_tBotColor==BLUE) {
                    tRobotStart=Location(it->m_fXCm,it->m_fYCm,it->m_tBot.theta());
                    nPusherId=it->m_tBot.id();
                    break;
                }
            }
            
            Coordinate tStart=pair<float,float>(tRobotStart.X,tRobotStart.Y);
            
            cout<<"Center of Boxes:"<<endl;
           
            for (TVecCoord::iterator it=tCenterBoxes1.begin(); it!=tCenterBoxes1.end(); ++it) {
               cout<<it->first<<'\t'<<it->second<<endl;
            }
           
            cout<<endl<<"Map of obstacles:"<<endl;
           
           
            for (TVecCoord::iterator it=tObstacles.begin(); it!=tObstacles.end(); ++it) {
               cout<<it->first<<'\t'<<it->second<<endl;
            }


            TVecCoord tAStarPath=P.AStarSearch(tStart,tGoal,tObstacles);
            TVecCoord tPath=P.SamplePath(tAStarPath);
            
            Path vecWaypointsHeadings=P.GetWaypointsAndHeadings(tPath,tRobotStart.Orientation,tPushStart.Orientation);
            
            vector<CubicBezier> tTrajectory= TrajectoryFromWaypointsAndHeadings(vecWaypointsHeadings);
            
            cout<<endl<<"Waypoints:"<<endl;
           
           
            for (Path::iterator it=vecWaypointsHeadings.begin(); it!=vecWaypointsHeadings.end(); ++it) {
               cout<<it->X<<'\t'<<it->Y<<it->Orientation<<endl;
            }
            
            cout<<endl<<"Astar path:"<<endl;
           
            for (TVecCoord::iterator it=tAStarPath.begin(); it!=tAStarPath.end(); ++it) {
               cout<<it->first<<'\t'<<it->second<<endl;
            }

            cout<<endl<<"Bezier:"<<endl;
            FollowTrajectory(nPusherId,fc,correction,tTrajectory);
            
            /* Push box to Parking Space */
            vecBoxes[i].getDiscretePosition();
            
            float fDistanceToParking;
            if (vecBoxes[i].currentPosition==RED_RIGHT) {
                fDistanceToParking=31; // update with real value and add as global const float
            }
            else if(vecBoxes[i].currentPosition==RED_LEFT){ // else would be enough
                fDistanceToParking=19; // update with real value and add as global const float 19,31,45,60
            }
            
            Location tPushGoal=Location(tPushStart.X+fDistanceToParking,tPushStart.Y,tPushStart.Orientation);
            
            Path vecWaypointsHeadings1;
            vecWaypointsHeadings1.push_back(tPushStart);
            vecWaypointsHeadings1.push_back(tPushGoal);
            
            vector<CubicBezier> tTrajectory1= TrajectoryFromWaypointsAndHeadings(vecWaypointsHeadings1);

            cout<<endl<<"Bezier 2:"<<endl;
            FollowTrajectory(nPusherId,fc,correction,tTrajectory1);
            
        }
    }
    
    /* STEP 2 : Move red boxes in Blue Zone to the Red Zone */
    
    /* REFRESH BOX COORDINATES (GET DATA FROM SERVER AGAIN)*/
    
    data = fc.getFieldData();
    
    vector<Entity> boxes2;
    vector<Box> vecBoxes2;
    vector<Bot> vecBots2;

    while(boxes2.size() < 4){
        data = fc.getFieldData();
        for(unsigned i = 0; i < data.entities.size(); i++){
            if (data.entities[i].id() >= 101 && data.entities[i].id() <= 112) {
                boxes2.push_back(data.entities[i]);
            }
        }
        if (boxes2.size() < 4){
            boxes2.clear();
        }
    }
    
    for (unsigned i = 0; i < boxes2.size(); i++) {
        if (boxes2[i].id() == 102) {
            // Red 1
            vecBoxes2.push_back(Box(boxes2[i],RED,correction));
        } else if (boxes2[i].id() == 112) {
            // Red 2
            vecBoxes2.push_back(Box(boxes2[i],RED,correction));
        } else if (boxes2[i].id() == 101) {
            // Blue 1
            vecBoxes2.push_back(Box(boxes2[i],BLUE,correction));
        } else if (boxes2[i].id() == 103) {
            // Blue 2
            vecBoxes2.push_back(Box(boxes2[i],BLUE,correction));
        }
    }
    
    for (unsigned i = 0; i < data.robots.size(); i++){
        if (data.robots[i].id() == RED_ID){
            vecBots2.push_back(Bot(data.robots[i],RED,correction));
            cout << "Found Red" << endl;
        }
        else if (data.robots[i].id() == GREEN_ID){
            vecBots2.push_back(Bot(data.robots[i],GREEN,correction));
            cout << "Found Green" << endl;
        }
        else if (data.robots[i].id() == BLUE_ID){
            vecBots2.push_back(Bot(data.robots[i],BLUE,correction));
            cout << "Found Blue" << endl;
        }
    }
    
    /* Get centers of the boxes in cm and put them in a TVecCoord */
    TVecCoord tCenterBoxes2;
    for (unsigned i = 0; i < vecBoxes2.size(); i++) {
        tCenterBoxes2.push_back(make_pair(vecBoxes2[i].xCm,vecBoxes2[i].yCm));
    }
    /* Map obstacles */
    TVecCoord tObstacles2=P.MapObstacles(tCenterBoxes2);
    
    /* Push red boxes which are in the Blue Zone to available position in the Red Zone*/
    for (unsigned i = 0; i < vecBoxes2.size(); i++) {
        if (vecBoxes2[i].isHighPriority && vecBoxes2[i].currentStatus==WRONG_SIDE) {
            
            /* Move red robot from its initial position to red box */
            Location tPushStart=vecBoxes2[i].getPushStartPosition(WEST);
            Location tRobotStart;
            Coordinate tGoal=pair<float,float>(tPushStart.X,tPushStart.Y); //
            int nPusherId;
            
            for (vector<Bot>::iterator it=vecBots2.begin(); it!=vecBots2.end(); ++it) {
                if (it->m_tBotColor==BLUE) {
                    tRobotStart=Location(it->m_fXCm,it->m_fYCm,it->m_tBot.theta());
                    nPusherId=it->m_tBot.id();
                    break;
                }
            }
            
            Coordinate tStart=pair<float,float>(tRobotStart.X,tRobotStart.Y);
            
            TVecCoord tAStarPath=P.AStarSearch(tStart,tGoal,tObstacles2);
            TVecCoord tPath=P.SamplePath(tAStarPath);
            
            Path vecWaypointsHeadings=P.GetWaypointsAndHeadings(tPath,tRobotStart.Orientation,tPushStart.Orientation);
            
            vector<CubicBezier> tTrajectory=TrajectoryFromWaypointsAndHeadings(vecWaypointsHeadings);
            
            FollowTrajectory(nPusherId,fc,correction,tTrajectory);
            
            /* Push box to available position in Red Zone */
            vecBoxes2[i].getDiscretePosition();

            float fDistanceToPosition;
            
            if(vecBoxes2[i].currentPosition==BLUE_RIGHT){
                fDistanceToPosition=t_BR2RR;
            }
            else if(vecBoxes2[i].currentPosition==BLUE_LEFT){
                fDistanceToPosition=t_BL2RL;
            }
            

            Location tPushGoal=Location(tPushStart.X+fDistanceToPosition,tPushStart.Y,tPushStart.Orientation);
            
            Path vecWaypointsHeadings1;
            vecWaypointsHeadings1.push_back(tPushStart);
            vecWaypointsHeadings1.push_back(tPushGoal);
            
            vector<CubicBezier> tTrajectory1=TrajectoryFromWaypointsAndHeadings(vecWaypointsHeadings1);
        
            FollowTrajectory(nPusherId,fc,correction,tTrajectory1);
        }
    }
    
    /* STEP 3 : Move red boxes to the End Zone */
    
    /* REFRESH BOX COORDINATES (GET DATA FROM SERVER AGAIN)*/
    data = fc.getFieldData();
    
    vector<Entity> boxes3;
    vector<Box> vecBoxes3;
    vector<Bot> vecBots3;

    while(boxes3.size() < 4){
        data = fc.getFieldData();
        for(unsigned i = 0; i < data.entities.size(); i++){
            if (data.entities[i].id() >= 101 && data.entities[i].id() <= 112) {
                boxes3.push_back(data.entities[i]);
            }
        }
        if (boxes3.size() < 4){
            boxes3.clear();
        }
    }
    
    for (unsigned i = 0; i < boxes3.size(); i++) {
        if (boxes3[i].id() == 102) {
            // Red 1
            vecBoxes3.push_back(Box(boxes3[i],RED,correction));
        } else if (boxes3[i].id() == 112) {
            // Red 2
            vecBoxes3.push_back(Box(boxes3[i],RED,correction));
        } else if (boxes3[i].id() == 101) {
            // Blue 1
            vecBoxes3.push_back(Box(boxes3[i],BLUE,correction));
        } else if (boxes3[i].id() == 103) {
            // Blue 2
            vecBoxes3.push_back(Box(boxes3[i],BLUE,correction));
        }
    }
    
    for (unsigned i = 0; i < data.robots.size(); i++){
        if (data.robots[i].id() == RED_ID){
            vecBots3.push_back(Bot(data.robots[i],RED,correction));
            cout << "Found Red" << endl;
        }
        else if (data.robots[i].id() == GREEN_ID){
            vecBots3.push_back(Bot(data.robots[i],GREEN,correction));
            cout << "Found Green" << endl;
        }
        else if (data.robots[i].id() == BLUE_ID){
            vecBots3.push_back(Bot(data.robots[i],BLUE,correction));
            cout << "Found Blue" << endl;
        }
    }
                    
                    
    /* Get centers of the boxes in cm and put them in a TVecCoord */
    TVecCoord tCenterBoxes3;
    for (unsigned i = 0; i < vecBoxes3.size(); i++) {
        tCenterBoxes3.push_back(make_pair(vecBoxes3[i].xCm,vecBoxes3[i].yCm));
    }
    /* Map obstacles */
    TVecCoord tObstacles3=P.MapObstacles(tCenterBoxes3);
    
    /* Push red boxes in Y direction to get them to end position (done State)*/
    for (unsigned i = 0; i < vecBoxes3.size(); i++) {

        if (vecBoxes3[i].isHighPriority && vecBoxes3[i].currentStatus==CORRECT_SIDE) {
        /* Move red robot from its current position to red box */
        Location tPushStart=vecBoxes3[i].getPushStartPosition(NORTH);
        Location tRobotStart;
        Coordinate tGoal=pair<float,float>(tPushStart.X,tPushStart.Y); //
        int nPusherId;
        
        for (vector<Bot>::iterator it=vecBots3.begin(); it!=vecBots3.end(); ++it) {
            if (it->m_tBotColor==BLUE) {
                tRobotStart=Location(it->m_fXCm,it->m_fYCm,it->m_tBot.theta());
                nPusherId=it->m_tBot.id();
                break;
            }
        }
        
        Coordinate tStart=pair<float,float>(tRobotStart.X,tRobotStart.Y);
        
        TVecCoord tAStarPath=P.AStarSearch(tStart,tGoal,tObstacles3);
        TVecCoord tPath=P.SamplePath(tAStarPath);
        
        Path vecWaypointsHeadings=P.GetWaypointsAndHeadings(tPath,tRobotStart.Orientation,tPushStart.Orientation);
        
        vector<CubicBezier> tTrajectory= TrajectoryFromWaypointsAndHeadings(vecWaypointsHeadings);
        
        FollowTrajectory(nPusherId,fc,correction,tTrajectory);
        
        /* Push box to its position in the End Zone */
        
        vecBoxes3[i].getDiscretePosition();

        float fDistanceToPosition;
        
         if(vecBoxes3[i].currentPosition==RED_RIGHT){
                fDistanceToPosition=t_RR2RRE;
            }
        else if(vecBoxes3[i].currentPosition==RED_LEFT){
                fDistanceToPosition=t_RL2RLE;
            }

        Location tPushGoal=Location(tPushStart.X,tPushStart.Y+fDistanceToPosition,tPushStart.Orientation);
        
        Path vecWaypointsHeadings1;
        vecWaypointsHeadings1.push_back(tPushStart);
        vecWaypointsHeadings1.push_back(tPushGoal);
        
        vector<CubicBezier> tTrajectory1= TrajectoryFromWaypointsAndHeadings(vecWaypointsHeadings1);
        
        FollowTrajectory(nPusherId,fc,correction,tTrajectory1);

        }
    }
    
    
    /* STEP 4 : Move blue boxes in Parking Space to the Blue Zone */
    
    /* REFRESH BOX COORDINATES (GET DATA FROM SERVER AGAIN)*/
    data = fc.getFieldData();
    
    vector<Entity> boxes4;
    vector<Box> vecBoxes4;
    vector<Bot> vecBots4;
    
    while(boxes4.size() < 4){
        data = fc.getFieldData();
        for(unsigned i = 0; i < data.entities.size(); i++){
            if (data.entities[i].id() >= 101 && data.entities[i].id() <= 112) {
                boxes4.push_back(data.entities[i]);
            }
        }
        if (boxes4.size() < 4){
            boxes4.clear();
        }
    }
    
    for (unsigned i = 0; i < boxes4.size(); i++) {
        if (boxes4[i].id() == 102) {
            // Red 1
            vecBoxes4.push_back(Box(boxes4[i],RED,correction));
        } else if (boxes4[i].id() == 112) {
            // Red 2
            vecBoxes4.push_back(Box(boxes4[i],RED,correction));
        } else if (boxes4[i].id() == 101) {
            // Blue 1
            vecBoxes4.push_back(Box(boxes4[i],BLUE,correction));
        } else if (boxes4[i].id() == 103) {
            // Blue 2
            vecBoxes4.push_back(Box(boxes4[i],BLUE,correction));
        }
    }
    
    for (unsigned i = 0; i < data.robots.size(); i++){
        if (data.robots[i].id() == RED_ID){
            vecBots4.push_back(Bot(data.robots[i],RED,correction));
            cout << "Found Red" << endl;
        }
        else if (data.robots[i].id() == GREEN_ID){
            vecBots4.push_back(Bot(data.robots[i],GREEN,correction));
            cout << "Found Green" << endl;
        }
        else if (data.robots[i].id() == BLUE_ID){
            vecBots4.push_back(Bot(data.robots[i],BLUE,correction));
            cout << "Found Blue" << endl;
        }
    }
    
    
    
    /* Get centers of the boxes in cm and put them in a TVecCoord */
    TVecCoord tCenterBoxes4;
    for (unsigned i = 0; i < vecBoxes4.size(); i++) {
        tCenterBoxes4.push_back(make_pair(vecBoxes4[i].xCm,vecBoxes4[i].yCm));
    }
    /* Map obstacles */
    TVecCoord tObstacles4=P.MapObstacles(tCenterBoxes4);
    
    /* Push blue boxes from the parking space to available position in the blue zone */
    for (unsigned i = 0; i < vecBoxes4.size(); i++) {
        
        if (!vecBoxes4[i].isHighPriority && vecBoxes4[i].currentStatus==PARKING) {
        /* Move blue robot from its current position to blue box (with correct orientation) */
        
        Location tPushStart=vecBoxes4[i].getPushStartPosition(WEST);
        Location tRobotStart;
        Coordinate tGoal=pair<float,float>(tPushStart.X,tPushStart.Y); //
        int nPusherId;
        
        for (vector<Bot>::iterator it=vecBots4.begin(); it!=vecBots4.end(); ++it) {
            if (it->m_tBotColor==BLUE) {
                tRobotStart=Location(it->m_fXCm,it->m_fYCm,it->m_tBot.theta());
                nPusherId=it->m_tBot.id();
                break;
            }
        }
        
        Coordinate tStart=pair<float,float>(tRobotStart.X,tRobotStart.Y);
        
        TVecCoord tAStarPath=P.AStarSearch(tStart,tGoal,tObstacles4);
        TVecCoord tPath=P.SamplePath(tAStarPath);
        
        Path vecWaypointsHeadings=P.GetWaypointsAndHeadings(tPath,tRobotStart.Orientation,tPushStart.Orientation);
        
        vector<CubicBezier> tTrajectory= TrajectoryFromWaypointsAndHeadings(vecWaypointsHeadings);
        
        FollowTrajectory(nPusherId,fc,correction,tTrajectory);
        
        /* Push box to available position in Blue Zone */
        
        vecBoxes4[i].getDiscretePosition();
        
        float fDistanceToPosition;

        if(abs(vecBoxes4[i].yCm- t_BRy)<=5){
            fDistanceToPosition=t_P2BR;
        }
               
        else if(abs(vecBoxes4[i].yCm-t_BLy)<=5){
            fDistanceToPosition=t_P2BL;
        }
        
      
        Location tPushGoal=Location(tPushStart.X-fDistanceToPosition,tPushStart.Y,tPushStart.Orientation);
        
        Path vecWaypointsHeadings1;
        vecWaypointsHeadings1.push_back(tPushStart);
        vecWaypointsHeadings1.push_back(tPushGoal);
        
        vector<CubicBezier> tTrajectory1=TrajectoryFromWaypointsAndHeadings(vecWaypointsHeadings1);
        
        FollowTrajectory(nPusherId,fc,correction,tTrajectory1);
        }
    }
    
    /* STEP 5 : Move blue boxes to the End Zone */
    
    /* REFRESH BOX COORDINATES (GET DATA FROM SERVER AGAIN)*/
    data = fc.getFieldData();
    
    vector<Entity> boxes5;
    vector<Box> vecBoxes5;
    vector<Bot> vecBots5;
    
    while(boxes5.size() < 4){
        data = fc.getFieldData();
        for(unsigned i = 0; i < data.entities.size(); i++){
            if (data.entities[i].id() >= 101 && data.entities[i].id() <= 112) {
                boxes5.push_back(data.entities[i]);
            }
        }
        if (boxes5.size() < 4){
            boxes5.clear();
        }
    }
    
    for (unsigned i = 0; i < boxes5.size(); i++) {
        if (boxes5[i].id() == 102) {
            // Red 1
            vecBoxes5.push_back(Box(boxes5[i],RED,correction));
        } else if (boxes5[i].id() == 112) {
            // Red 2
            vecBoxes5.push_back(Box(boxes5[i],RED,correction));
        } else if (boxes5[i].id() == 101) {
            // Blue 1
            vecBoxes5.push_back(Box(boxes5[i],BLUE,correction));
        } else if (boxes5[i].id() == 103) {
            // Blue 2
            vecBoxes5.push_back(Box(boxes5[i],BLUE,correction));
        }
    }
    
    for (unsigned i = 0; i < data.robots.size(); i++){
        if (data.robots[i].id() == RED_ID){
            vecBots5.push_back(Bot(data.robots[i],RED,correction));
            cout << "Found Red" << endl;
        }
        else if (data.robots[i].id() == GREEN_ID){
            vecBots5.push_back(Bot(data.robots[i],GREEN,correction));
            cout << "Found Green" << endl;
        }
        else if (data.robots[i].id() == BLUE_ID){
            vecBots5.push_back(Bot(data.robots[i],BLUE,correction));
            cout << "Found Blue" << endl;
        }
    }
    
    /* Get centers of the boxes in cm and put them in a TVecCoord */
    TVecCoord tCenterBoxes5;
    for (unsigned i = 0; i < vecBoxes5.size(); i++) {
        tCenterBoxes5.push_back(make_pair(vecBoxes5[i].xCm,vecBoxes5[i].yCm));
    }
    /* Map obstacles */
    TVecCoord tObstacles5=P.MapObstacles(tCenterBoxes5);
    
    /* Push blue boxes in Y direction to get them to end position (done State)*/
    for (unsigned i = 0; i < vecBoxes5.size(); i++) {
        if (!vecBoxes5[i].isHighPriority && vecBoxes5[i].currentStatus==CORRECT_SIDE) {
        /* Move blue robot from its current position to blue box */
        Location tPushStart=vecBoxes5[i].getPushStartPosition(NORTH);
        Location tRobotStart;
        Coordinate tGoal=pair<float,float>(tPushStart.X,tPushStart.Y); //
        int nPusherId;
        
        for (vector<Bot>::iterator it=vecBots5.begin(); it!=vecBots5.end(); ++it) {
            if (it->m_tBotColor==BLUE) {
                tRobotStart=Location(it->m_fXCm,it->m_fYCm,it->m_tBot.theta());
                nPusherId=it->m_tBot.id();
                break;
            }
        }
        
        Coordinate tStart=pair<float,float>(tRobotStart.X,tRobotStart.Y);
        
        TVecCoord tAStarPath=P.AStarSearch(tStart,tGoal,tObstacles5);
        TVecCoord tPath=P.SamplePath(tAStarPath);
        
        Path vecWaypointsHeadings=P.GetWaypointsAndHeadings(tPath,tRobotStart.Orientation,tPushStart.Orientation);
        
        vector<CubicBezier> tTrajectory= TrajectoryFromWaypointsAndHeadings(vecWaypointsHeadings);
        
        FollowTrajectory(nPusherId,fc,correction,tTrajectory);
        
        /* Push box to position in End Zone */
        
        vecBoxes5[i].getDiscretePosition();
        
        float fDistanceToPosition;
       if(vecBoxes5[i].currentPosition==BLUE_RIGHT){
                fDistanceToPosition=t_BR2BRE;
        }
        else if(vecBoxes5[i].currentPosition==BLUE_LEFT){
                fDistanceToPosition=t_BL2BLE;
        }

        Location tPushGoal=Location(tPushStart.X,tPushStart.Y+fDistanceToPosition,tPushStart.Orientation);
        
        Path vecWaypointsHeadings1;
        vecWaypointsHeadings1.push_back(tPushStart);
        vecWaypointsHeadings1.push_back(tPushGoal);
        
        vector<CubicBezier> tTrajectory1= TrajectoryFromWaypointsAndHeadings(vecWaypointsHeadings1);
        
        FollowTrajectory(nPusherId,fc,correction,tTrajectory1);
        }
    }
    
//    /* Print outs for debugging */
//    
//    Coordinate tStart=pair<float,float>(10,10);
//    Coordinate tGoal=pair<float,float>(110,110);
//    
//    TVecCoord tAStarPath=P.AStarSearch(tStart,tGoal,tObstacles);
//    
//    cout<<"Center of Boxes:"<<endl;
//    
//    for (TVecCoord::iterator it=tCenterBoxes.begin(); it!=tCenterBoxes.end(); ++it) {
//        cout<<it->first<<'\t'<<it->second<<endl;
//    }
//    
//    cout<<endl<<"Map of obstacles:"<<endl;
//    
//    
//    for (TVecCoord::iterator it=tObstacles.begin(); it!=tObstacles.end(); ++it) {
//        cout<<it->first<<'\t'<<it->second<<endl;
//    }
//    
//    cout<<endl<<"Astar path:"<<endl;
//    
//    for (TVecCoord::iterator it=tAStarPath.begin(); it!=tAStarPath.end(); ++it) {
//        cout<<it->first<<'\t'<<it->second<<endl;
//    }
//


/* Joe's code*/

 //    Entity botRed;
 //    Entity botGreen;
 //    Entity botBlue;

 //    for (unsigned i = 0; i < data.robots.size(); i++){
	// 	if (data.robots[i].id() == RED_ID){
 //            botRed = data.robots[i];
 //            cout << "Found Red" << endl;
	// 	}
	// 	if (data.robots[i].id() == GREEN_ID){
 //            botGreen = data.robots[i];
 //            cout << "Found Green" << endl;
	// 	}
 //        if (data.robots[i].id() == BLUE_ID){
 //            botBlue = data.robots[i];
 //            cout << "Found Blue" << endl;
 //        }
 //    }

	// Matrix<double, 1, 3> startPositionA= correction.correctPerspectiveMetric((double)botGreen.x(),(double)botGreen.y());
	// //Matrix<double, 1, 3> startPositionB= correction.correctPerspectiveMetric((double)bot1.x(),(double)bot1.y());

	// //vector<pair<Point2f,float>> waypoints;
 //    vector<location> waypoints;
 //    //waypoints.push_back(location{(float)startPositionA(0,0),(float)startPositionA(0,1),90});
 //    //waypoints.push_back(location{(float)startPositionA(0,0),(float)startPositionA(0,1)+100,90});

 //    //waypoints.push_back(location{0,0,30});
 //    //waypoints.push_back(location{231,110,30});

 //    waypoints.push_back(Location{50,50,0});
 //    waypoints.push_back(Location{100,50,90});
 //    waypoints.push_back(Location{100,100,180});
 //    waypoints.push_back(Location{100,100,180});

	// //waypoints.push_back(make_pair(make_pair(startPositionA(0,0),startPositionA(0,1)),0));
	// //waypoints.push_back(make_pair(make_pair(startPositionA(0,0),startPositionA(0,1)+75),0));
	// //waypoints.push_back(make_pair(Point2f(startPositionA(0,1)+50,startPositionA(0,0)+50),180));
	// //waypoints.push_back(make_pair(Point2f(startPositionA(0,1)+50,startPositionA(0,0)),270));

 //    //CubicBezier trajectoryA((float)startPositionA(0,0), (float)startPositionA(0,1), 0, (float)startPositionA(0,0), (float)startPositionA(0,1)+50, 0, 1);

 //    //CubicBezier trajectoryB((float)startPositionB(0,0), (float)startPositionB(0,1), 0, (float)startPositionB(0,0), (float)startPositionB(0,1)+50, 0, 1);
	// //CubicBezier trajectory(Point2f(0,0),Point2f(0,50),Point2f(100,50),Point2f(100,100));

	// thread runBotGreen(FollowTrajectory, botGreen.id(), fc, correction, TrajectoryFromWaypointsAndHeadings(waypoints));

 //    //thread robotTracker(TrackRobot, botGreen.id(), correction, fc);

 //    cout << waypoints[0].x << '\t' << waypoints[0].y << endl;
 //    cout << waypoints[1].x << '\t' << waypoints[1].y << endl;
	// runBotGreen.join();

 //    //fc.closeGripper(0);

 //    //Point2f last(0,0);
 //    /*
 //    vector<CubicBezier> bot0path, bot1path;
 //    bot0path.push_back(trajectoryA);
 //    bot1path.push_back(trajectoryB);

 //    thread runBot0(FollowTrajectory, bot0.id(), fc, correction, bot0path);
	// thread runBot1(FollowTrajectory, bot1.id(), fc, correction, bot1path);
	// runBot0.join();
	// runBot1.join();
	// */

	// //for(float i = 0.05; i <= 1; i += 0.05) {
	// 	//Point2f currentPoint = trajectory.GetPoint(i);
	// 	//cout << "X: " << currentPoint.x << " Y: " << currentPoint.y << endl;


	// 	//Matrix<double,1,3> coordinateA = correction.correctPerspectivePixels(trajectoryA.GetPoint(i).x,trajectoryA.GetPoint(i).y);
	// 	//Matrix<double,1,3> coordinateB = correction.correctPerspectivePixels(trajectoryB.GetPoint(i).x,trajectoryB.GetPoint(i).y);

	// 	//thread runBot0(SendToPoint, 0, (int)coordinateA(0,0),(int)coordinateA(0,1), fc);
	// 	//thread runBot1(SendToPoint, 5, (int)coordinateB(0,0),(int)coordinateB(0,1), fc);
	// 	//thread runBot0(FollowTrajectory, 0, fc, correction, bot0path);
	// 	//thread runBot1(FollowTrajectory, 0, fc, correction, bot1path);
	// 	//runBot0.join();
	// 	//runBot1.join();
	// 	/*
	// 	std::ostringstream command;
	// 	command << "./g2p 127.0.0.1 0 " << (int)coordinate(0,0) << " " << (int)coordinate(0,1);
 //    	cout << command.str() << endl;
 //    	const char *c = command.str().c_str();
 //    	std::system(c);
 //    	*/
	// 	//float delta = sqrt(pow(currentPoint.x - last.x,2) + pow(currentPoint.y - last.y,2));
	// 	//cout << "Delta: " << delta << endl;
	// 	//last = currentPoint;
	// //}
	
	// //fc.openGripper(0);
 //    /*
 //    for (float t = 0; t <= 1; t+=0.01) {
 //    	Point2f newPoint = path.GetPoint(t);
 //    	cout << newPoint.x << '\t' << newPoint.y << endl;
 //    }
 //    */

    return 0;
}

