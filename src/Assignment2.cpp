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

// Allowed distance from target is in pixels.
#define ALLOWED_DISTANCE 10

#define RED_ID 3
#define GREEN_ID 0
#define BLUE_ID 5

#define BOX_OFFSET 10

// Width and height of rectangle marked by corner codes in cm
double fieldWidth = 231.14;
double fieldHeight = 109.86;

enum State {WRONG_SIDE, CORRECT_SIDE, PARKING, DONE};
// East = X-positive
// North = Y-positive
// West = X-negative
// South = Y-negative (shouldn't ever have to push south)
enum Direction {EAST, NORTH, WEST, SOUTH};
enum Color{RED, BLUE, GREEN};

/*
struct location {
    float x;
    float y;
    float orientation;
};
*/

class Box {
private:
// Need to reference the perspective correction to get the position of the box in cm
	PerspectiveCorrection correction;
public:
	Entity box;
	Color boxColor;
	State currentStatus;
	
	bool isHighPriority;
	
	int xCm;
	int yCm;
	
	Box(Entity inputBox, Color inputColor, PerspectiveCorrection inputCorrection){
		this->box = inputBox;
		this->boxColor = inputColor;
		this->correction = inputCorrection;
		
		// Perform perspecitve correction to get metric position
		Matrix<double, 1, 3> position= correction.correctPerspectiveMetric(box.x(),box.y());
		
		this->xCm = position(0,0);
		this->yCm = position(0,1);
		
		// Update box position based on field zone rules
		this->currentStatus = updateBoxStatus();
		if (boxColor == Color.RED) {
			isHighPriority = true;
		} else {
			isHighPriority = false;
		}
	}
	
	// Get robot position and orientation needed to start pushing the box in the requested direction
	Location getPushStartPosition(Direction pushDirection) {
		Location output;
		if (pushDirection == Direction.EAST) {
			output.Orientation = 0;
			output.y = yCm;
			output.x = xCm - BOX_OFFSET;
		} else if (pushDirection == Direction.NORTH) {
			output.Orientation = 90;
			output.x = xCm;
			output.y = yCm - BOX_OFFSET;
		} else if (pushDirection == Direction.WEST) {
			output.Orientation = 180;
			output.y = yCm;
			output.x = xCm + BOX_OFFSET;
		} else if (pushDirection == Direction.SOUTH) {
			output.Orientation = 270;
			output.x = xCm;
			output.y = yCm + BOX_OFFSET;
		}
	}
	
	// Update box state based on its position on the field.
	// Red boxes in the red zone (west of centerline) and blue boxes in the blue zone (east of centerline) are on the correct side.
	// Boxes in the opposite color's zone are on the wrong side.
	// Boxes in the parking zone east of the red zone are parking (only low-priority (i.e. blue) boxes should end up here)
	// Boxes in the appropriate color zone and close to the north edge of the field are done being moved
	State updateBoxStatus() {
		State output;
		if (boxColor == Color.RED) {
			if (xCm > fieldWidth/2) {
				output = State.CORRECT_SIDE;
				if (yCm > fieldHeight*0.75) {
					output = State.DONE;
				}
			} else {
				output = State.WRONG_SIDE;
			}
		} else if (boxColor == Color.BLUE) {
			if (xCm <= fieldWidth/2) {
				output = State.CORRECT_SIDE;
				if (yCm > fieldHeight*0.75) {
					output = State.DONE;
				}
			} else if (xCm >= fieldWidth*0.8) {
				output = State.PARKING;
			} else {
				output = State.WRONG_SIDE;
			}
		}
	}		
}

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
		cout << "Current segment: " << i << endl;
        //cout << "Making a curve starting at [" << current.controlPoints[0].x << "," << current.controlPoints[0].y << "]cm and ending at [" << current.controlPoints[3].x << "," << current.controlPoints[3].y << "]" << endl;

		CubicBezier current = trajectory[i];
        float stepSize = 0.05;
		for (float j = 0.2; j < 1 + stepSize; j += stepSize){
			Matrix<double,1,3> coordinate = correction.correctPerspectivePixels(current.GetPoint(j).x,current.GetPoint(j).y);
			cout << "Current Target: #" << j << " " << coordinate << "px " << '\t' << current.GetPoint(j).x << "cmX " << current.GetPoint(j).y << "cmY " << endl;
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
    FieldData data = fc.getFieldData();
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
    string ip = "127.0.0.1";
    FieldComputer fc(ip);
    fc = FieldComputer(ip);
    fc.disableVerbose();
    FieldData data = fc.getFieldData();



	Eigen::Matrix<double, 4, 2> p1;
	Eigen::Matrix<double, 4, 2> p2;
	p2 << 0, 0, 0, fieldHeight, fieldWidth, fieldHeight, fieldWidth, 0;

	vector<Entity> corners;
	vector<Entity> boxes;
	
	//Entity redBox1;
	//Entity redBox2;
	//Entity blueBox1;
	//Entity blueBox2;
	
	Box redBox1;
	Box redBox2;
	Box blueBox1;
	Box blueBox2;
	
	Location boxStartPos0{0,0,0};
	Location boxStartPos1{0,0,0};
	Location boxStartPos2{0,0,0};
	Location boxStartPos3{0,0,0};
	
	Location boxParkingPos0{0,0,0};
	Location boxParkingPos1{0,0,0};
	
	Location boxRedDest0{0,0,0};
	Location boxRedDest1{0,0,0};
	
	Location boxBlueDest0{0,0,0};
	Location boxBlueDest1{0,0,0};

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
	
	for (unsigned i = 0; i < boxes.size(); i++) {
		if (boxes[i].id() == 102) {
			// Red 1
			redBox1 = Box(boxes[i],Color.RED,correction);
		} else if (boxes[i].id() == 112) {
			// Red 2
			redBox2 = Box(boxes[i],Color.RED,correction);
		} else if (boxes[i].id() == 101) {
			// Blue 1
			blueBox1 = Box(boxes[i],Color.BLUE,correction);
		} else if (boxes[i].id() == 103) {
			// Blue 2
			blueBox2 = Box(boxes[i],Color.BLUE,correction);
		}
	}
	
    Matrix<double, 1, 3> origin= correction.correctPerspectiveMetric((double)p1(0,0),(double)p1(0,1));
    Matrix<double, 1, 3> upperRight= correction.correctPerspectiveMetric((double)p1(2,0),(double)p1(2,1));
    cout << "CORRECTION SANITY CHECK:" << endl << "Origin: [" << origin(0,0) << "," << origin(0,1) << "]" << endl << "URC: [" << upperRight(0,0) << "," << upperRight(0,1) << "]" << endl;

    Entity botRed;
    Entity botGreen;
    Entity botBlue;

    for (unsigned i = 0; i < data.robots.size(); i++){
		if (data.robots[i].id() == RED_ID){
            botRed = data.robots[i];
            cout << "Found Red" << endl;
		}
		if (data.robots[i].id() == GREEN_ID){
            botGreen = data.robots[i];
            cout << "Found Green" << endl;
		}
        if (data.robots[i].id() == BLUE_ID){
            botBlue = data.robots[i];
            cout << "Found Blue" << endl;
        }
    }

	Matrix<double, 1, 3> startPositionA= correction.correctPerspectiveMetric((double)botGreen.x(),(double)botGreen.y());
	//Matrix<double, 1, 3> startPositionB= correction.correctPerspectiveMetric((double)bot1.x(),(double)bot1.y());

	//vector<pair<Point2f,float>> waypoints;
    vector<location> waypoints;
    //waypoints.push_back(location{(float)startPositionA(0,0),(float)startPositionA(0,1),90});
    //waypoints.push_back(location{(float)startPositionA(0,0),(float)startPositionA(0,1)+100,90});

    //waypoints.push_back(location{0,0,30});
    //waypoints.push_back(location{231,110,30});

    waypoints.push_back(Location{50,50,0});
    waypoints.push_back(Location{100,50,90});
    waypoints.push_back(Location{100,100,180});
    waypoints.push_back(Location{100,100,180});

	//waypoints.push_back(make_pair(make_pair(startPositionA(0,0),startPositionA(0,1)),0));
	//waypoints.push_back(make_pair(make_pair(startPositionA(0,0),startPositionA(0,1)+75),0));
	//waypoints.push_back(make_pair(Point2f(startPositionA(0,1)+50,startPositionA(0,0)+50),180));
	//waypoints.push_back(make_pair(Point2f(startPositionA(0,1)+50,startPositionA(0,0)),270));

    //CubicBezier trajectoryA((float)startPositionA(0,0), (float)startPositionA(0,1), 0, (float)startPositionA(0,0), (float)startPositionA(0,1)+50, 0, 1);

    //CubicBezier trajectoryB((float)startPositionB(0,0), (float)startPositionB(0,1), 0, (float)startPositionB(0,0), (float)startPositionB(0,1)+50, 0, 1);
	//CubicBezier trajectory(Point2f(0,0),Point2f(0,50),Point2f(100,50),Point2f(100,100));

	thread runBotGreen(FollowTrajectory, botGreen.id(), fc, correction, TrajectoryFromWaypointsAndHeadings(waypoints));

    //thread robotTracker(TrackRobot, botGreen.id(), correction, fc);

    cout << waypoints[0].x << '\t' << waypoints[0].y << endl;
    cout << waypoints[1].x << '\t' << waypoints[1].y << endl;
	runBotGreen.join();

    //fc.closeGripper(0);

    //Point2f last(0,0);
    /*
    vector<CubicBezier> bot0path, bot1path;
    bot0path.push_back(trajectoryA);
    bot1path.push_back(trajectoryB);

    thread runBot0(FollowTrajectory, bot0.id(), fc, correction, bot0path);
	thread runBot1(FollowTrajectory, bot1.id(), fc, correction, bot1path);
	runBot0.join();
	runBot1.join();
	*/

	//for(float i = 0.05; i <= 1; i += 0.05) {
		//Point2f currentPoint = trajectory.GetPoint(i);
		//cout << "X: " << currentPoint.x << " Y: " << currentPoint.y << endl;


		//Matrix<double,1,3> coordinateA = correction.correctPerspectivePixels(trajectoryA.GetPoint(i).x,trajectoryA.GetPoint(i).y);
		//Matrix<double,1,3> coordinateB = correction.correctPerspectivePixels(trajectoryB.GetPoint(i).x,trajectoryB.GetPoint(i).y);

		//thread runBot0(SendToPoint, 0, (int)coordinateA(0,0),(int)coordinateA(0,1), fc);
		//thread runBot1(SendToPoint, 5, (int)coordinateB(0,0),(int)coordinateB(0,1), fc);
		//thread runBot0(FollowTrajectory, 0, fc, correction, bot0path);
		//thread runBot1(FollowTrajectory, 0, fc, correction, bot1path);
		//runBot0.join();
		//runBot1.join();
		/*
		std::ostringstream command;
		command << "./g2p 127.0.0.1 0 " << (int)coordinate(0,0) << " " << (int)coordinate(0,1);
    	cout << command.str() << endl;
    	const char *c = command.str().c_str();
    	std::system(c);
    	*/
		//float delta = sqrt(pow(currentPoint.x - last.x,2) + pow(currentPoint.y - last.y,2));
		//cout << "Delta: " << delta << endl;
		//last = currentPoint;
	//}
	
	//fc.openGripper(0);
    /*
    for (float t = 0; t <= 1; t+=0.01) {
    	Point2f newPoint = path.GetPoint(t);
    	cout << newPoint.x << '\t' << newPoint.y << endl;
    }
    */

    return 0;
}

