// Copyright 2006-2013 Dr. Marc Andreas Freese. All rights reserved.
// marc@coppeliarobotics.com
// www.coppeliarobotics.com
//
// -------------------------------------------------------------------
// This file is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
//
// You are free to use/modify/distribute this file for whatever purpose!
// -------------------------------------------------------------------
//
// This file was automatically created for V-REP release V3.0.3 on April 29th 2013

#include <unistd.h>
#include "v_repExtAMRProject.h"
#include "v_repLib.h"
#include <iostream>
#include <vector>
#include <map>
#include <ctime>
#include <fenv.h>
#include <time.h>
#include <fstream>
#include <chrono>
#include <pthread.h> 
#include <string>
#include <thread>
#include "Eigen/Core"
#include "Eigen/Dense"
#include "qpOASES.hpp"

#ifdef _WIN32
    #include <shlwapi.h>
    #pragma comment(lib, "Shlwapi.lib")
#endif /* _WIN32 */

#if defined (__linux) || defined (__APPLE__)
    #include <string.h>
    #include <sys/time.h>
    #define _stricmp(x,y) strcasecmp(x,y)
#endif

#define PLUGIN_VERSION 1
#define EIGEN_INITIALIZE_MATRICES_BY_ZERO

LIBRARY vrepLib; // the V-REP library that we will dynamically load and bind

//******************************************list of output files of this plug-in*************************
std::ofstream logfileListNodes;//file of logs which only contains the list of nodes (in case we want to draw the tree with another program)
std::ofstream logfileQPsolver;//file of logs for QP solver
std::ofstream logfileRRTplanning;//file of logs for RRT planning algorithm
std::ofstream report;//file which gives the report for each run of the simulator
bool LOGS_ON = false;//false means we dont write logs in the files (to modify here at necessity) (during simulation losgs must be off)

//******************************************references to v-rep graphics**********************************
simInt hTarget;  // target handle
simInt hFloor;  // floor handle
simInt hFloor_RF; // floor RF handle
simInt hObstacles;       // obstacles handle
simInt hRobot; // robot handle
simInt hTRegion; // tolerance region handle

//******************************************rinformations about objects of the graphics********************
float zIni; // z-component of the robot position (it will be constant during the simulation)
float unicycleRadius;//we imagine a circle aroud the unicycle reference point
float xMin, xMax, yMin, yMax, thetaMin, thetaMax;//min and max values of the coordinates on configuration space
float dt; // control timestep (can be modified directly in the V-REP scene)
int numOfObstacles = 0;//number of obstacles present in the scene

simFloat startTime;//the time when the execution of the planning starts. (just the playback, it doesn't have anything to do with the planning)
bool firstTime = true;//flag needed to execute correctly the founded path (since we need to be synchrounous with V-rep simulation environemet)

//******************************************necessary structures for RRT************************************
//a simple node which compose the tree
class Node{
    public :
        Node* father;
        //q contains x,y,theta;
        Eigen::Vector3f q;
        //inputs to reach this node starting from father (only for playback)
        Eigen::Vector2f uApplied;
    
    Node(Node* f, Eigen::Vector3f q_k){
        father = f;
        q = q_k;
    }
};
std::vector<Node*> tree;//data structure which keep the nodes of the tree
int iterations = 0;//number of iterations that RRT or RRT+CBF did
int maxNumOfIterations;//max allowed number of iterations (input by user)
bool goalRegionReached = false;// this flag become true if the planning algorithm reaches the target area
float delta;//interval of time for which primitives are applied
float subItervals;//number of subintervals for which we do collision checking inside a primitive in RRT algorithm
std::vector<float> v_primitives;//list of v component of primitives
std::vector<float> w_primitives;//list of w component of primitives
Eigen::Vector3f q_goal;//the position of the goal
Node* goalNode = NULL;//the goal node
std::vector<Node*> pathPlannedRRT;//if the planning succeed, here we put the only nodes from start to goal
float gamma1;// (gamma goes from 0 to 1) parameter used to compute distance between 2 node considering DeltaTheta multiplied by gamma
bool isCollisionCheckingEnabledForRRTCBF;//enable or disable collision checking for CBF
bool chooseOfPrimitive;//RRT primitive choose method: TRUE for random choose, FALSE for closest primitive to q_rand
bool isRRTplusCBF;//TRUE execute RRT with cbf, FALSE execute RRT with collision checking
float farthestNodeDistance = 0;//distance from the starting point to the fartest node

//******************************************necessary structures for Trajectory tracking***********************
Eigen::Matrix2f gainMatrix;//the gain matrix used for trajectory tracking (just a test)


//******************************************necessary structures for QP and CBF ********************************
float minimumInputModule;//minimum module accepted as control input in RRT+CBF
float alpha;//the constant we use as K-inf function in the alogithm RRT+CBF
float minimumVModule;//we allow to v to decrease only till a minimum value or could happen that we get a lot of nodes with nodes which contains (v,w)=(0,0)
float b;//offset to consider along the sagittal axis for trajectory tracking or RRT+CBF

int errorQPoases[5] = {0,0,0,0,0};//array which stores the results of QP-oases problem
int discardedQOases1solProblem = 0;//QP oases returned error when we want to get the primal solution
int QPOasesTrivSolNotFound = 0;//we suppose trivial solution is (u,w)=(u_ref,w_ref), we count how many times QPoases could not find this solution
int discardedOutOfWS = 0;//we discarded a node because the trajectory actually goes out of WS
int discardedCollision = 0;//we discarded a node because the trajectory actually collided with an obstacle (checked with collision checking)
int discardedNodes = 0;//number of nodes discarded from the tree because thier distance from another one already belonging to the tree was too small ==(q_new reached with by an input too 
//low )


//******************************************necessary structures at execution time ********************************
bool actSaturation;//flag which apply saturation only at EXECUTION time (not used)
float radiusTolerance;//goal region radius (this variable is actually used also from the planning)
int currentNodeIdx;//index of the node which is currently been executed (for the palyback)
Eigen::Vector3f q_robot_start;// save the starting point of the robot in the 'test primitive' task
int idx_v = 0;// index of the velocity in execution. 'test primitive' task
int idx_w = 0;// index of the velocity in execution. 'test primitive' task


//******************************************time performances variables********************
//duration times for Quadratic problems [microseconds]
int totalQPduration = 0;
int quickerQPduration  = std::numeric_limits<int>::max();;
int slowerQPduration = 0;

//duration times for Collision Checking [microseconds]
int totalCCduration = 0;
int quickerCCduration  = std::numeric_limits<int>::max();;
int slowerCCduration = 0;

//******************************************variables used on menu to choose the activity********************
char desiredTask;//choose the desired task to do





//reset the primitive execution indices
void ResetPrimitivesTest(){
    idx_v = 0;
    idx_w = 0;
}

//when we press STOP we have to reset all data structures before going on with another simulation
void ResetStructures(){
    report.close();
    
    simScaleObject(hTRegion,1/radiusTolerance,1/radiusTolerance,1,0);
    
    ResetPrimitivesTest();
    
    QPOasesTrivSolNotFound = 0;
    for(int i = 0;i<5;i++)
        errorQPoases[i]=0;
    discardedQOases1solProblem = 0;
    discardedNodes = 0;
    discardedCollision = 0;
    discardedOutOfWS = 0;
    goalRegionReached = false;
    firstTime = true;
    for(int i=0; i<tree.size();i++)
        delete tree[i];
    iterations = 0;
    v_primitives.clear();
    w_primitives.clear();
    goalNode = NULL;
    tree.clear();
    pathPlannedRRT.clear();
    if(LOGS_ON){
        logfileListNodes.close();
        logfileQPsolver.close();
        logfileRRTplanning.close();
    }
    totalQPduration = 0;
    quickerQPduration = std::numeric_limits<int>::max();
    slowerQPduration = 0;
    
    totalCCduration = 0;
    quickerCCduration = std::numeric_limits<int>::max();
    slowerCCduration = 0;
    std::cout << "*** Clear done!! ***" <<std::endl;
}

//initialization for trajectory tracking (not need for the project)
void InitTrajectoryTracking(){
    b = 0.4;//offset for trajectory tracking exact FL
    float k1 = 1;//gain 1
    float k2 = 0.1;//gain 2
    
    
    gainMatrix(0,0) = k1;
    gainMatrix(0,1) = 0;
    gainMatrix(1,0) = 0;
    gainMatrix(1,1) = k2;
}

//initialization for RRT and RRT+CBF algorithms
void INIT_RRT_PLANNING(){
    char rrtPlusCbf;
    std::cout << "Use CBF ? ('y' yes, 'n' no, so use collision checking):" << std::endl;
    std::cin >> rrtPlusCbf;
    isRRTplusCBF = (rrtPlusCbf == 'y' ? true : false);
    //isRRTplusCBF = true;
    
    //resulting are all possiblòe compinations of v and w
    v_primitives.push_back(0.5);
    v_primitives.push_back(1);
    
    w_primitives.push_back(-1.3);
    w_primitives.push_back(-0.7);
    w_primitives.push_back(0);
    w_primitives.push_back(0.7);
    w_primitives.push_back(1.3);
     
//      std::cout << "Insert time interval of primitives (def:0.5):" << std::endl;
//      std::cin >> delta;
    delta = 0.5;//interval of time for which primitives are applied
    
    subItervals = 50.0;//subinterval on which we divide the primitive to apply collision detection
    
//     char primChooseMethod;
//     std::cout << "Insert primitives method choose ('r' for random, 'c' for closest to q_rand):" << std::endl;
//     std::cin >> primChooseMethod;
//     chooseOfPrimitive = (primChooseMethod == 'r' ? true : false;
    chooseOfPrimitive = true;//RRT primitive choose method: TRUE for random choose, FALSE for closest primitive to q_rand
    
    isCollisionCheckingEnabledForRRTCBF = false;
    
    std::cout << "Insert max number of iterations for RRT(def:10000):" << std::endl;
    std::cin >> maxNumOfIterations;
    //maxNumOfIterations = 5000;
    
    if(isRRTplusCBF){
//         std::cout << "Insert alpha for CBF(def:2):" << std::endl;
//         std::cin >> alpha;//K-infinity function (constant) for control barrier function 
       alpha = 2;//the higher alpha , the faster get to the obstacles without touch u_ref
    }
    
    gamma1 = 0;
//     std::cout << "Insert gamma multiplying constant for DISTANCE:" << std::endl;
//     std::cout << "(classic RRT works better with gamma = 0)" << std::endl;
//     std::cin >> gamma1;//classic RRT works better with gamma = 0
    
    b = 0.1;//offset to consider along the sagittal axis
    
    unicycleRadius = 0.26;//since unicycle is roughly 1x1, and we compute the distance from the center of unicycle to the nearest object, we need to add a safe distance
    
    //std::cout << "Insert minimum input module accepted(def:0):" << std::endl;
    //std::cin >> minimumInputModule;
     minimumInputModule = 0;//minimumInputModule to be accepted as a node
    
    if(isRRTplusCBF){
        //we allow to v to decrease only till a minimum value or could happen that we get a lot of nodes with nodes which contains (v,w)=(0,0)
//         std::cout << "Insert minimum v module accepted(def:0.1):" << std::endl;
//         std::cin >> minimumVModule;
        minimumVModule = 0.1;
    }
}

//return the current robot position
Eigen::Vector3f getRobotPosition(){
    simFloat pRobot[3];
	simFloat eRobot[3];

	// get the current configuration
	simGetObjectPosition(hRobot, -1, pRobot);
	simGetObjectOrientation(hRobot, -1, eRobot);
    
    Eigen::Vector3f q;
    q << pRobot[0], pRobot[1], eRobot[2];
    return q;
}

//move the robot in the position q
void setRobotPosition(Eigen::Vector3f q){
    simFloat pRobot[3];
	simFloat eRobot[3];
    
    pRobot[0] = q(0);
	pRobot[1] = q(1);
	pRobot[2] = zIni;
    //orientation
	eRobot[0] = 0.0;
	eRobot[1] = 0.0;
	eRobot[2] = q(2);
    
    //apply on the simulation
	simSetObjectPosition(hRobot, -1, pRobot);
	simSetObjectOrientation(hRobot, -1, eRobot);
}

//return the closest point of the closest obstacle to the robot
Eigen::Vector3f getClosestObjPoint(Eigen::Vector3f q){
    simFloat distanceData[7];
    Eigen::Vector3f saveInitPos = getRobotPosition();
    
    setRobotPosition(q);
    simCheckDistance(hRobot, hObstacles,std::numeric_limits<float>::max(),distanceData);
    
    //restore initial position
    setRobotPosition(saveInitPos);
    
    Eigen::Vector3f closestObsPoint;
    closestObsPoint << distanceData[3], distanceData[4], distanceData[5];
    return closestObsPoint;
}

//display the position of the V-REP object, which name is 'name'
void displayPosition(std::string name_S){
    char *cstr = new char[name_S.length() + 1];
    strcpy(cstr, name_S.c_str());
    simChar* name = cstr;
    
    simFloat pObs[3] = {0, 0, 0};
    simInt hObs = simGetObjectHandle(name);
    // get the current configuration
    simGetObjectPosition(hObs, -1, pObs);

    std::cout << name <<" is in position "<< pObs[0] <<","<< pObs[1] << std::endl;
    
}

//initialize the variable q_goal with the current position of the target
void InitTargetPosition(){
    //get target position
    simFloat pTarget[3];
    simGetObjectPosition(hTarget,-1,pTarget);
    q_goal << pTarget[0] , pTarget[1], 0;
}

//get Current Date As String
std::string getCurrentDateAsString(){
    std::time_t rawtime;
    std::time(&rawtime);
    std::tm* timeinfo = std::localtime(&rawtime) ;

    char yyyymmdd[16] ;
    std::strftime( yyyymmdd, sizeof(yyyymmdd), "%Y%m%d_%H%M%S", timeinfo ) ;

    return yyyymmdd;
}

void initReport(){
    std::string reportFileName = isRRTplusCBF ? "REPORT/REPORT_RRT_CBF_" : "REPORT/REPORT_CLASSIC_RRT_";
    report.close();
    report.clear();
    report.open(reportFileName + getCurrentDateAsString() + ".txt"); 
    report.precision(3);
    
    if(isRRTplusCBF)
        report << " *** RRT Motion Planning with CBF" << std::endl;
    else
        report << " *** RRT Motion Planning with collision checking" << std::endl;
    
    report << std::endl;
    
    report << "Configuration space boundaries: x=[" <<xMin<<","<<xMax<<"] y=["<<yMin<<","<<yMax<<"] theta=["<<thetaMin<<",2pi]"<<std::endl;
    
    report << std::endl;
    
    Eigen::Vector3f qrobot = getRobotPosition();
    report << "Robot Position: x=" << qrobot(0) << " y=" << qrobot(1) << std::endl;
    report << "Target Position: x=" << q_goal(0) << " y=" << q_goal(1) << std::endl;
    
    report << std::endl;
    
    for(int i=0;i<numOfObstacles;i++){
        std::string name_S = "Obs"+std::to_string(i+1);
        char *cstr = new char[name_S.length() + 1];
        strcpy(cstr, name_S.c_str());
        simChar* name = cstr;
        
        simFloat pObs[3] = {0, 0, 0};
        simInt hObs = simGetObjectHandle(name);
        // get the current configuration
        simGetObjectPosition(hObs, -1, pObs);
        report << name <<" is in position " << pObs[0] <<","<< pObs[1] <<","<< pObs[2] << std::endl;
    }
    
    report << std::endl;
    report << "************************** RRT PARAMETERS *******************" << std::endl;
    report << std::endl;
    
    report << "Primitives: ";
    for(int i=0; i<v_primitives.size();i++){
        for(int j=0; j<w_primitives.size();j++){
            report << "(" << v_primitives[i] << " , " << w_primitives[j] << ") ";
        }
    }
    report << std::endl;
    report << std::endl;
    
    report << "Time interval (integration of primitives):" << delta << std::endl;
    report << std::endl;
    
    report << "Radius of tolerance around the target (goal region):" << radiusTolerance<< std::endl;
    report << std::endl;
    
    if(!isRRTplusCBF){
        report << "Subintervals in which is divided for collision checking :" << subItervals << std::endl;
        report << std::endl;
    }
    
    report << "RRT primitive choose method:" << (chooseOfPrimitive == true ? " random " : " closest primitive to q_rand ");
    report << std::endl;
    
    if(isRRTplusCBF){
        report << "Collision checking: " << (isCollisionCheckingEnabledForRRTCBF == true ? " enabled " : " disabled");
        report << std::endl;
    }
    report << std::endl;
    
    report << "Max number of iterations:" << maxNumOfIterations << std::endl;
    report << std::endl;
    
    report << "Costant alpha (used as K-inf function for control barrier function):" << alpha << std::endl;
    report << std::endl;
    
    report << "Minimum module of input to be accepted as node:" << minimumInputModule << std::endl;
    report << std::endl;
    
    report << "Minimum v module accepted:" <<  minimumVModule  << std::endl;
    report << std::endl;
    
    
}

void RRTMotionPlanningV2();//just a predeclaration

//GENERAL initialization executed when we press play
void Initialize(){
	std::cout << "Initializing..." << std::endl;	
	// get handles from simulation
	hRobot = simGetObjectHandle("Pioneer_p3dx");
    hTRegion = simGetObjectHandle("Tolerance_region");
	hObstacles = simGetCollectionHandle("Obstacles");
	hTarget = simGetObjectHandle("Target_position");
    hFloor_RF = simGetObjectHandle("Floor_RF");
    hFloor = simGetObjectHandle("Floor");

    simFloat pRobot[3];
	simGetObjectPosition(hRobot, -1, pRobot);
    zIni = pRobot[2];
    Eigen::Vector3f qrobot = getRobotPosition();
	
    std::cout.precision(3);
    std::cout << "Robot Position: ["  << qrobot(0) << "," << qrobot(1) <<"]" << std::endl;
    InitTargetPosition();
    std::cout  << "Target Position: [" << q_goal(0) << "," << q_goal(1) <<"]"<< std::endl; 
    
    // get floor dimensions
    //simFloat sizeFloor_RF[3];
    //simGetObjectSizeValues(hFloor_RF,sizeFloor_RF);
    //simFloat sizeFloor[3];
    //simGetObjectSizeValues(hFloor,sizeFloor);
    //std::cout << "Floor size: (" << sizeFloor[0]*sizeFloor_RF[0]<< "x" << sizeFloor[1]*sizeFloor_RF[1] << ")"<< std::endl;
    std::cout << "Floor size: (5x5)"<< std::endl;
    //Configuration space boundary
    xMin = -2.5; 
    xMax = 2.5; 
    yMin = -2.5;
    yMax = 2.5;
    thetaMin = 0;
    thetaMax = 2*M_PI;
    
    if(LOGS_ON){
        // open file to save data
        logfileListNodes.clear();
        logfileListNodes.open("LOGS/listOfNodes.txt");
        logfileQPsolver.clear();
        logfileQPsolver.open("LOGS/QPsolver.log");
        logfileRRTplanning.clear();
        logfileRRTplanning.open("LOGS/RRTplanning.log");
    }
    /*char sat;
    std::cout << "Include actuator Saturation? (y/n)" << std::endl;
    std::cin >> sat;*/
    actSaturation = false;//OLD
    //InitTrajectoryTracking();
    
    std::cout << "Insert number of obstacles" << std::endl;
    std::cin >> numOfObstacles;
    
    for(int i=1;i<=numOfObstacles;i++){
        displayPosition("Obs"+std::to_string(i));
    }
    
    //Eigen::Vector3f q_closest = getClosestObjPoint(qrobot);
    //std::cout << "Current closest obstacle:" << q_closest.transpose() <<std::endl;
    std::cout << std::endl;
    std::cout << "Insert radius of tolerance region(def:0.2[m]):" << std::endl;
    std::cin >> radiusTolerance;
    
    //simFloat sizeValues[] = {radiusTolerance*2, radiusTolerance*2, 0};
    //simSetObjectSizeValues(hTRegion,sizeValues);
    simScaleObject(hTRegion,radiusTolerance,radiusTolerance,1,0);
    
    INIT_RRT_PLANNING();
    
    initReport();
    
	dt = (float)simGetSimulationTimeStep();
	
	std::cout << "Initialization Completed" << std::endl;	
    std::cout << std::endl;	
    
    std::cout << "Insert the desired action?" << std::endl;	
    std::cout << "(1) planning" << std::endl;	
    std::cout << "(2) Primitives test" << std::endl;	
    std::cout << "(3) regulation task" << std::endl;	
    std::cin >> desiredTask;
    if(desiredTask=='1'){
        //RRTMotionPlanning();
        RRTMotionPlanningV2();
    }
}

//return a value inside range 'min' and 'max', starting from a value which is from -1 and 1
float getValueInRange(float value, float min, float max){
    float temp = value + 1.0;// random in [0,2]
    temp = temp*(max-min)/2.0;// random in [0,range]
    temp = temp+min;// random in [min,max]
    return temp;
}

//return a random q (q_rand)
Eigen::Vector3f getQrand(){
    Eigen::MatrixXd m_rand = Eigen::MatrixXd::Random(3,1);//Eigen random matrices
    Eigen::Vector3f q_rand;
    q_rand(0) = getValueInRange(m_rand(0,0), xMin, xMax);
    q_rand(1) = getValueInRange(m_rand(1,0), yMin, yMax);
    q_rand(2) = getValueInRange(m_rand(2,0), thetaMin, thetaMax);
    return q_rand;
}

//return the distance computed in the plane x y (cartesian distance)
float getXYcartesianDistance(Eigen::Vector3f q1, Eigen::Vector3f q2){
    return sqrt(pow(q1(0)-q2(0),2)+pow(q1(1)-q2(1),2));
}

//return the distance computed in the plane, but considering also the orientation
float getEnachedDistance(Eigen::Vector3f q1, Eigen::Vector3f q2){
    return sqrt(pow(q1(0)-q2(0),2)+pow(q1(1)-q2(1),2)) + gamma1*(q1(2)-q2(2));
}

//return the node belonging to the tree which is the nearest to q_rand
Node* getNodeNear(Eigen::Vector3f q_rand){

    Node* closestOne = NULL;
    float closestDist = std::numeric_limits<float>::max();
    
    for (std::vector<Node*>::iterator it = tree.begin(); it != tree.end(); ++it){
        //float currDist = getXYcartesianDistance((*it)->q, q_rand);
        float currDist = getEnachedDistance((*it)->q, q_rand);
        if(currDist < closestDist){
            closestOne = (*it);
            closestDist = currDist;
        }
    }
    
    return closestOne;
}

//return next position of unicycle starting from q and applayng v and w for deltaT. (used at execution time)
Eigen::Vector3f IntegrateSystem(Eigen::Vector3f q, float v, float w, float deltaT){
    Eigen::Vector3f q_new;
    //Euler
    // integrate the system (to find the next configuration)
	/*q_new(0) = q(0) + deltaT * v * cos(q(2));
	q_new(1) = q(1) + deltaT * v * sin(q(2));		
	q_new(2) = q(2) + deltaT * w;*/
    if(w!=0){
        q_new(2) = q(2) + w*deltaT;
        q_new(0) = q(0) + v/w*(sin(q_new(2)) - sin(q(2)));
        q_new(1) = q(1) - v/w*(cos(q_new(2)) - cos(q(2)));
    }else{
        q_new(0) = q(0) + v*cos(q(2))*deltaT;
        q_new(1) = q(1) + v*sin(q(2))*deltaT;
        q_new(2) = q(2);
    }
    return q_new; 
}

//return q_new starting from q_near and applaying the control input v and w for deltaT (used at planning time)
Eigen::Vector3f computeQnew(Eigen::Vector3f q, float v, float w, float delta){
    Eigen::Vector3f q_new;
    if(w!=0){
        q_new(2) = q(2) + w*delta;
        q_new(0) = q(0) + v/w*(sin(q_new(2)) - sin(q(2)));
        q_new(1) = q(1) - v/w*(cos(q_new(2)) - cos(q(2)));
    }else{
        q_new(0) = q(0) + v*cos(q(2))*delta;
        q_new(1) = q(1) + v*sin(q(2))*delta;
        q_new(2) = q(2);
    }
    return q_new;
}

//return a random primitive from the available list
Eigen::Vector2f  getRandomPrimitive(){
//     Eigen::Vector2f u;
//     srand(time(NULL));
//     int choosenV = rand() % v_primitives.size();  
//     srand(time(NULL));
//     int choosenW = rand() % w_primitives.size();
//     u << v_primitives[choosenV], w_primitives[choosenW];
//     return u;
    Eigen::Vector2f u;
    Eigen::MatrixXd m_rand = Eigen::MatrixXd::Random(2,1);
    int choosenV = getValueInRange(m_rand(0,0), 0, v_primitives.size()-1);  
    int choosenW = getValueInRange(m_rand(0,0), 0, w_primitives.size()-1);  
    u << v_primitives[choosenV], w_primitives[choosenW];
    return u;
}

//get the primitive from the available list which minimize the distance from q_new and q_rand
Eigen::Vector2f  getPrimitiveMinimizingDistance(Eigen::Vector3f q_near, Eigen::Vector3f q_rand){
    //closest to q_rand
    Eigen::Vector2f u;
    float currDist = std::numeric_limits<float>::max();
    
    for(std::vector<float>::iterator it_v = v_primitives.begin(); it_v != v_primitives.end(); ++it_v)
    {
        for(std::vector<float>::iterator it_w = w_primitives.begin(); it_w != w_primitives.end(); ++it_w)
        {
            Eigen::Vector3f q_new = computeQnew(q_near, *it_v, *it_w, delta);
            //std::cout << "v,w" << *it_v << "," << *it_w<< std::endl;
            float dist = getXYcartesianDistance(q_new, q_rand);
            //std::cout << "dist:" << dist << " currDist:" << currDist << std::endl;
            if(currDist > dist){
                currDist = dist; 
                u(0) = *it_v;
                u(1) = *it_w;
            }
        }
    }
    return u;
}

//check is the robot collide with obstacles when it is in position q. (collision checking)
bool isCollisionFree(Eigen::Vector3f q){
    
    Eigen::Vector3f saveInitPos = getRobotPosition();
    
    setRobotPosition(q);
    
    simInt collisionResult = simCheckCollision(hRobot, hObstacles);
    
    //restore initial position
    setRobotPosition(saveInitPos);
//     if (collisionResult==1){
//         std::cout << "COLLISION DETECTED" << std::endl;
//     }
    
    return collisionResult == 0;//true = no collision   
}

//check is the robot collide with obstacles when it goes from q_near to q_new. (collision checking)
bool isCollisionFreePath(Node* nearNode, Node* newNode){
 
    float newT = delta/subItervals; 
    bool isCollFree = true;
    Eigen::Vector3f q_curr = nearNode->q;
    
    for(int i = 0; i < subItervals; i++){
        q_curr =  IntegrateSystem(q_curr, newNode->uApplied(0), newNode->uApplied(1), newT); 
        isCollFree = isCollisionFree(q_curr);
        if(!isCollFree)
            break;
    }
    
    return isCollFree;//true = no collision   
}

//check if the position q in input is out of WS
bool isOutWS(Eigen::Vector3f q){
    return q(0)+unicycleRadius > xMax || q(0)-unicycleRadius < xMin || q(1)+unicycleRadius > yMax || q(1)-unicycleRadius < yMin;
}


//check is the robot collide with obstacles when it goes from q_near to q_new. (collision checking)
bool isPathOutOfWs(Node* nearNode, Node* newNode){
 
    float newT = delta/subItervals; 
    bool isOutOfWs = false;
    Eigen::Vector3f q_curr = nearNode->q;
    
    for(int i = 0; i < subItervals; i++){
        q_curr =  IntegrateSystem(q_curr, newNode->uApplied(0), newNode->uApplied(1), newT); 
        isOutOfWs = isOutWS(q_curr);
        if(isOutOfWs)
            break;
    }
    
    return isOutOfWs;//true = no collision   
}

//return the cartesia distance to the goal (equal to the WS distance)
float getDistanceToGoal(Eigen::Vector3f q){
    
    return sqrt(pow(q(0) - q_goal(0), 2) + pow(q(1) - q_goal(1), 2));
}

//convert QP-Oases return value to string results
std::string returnValueTostring(int simpleRetVal){
    std::string message;
    switch(simpleRetVal){
        case 0: 
            message = "QP was solved";
            break;
        case 1: 
            message = "QP could not be solved within the given number of iterations";
            break;
        case -1: 
            message = "QP could not be solved due to an internal error";
            break;
        case -2: 
            message = "QP is infeasible and thus could not be solved";
            break;
        case -3: 
            message = "QP is unbounded and thus could not be solved"; 
            break;
        default:
            message = "Error code("+std::to_string(simpleRetVal)+") not solve";
    }
    return message;
}

//it displays RRT planning algorithm results
void displayRRTResults(std::string comment){
    std::cout << " *** number of iterations: " << iterations << std::endl;
    std::cout << " *** number of nodes in the tree: " << tree.size() << std::endl;
    std::cout << " *** discarded nodes (because u was too little): " << discardedNodes << std::endl;
    if(isCollisionCheckingEnabledForRRTCBF)
        std::cout << " *** discarded nodes (because actually collided): " << discardedCollision << std::endl;
    std::cout << " *** discarded nodes (because is out of WS): " << discardedOutOfWS << std::endl;
    
    if(isRRTplusCBF){
        for(int i = 0; i<5;i++){
            std::cout << " *** error (QOases init:" << returnValueTostring(i-3) << "): " << errorQPoases[i] << std::endl;
        }
        std::cout << " *** error (QOases was not able to find trivial sol): " << QPOasesTrivSolNotFound << std::endl;
        std::cout << " *** error (because QOases was not able to get 1 sol): " << discardedQOases1solProblem << std::endl;
    }
    
    std::cout << " *** solution " << (goalRegionReached ? "FOUND" : "NOT FOUND") << std::endl; 
    if(comment.size() > 0)
        std::cout << comment << std::endl; 
    std::cout << " *** Farthest distance reached: " << farthestNodeDistance << std::endl;
    
    
    report << " **************************** RESULTS ******************* "<< std::endl;
    report << "Number of iterations: " << iterations << std::endl;
    report << "Number of nodes in the tree: " << tree.size() << std::endl;
    report << "Discarded nodes (because u was too little): " << discardedNodes << std::endl;
    if(isCollisionCheckingEnabledForRRTCBF)
        report << "Discarded nodes (because actually collided): " << discardedCollision << std::endl;
    report << "Discarded nodes (because is out of WS): " << discardedOutOfWS << std::endl;
    
    if(isRRTplusCBF){//QP Oasis report   
        for(int i = 0; i<5;i++){
            report << "Error (QOases init:" << returnValueTostring(i-3) << "): " << errorQPoases[i] << std::endl;
        }
        report << "Error (QOases was not able to find trivial sol): " << QPOasesTrivSolNotFound << std::endl;
        report << "Error (because QOases was not able to get 1 sol): " << discardedQOases1solProblem << std::endl;
    }
    
    report << "Solution " << (goalRegionReached ? "FOUND" : "NOT FOUND") << std::endl; 
    if(comment.size() > 0)
        report << comment << std::endl; 
    report << "Farthest distance reached: " << farthestNodeDistance << std::endl;
}

//return control inputs by solving a QP (v2:only w is part of cost function. v is fixed to a value) (not used in the project)
// Eigen::Vector2f getUcbf_v2(Eigen::Vector3f q, Eigen::Vector2f u_ref){
//     Eigen::Vector2f u;
//     
//     Eigen::Vector3f qObs = getClosestObjPoint(q);
//     //std::cout << "qObs" << qObs.transpose() << std::endl;
//     float x_0 = qObs(0);
//     float y_0 = qObs(1);
//     float x = q(0);
//     float y = q(1);
//     float theta = q(2);
//     
//     float Dx = x-x_0, Dy = y-y_0;
// 	
// 	// set the QP 	
// 	int nVariables = 1;//only w
// 	int nConstraintsLim = 1;
// 
// 	qpOASES::QProblem qp;
// 
// 	qpOASES::real_t H[nVariables*nVariables];
//     qpOASES::real_t A[nVariables];
// 	qpOASES::real_t g[nVariables];
// 	qpOASES::real_t lbA[nConstraintsLim];
// 	qpOASES::real_t ubA[nConstraintsLim];
//     qpOASES::real_t lb[nVariables];
// 	qpOASES::real_t ub[nVariables];
// 
//     //b offset fo the point of unicycle along sagittal axis
//     //float safeDistance = unicycleRadius;
//     float safeDistance = 0;
//     float distance = sqrt(pow(Dx + b*cos(theta),2) + pow(Dy + b*sin(theta),2));
//     A[0] = (b*(Dy*cos(theta) - Dx*sin(theta)))/distance;
// 	
// 	H[0] = 1;	
// 	g[0] = -u_ref(1);
// 	
//     float cbf = distance - safeDistance;
//     lbA[0] = - alpha * cbf - ((b + Dx*cos(theta) + Dy*sin(theta))/distance)*u_ref(0);
//     ubA[0] = std::numeric_limits<float>::max();
// 	
// 
// 	qpOASES::real_t uOpt[nVariables];
// 	qpOASES::int_t nWSR = 1000;
//     
//     lb[0] = -2;//-std::numeric_limits<float>::max(); //to deactivate -inf
// 	ub[0] = 2;//std::numeric_limits<float>::max();//to deactivate  inf
// 
// 	qp = qpOASES::QProblem(nVariables, nConstraintsLim, qpOASES::HST_IDENTITY);
// 	qpOASES::returnValue rv = qp.init(NULL,g,A,lb,ub,lbA,ubA,nWSR);
// // returnValue init( const real_t* H,
// // const real_t* g,
// // const real_t* A,
// // const real_t* lb,
// // const real_t* ub,
// // const real_t* lbA,
// // const real_t* ubA,
// // int& nWSR
// // real_t* const cputime
// // );
// 
//     if(rv != qpOASES::SUCCESSFUL_RETURN){
//         if(LOGS_ON)
//             logfileQPsolver << "ERROR on the solver INITIALIZATION : "<< returnValueTostring(qpOASES::getSimpleStatus(rv)) << std::endl;
//         errorQPoases[qpOASES::getSimpleStatus(rv)+3]++;
//         u << 0, 0;
//     }else{
// 
//         // solve the QP
//         if(qp.getPrimalSolution(uOpt) == qpOASES::RET_QP_NOT_SOLVED){
//             std::cout << "ERROR on the solver" << qpOASES::RET_QP_NOT_SOLVED << std::endl;
//             discardedQOases1solProblem++;
//             u << 0, 0;
//         }else{
//             u << u_ref(0), uOpt[0];
//         }
//     }
//     
//     if(LOGS_ON){
//         logfileQPsolver << "h_dot*uref >= lba_f (uref(v,w): " << u_ref.transpose() << ")  ====>   " << A[0]*u_ref(1) << " >= " << lbA[0] << std::endl;
//         logfileQPsolver << "h_dot*u >= lba_f (u(v,w): " << u.transpose() << ")  ====>   " << A[0]*u(1) << " >= " << lbA[0] << std::endl;
//     }
//     if(A[0]*u_ref(1) >= lbA[0] && (u_ref-u).norm()!=0) {
//         std::cout << "ERROR:Qoases does not find trivial solution!"<<std::endl;
//         QPOasesTrivSolNotFound++;
//          if(LOGS_ON){
//              logfileQPsolver << "ERROR:" <<std::endl;
//          }
//         u = u_ref;
//     }
//     if(LOGS_ON)
//         logfileQPsolver << std::endl;
//     
//     
// 	return u;
//     
// }
// 
//it compute cost function for the specific u (used only for debug)
float computeObjFunction(Eigen::MatrixXf costFunctionH, Eigen::Vector2f costFunctionG,Eigen::Vector2f u_ref, Eigen::Vector2f u){
    return 0.5*(pow(u(0),2)+ pow(u(1),2))+u(0)*costFunctionG(0)+u(1)*costFunctionG(1); 
}

//return control inputs by solving a QP (both velocities are inside the cost function)
Eigen::Vector2f getUcbf(Eigen::Vector3f q, Eigen::Vector2f u_ref){
    auto timeStart = std::chrono::high_resolution_clock::now();
    // vector result
	Eigen::Vector2f u;
    
    // set the QP 	
	int nVariables = 2;//u v
	int nConstraintsLim = 1;//h_dot>=-alpha*h
    
    // cost function matrices 
	Eigen::MatrixXf costFunctionH = Eigen::MatrixXf::Identity(nVariables, nVariables);
	Eigen::Vector2f costFunctionG;
	costFunctionG << -u_ref(0), -u_ref(1);
    
    Eigen::Vector3f qObs = getClosestObjPoint(q);

    float x_0 = qObs(0), y_0 = qObs(1);
    float x = q(0), y = q(1), theta = q(2);
    
    float Dx = x-x_0, Dy = y-y_0;
    
    float safeDistance = unicycleRadius+0.1;
    float distance = sqrt(pow(Dx + b*cos(theta),2) + pow(Dy + b*sin(theta),2));//obstacle-unicycle
    Eigen::Vector2f A_f;//dh/dq * G
    A_f(0) = (b + Dx*cos(theta) + Dy*sin(theta))/distance;
    A_f(1) = (b*(Dy*cos(theta) - Dx*sin(theta)))/distance;
    
    float cbf = distance - safeDistance;
    float lbA_f = -alpha * cbf;
	
	// constraint matrices 
	//Eigen::VectorXf lowerBound(nVariables); // lower bounds for v and omega 
	//lowerBound << minimumVModule, -std::numeric_limits<float>::max(); //to deactivate -inf, -inf
	//Eigen::VectorXf upperBound(nVariables); // upper bounds for v and omega
	//upperBound << std::numeric_limits<float>::max(), std::numeric_limits<float>::max();//to deactivate inf, inf
    
    // constraint matrices (WITH ACTUATOR SATURATION)
	Eigen::VectorXf lowerBound(nVariables); // lower bounds for v and omega 
	lowerBound << minimumVModule, *std::min_element(std::begin(w_primitives),std::end(w_primitives)); //to deactivate -inf, -inf
	Eigen::VectorXf upperBound(nVariables); // upper bounds for v and omega
	upperBound << *std::max_element(std::begin(v_primitives),std::end(v_primitives)), *std::max_element(std::begin(w_primitives),std::end(w_primitives));//to deactivate inf, inf
	
	qpOASES::QProblem qp;

	qpOASES::real_t H[nVariables*nVariables];
    qpOASES::real_t A[nConstraintsLim*nVariables];
	qpOASES::real_t g[nVariables];
	qpOASES::real_t lbA[nConstraintsLim];
	qpOASES::real_t ubA[nConstraintsLim];
    qpOASES::real_t lb[nVariables];
	qpOASES::real_t ub[nVariables];

    for(int j = 0; j < nVariables; j++){
        A[j] = A_f(j);
        lb[j] = lowerBound(j);
        ub[j] = upperBound(j);
    }
	
	for(int i = 0; i < nVariables; i++){
		for(int j = 0; j < nVariables; j++){
			H[i*nVariables+j] = costFunctionH(i,j);
		}
		g[i] = costFunctionG(i);
	}
	
    lbA[0] = lbA_f;
    ubA[0] = std::numeric_limits<float>::max();
	

	qpOASES::real_t uOpt[nVariables];
	qpOASES::int_t nWSR = 1000;
    
    qpOASES::real_t xOpt[nVariables]; 
    xOpt[0] = u_ref(0);
    xOpt[1] = u_ref(1);
    
	qp = qpOASES::QProblem(nVariables, nConstraintsLim, qpOASES::HST_IDENTITY);
	qpOASES::returnValue rv = qp.init(NULL,g,A,lb,ub,lbA,ubA,nWSR,NULL,xOpt,NULL,NULL,NULL);
// returnValue init( const real_t* H,   //nVariables*nVariables
// const real_t* g,         //nVariables
// const real_t* A,         //nConstraintsLim*nVariables
// const real_t* lb,        //nVariables
// const real_t* ub,        //nVariables
// const real_t* lbA,       //nConstraintsLim
// const real_t* ubA,       //nConstraintsLim
// int& nWSR,
// real_t* const cputime,
// const real_t* const xOpt,
// const real_t* const yOpt,
// const Bounds* const guessedBounds,
// const Constraints* const guessedConstraints
// );
    if(rv != qpOASES::SUCCESSFUL_RETURN){
        if(LOGS_ON)
            logfileQPsolver << "ERROR on the solver INITIALIZATION : "<< returnValueTostring(qpOASES::getSimpleStatus(rv)) << std::endl;
        errorQPoases[qpOASES::getSimpleStatus(rv)+3]++;
        u << 0, 0;
    }else{

        // solve the QP
        if(qp.getPrimalSolution(uOpt) == qpOASES::RET_QP_NOT_SOLVED){
            //std::cout << "ERROR on the solver" << qpOASES::RET_QP_NOT_SOLVED << std::endl;
            discardedQOases1solProblem++;
            u << 0, 0;
        }else{
            u << uOpt[0], uOpt[1];
        }
    }
    
    if(LOGS_ON){
        logfileQPsolver << "h_dot*uref >= lba_f (uref(v,w): " << u_ref.transpose() << ")  ====>   " << A_f.transpose()*u_ref << " >= " << lbA_f << std::endl;
        logfileQPsolver << "h_dot*u >= lba_f (u(v,w): " << u.transpose() << ")  ====>   " << A_f.transpose()*u << " >= " << lbA_f << std::endl;
    }
    if(A_f.transpose()*u_ref >= lbA_f && (u_ref-u).norm()!=0) {
        //std::cout << "ERROR:Qoases does not find trivial solution!"<<std::endl;
        QPOasesTrivSolNotFound++;
        if(LOGS_ON){
            logfileQPsolver << "ERROR:" <<  "obj function val computed(u): "<< 2*computeObjFunction(costFunctionH, costFunctionG, u_ref,u) + pow(u_ref(0),2)+ pow(u_ref(1),2)<< " objfunc val computed (uref): "<< 2*computeObjFunction(costFunctionH, costFunctionG, u_ref,u_ref) + pow(u_ref(0),2)+ pow(u_ref(1),2)<<std::endl;
            logfileQPsolver << "ERROR:" <<  "obj function val QOeses: "<<2*qp.getObjVal()+ pow(u_ref(0),2)+ pow(u_ref(1),2)<<std::endl;
        }
        u = u_ref;
    }
    if(LOGS_ON)
        logfileQPsolver << std::endl;
    
    auto timeStop = std::chrono::high_resolution_clock::now();
    auto duration = std::chrono::duration_cast<std::chrono::microseconds>(timeStop-timeStart);
    totalQPduration += duration.count();
    quickerQPduration = duration.count() < quickerQPduration ? duration.count() : quickerQPduration;
    slowerQPduration = duration.count() > slowerQPduration ? duration.count() : slowerQPduration;
	return u;
}

//it writes last things on the report
void concludeReport(int totAlgorithmDuration, float closestObstDist){
    if(goalRegionReached)
    {
        report << " *** Goal region REACHED *** " << std::endl;
        report << "Total number of nodes from root to goal:" << (pathPlannedRRT.size()-1) << std::endl;
        report << "distance from the closest obstacle in the entire path:" << closestObstDist << std::endl;
    }else{
        report << " *** Goal region NOT REACHED *** " << std::endl;   
    }
    report << "Total Execution time: " << totAlgorithmDuration << " [milliseconds]" << std::endl;
    report << std::endl;
    if(isRRTplusCBF){
        report << "Total QP Execution time: " << totalQPduration << " [microseconds]" << std::endl;
        report << "Faster QP Execution time: " << quickerQPduration << " [microseconds]" << std::endl;
        report << "Slower QP Execution time: " << slowerQPduration << " [microeconds]" << std::endl;
        report << "Avg QP Execution time: " << totalQPduration/iterations << " [microseconds]" << std::endl;
    }else{
        report << "Total CC Execution time: " << totalCCduration << " [microseconds]" << std::endl;
        report << "Faster CC Execution time: " << quickerCCduration << " [microseconds]" << std::endl;
        report << "Slower CC Execution time: " << slowerCCduration << " [microseconds]" << std::endl;
        report << "Avg CC Execution time: " << totalCCduration/iterations << " [microseconds]" << std::endl;
    }
    
}

//it get the distance from this point to the closest obstacle
float getDistanceFromThisPointToClosestObs(Eigen::Vector3f q){
    simFloat distanceData[7];
    Eigen::Vector3f saveInitPos = getRobotPosition();
    
    setRobotPosition(q);
    int ret = simCheckDistance(hRobot, hObstacles,std::numeric_limits<float>::max(),distanceData);
    
    //restore initial position
    setRobotPosition(saveInitPos);
    if(ret!=-1)
        return distanceData[6];
    else
        return std::numeric_limits<float>::max();
}

//compute the minimum distance to the obstacles in the current path
float computeClosestObstacleDistance(){
    float minDistance = std::numeric_limits<float>::max();
    
    if(pathPlannedRRT.size() != 0){
        Eigen::Vector3f q_curr = pathPlannedRRT[0]->q;//root
        
        for(int i=1; i<pathPlannedRRT.size(); i++){
            
            float newT = delta/subItervals; 
            for(int j = 0; j < subItervals; j++){
                q_curr =  IntegrateSystem(q_curr, pathPlannedRRT[i]->uApplied(0), pathPlannedRRT[i]->uApplied(1), newT); 
                float dist = getDistanceFromThisPointToClosestObs(q_curr);
                if(minDistance > dist)
                    minDistance = dist;
            }
            
            q_curr = pathPlannedRRT[i]->q;
        }
    }
    return minDistance;
}

//it run RRT Planning algorithm with Control Barrier Function or collision checking depending on which one we choose
void RRTMotionPlanningV2(){
    if(isRRTplusCBF)
        std::cout << " *** RRT Motion Planning with CBF..." << std::endl;
    else
        std::cout << " *** RRT Motion Planning with collision checking..." << std::endl;
    
    if(!isCollisionFree(q_goal)){
        std::cout << "you choose a goal on an obstacle" << std::endl;
        report << "you choose a goal on an obstacle" << std::endl;
        return;
    }
    if(radiusTolerance < 0){
        std::cout << "radius tolerance must be positive" << std::endl;
        report << "radius tolerance must be positive" << std::endl;
        return;
    }
    
    auto timeStart = std::chrono::high_resolution_clock::now();
    
    // get the current configuration
    Eigen::Vector3f q_k = getRobotPosition();

    //set q_start as root node
    Node* root = new Node(NULL, q_k);
    tree.push_back(root);
    
    if(getDistanceToGoal(q_k) > radiusTolerance){
        while(iterations < maxNumOfIterations && !goalRegionReached){

            Eigen::Vector3f q_rand = getQrand();
            Node* nodeNear = getNodeNear(q_rand);

            Eigen::Vector2f u_ref;
            if(chooseOfPrimitive){//TRUE = takes a random primitive
                u_ref = getRandomPrimitive();
            }else{//FALSE: we adopt the strategy of taking the primitives which minimizes the cartesian distance from q_new to q_rand
                u_ref = getPrimitiveMinimizingDistance(nodeNear->q, q_rand);
            }             

            Eigen::Vector2f u;
            if(isRRTplusCBF){
                u = getUcbf(nodeNear->q, u_ref);// get u closest to u_ref modified by CBF
            }else{
                u = u_ref;//classic RRT
            }
            
            //depending on the u we compute q_new
            Eigen::Vector3f q_new = computeQnew(nodeNear->q, u(0), u(1), delta);
            Node* newNode = new Node(nodeNear, q_new);
            newNode->uApplied = u;
            
            if(LOGS_ON){
                logfileRRTplanning << iterations << " *** q_rand: " << q_rand.transpose()<< " q_near: " << nodeNear->q.transpose() << std::endl;
                logfileRRTplanning << iterations << " *** u " << u_ref.transpose() << std::endl;
                if(isRRTplusCBF)
                    logfileRRTplanning << iterations << " *** u CBF " << u.transpose() << std::endl;
                logfileRRTplanning << iterations << " *** q_new: " << q_new.transpose() << std::endl;
                logfileRRTplanning << std::endl;
            }
            
            if((!isOutWS(q_new)) && (!isPathOutOfWs(nodeNear, newNode))){
                bool isqNewCollisionFree = true;
                if(!isRRTplusCBF || isCollisionCheckingEnabledForRRTCBF){
                   auto timeStart_CC = std::chrono::high_resolution_clock::now();
                   isqNewCollisionFree = isCollisionFree(q_new) && isCollisionFreePath(nodeNear, newNode);
                   auto timeStop_CC = std::chrono::high_resolution_clock::now();
                   auto duration_CC = std::chrono::duration_cast<std::chrono::microseconds>(timeStop_CC-timeStart_CC);
                   totalCCduration += duration_CC.count();
                   quickerCCduration = duration_CC.count() < quickerCCduration ? duration_CC.count() : quickerCCduration;
                   slowerCCduration = duration_CC.count() > slowerCCduration ? duration_CC.count() : slowerCCduration;
                }
                
                if(isqNewCollisionFree){
                    if(u.norm() > minimumInputModule){
                        tree.push_back(newNode);// Expand tree
                        float currDist = getXYcartesianDistance(tree[0]->q, newNode->q);
                        farthestNodeDistance = farthestNodeDistance > currDist ? farthestNodeDistance : currDist;
                        // write data on file
                        if(LOGS_ON)
                            logfileListNodes << nodeNear->q.transpose() << " " << newNode->q.transpose() << "\n";
                        goalRegionReached = getDistanceToGoal(q_new) < radiusTolerance;
                        if(goalRegionReached){
                            std::cout << " *** Goal region reached : dist =" << getDistanceToGoal(q_new) << std::endl;
                            goalNode = newNode;
                        }
                    }else{
                        discardedNodes++;
                        //new node discarded
                        delete newNode;
                        newNode = NULL;
                    }
                }else{
                    //new node discarded
                    delete newNode;
                    newNode = NULL;
                    discardedCollision++;
                }
            }else{
                //new node out of WS
                delete newNode;
                newNode = NULL;
                discardedOutOfWS++;
            }
            iterations++;
        }
        displayRRTResults("");
    }else{
        displayRRTResults(" *** initial configuration already on the goal region");
    }
    
    std::cout << " *** RRT Completed" << std::endl;
    auto timeStop = std::chrono::high_resolution_clock::now();
    
    float closestObstDist = 0;
    // if a solution is found, we take the path from goal to root and take control inputs for our robot
    if(goalRegionReached)
    {
        std::cout << " *** Reconstructing path from goal node to root node using built tree" << std::endl;
        Node* currentNode = goalNode;
        while(currentNode != NULL){
            pathPlannedRRT.push_back(currentNode);
            currentNode = currentNode->father;
        }
        
        std::reverse(pathPlannedRRT.begin(),pathPlannedRRT.end());//now from root to goal
        currentNodeIdx = 0;//start from first elem because root has non commands
        std::cout << " *** Total number of nodes from root to goal:" << (pathPlannedRRT.size()-1) << std::endl << "..." << std::endl;
        closestObstDist = computeClosestObstacleDistance();
        
        std::string continua;
        std::cout << "Start simulation? (y/n)" << std::endl;
        std::cin >> continua;
        if(continua == "n" || continua == "N")
            goalRegionReached = false;
    }
    
    auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(timeStop-timeStart);
    concludeReport(duration.count(), closestObstDist);
}

//it run RRT Planning algorithm with collision checking (NOT ANYMORE USED, v2 does both cases)
void RRTMotionPlanning(){
    std::cout << " *** RRT Motion Planning..." << std::endl;
    if(!isCollisionFree(q_goal)){
        std::cout << "you choose a goal on an obstacle" << std::endl;
        return;
    }
    
    // get the current configuration
    Eigen::Vector3f q_k = getRobotPosition();

    //set q_start as root node
    Node* root = new Node(NULL, q_k);
    tree.push_back(root);
    
    if(getDistanceToGoal(q_k) > radiusTolerance){
        while(iterations < maxNumOfIterations && !goalRegionReached){

            Eigen::Vector3f q_rand = getQrand();
            Node* nodeNear = getNodeNear(q_rand);

            //we adopt the strategy of taking the primitives which minimizes the cartesian distance from q_new to q_rand
            Eigen::Vector2f u = getPrimitiveMinimizingDistance(nodeNear->q, q_rand);
            //std::cout << " *** RTT control inputs " << u(0) << "," << u(1) << std::endl;
            Eigen::Vector3f q_new = computeQnew(nodeNear->q, u(0), u(1), delta);
            Node* newNode = new Node(nodeNear, q_new);
            newNode->uApplied = u;
            
            if(isCollisionFree(q_new) && isCollisionFreePath(nodeNear, newNode)){
                tree.push_back(newNode);// Expand tree
                // extrac data on file
                if(LOGS_ON)
                    logfileListNodes << nodeNear->q.transpose() << " " << newNode->q.transpose() << "\n";
                goalRegionReached = getDistanceToGoal(q_new) < radiusTolerance;
                if(goalRegionReached){
                    std::cout << " *** Goal region reached : dist =" << getDistanceToGoal(q_new) << std::endl;
                    goalNode = newNode;
                }
            }else{
                //new node discarded
                delete newNode;
                newNode = NULL;
            }
            //std::cout << " q_new:" << q_new.transpose() << " aggiunto:" << (newNode!=NULL) << std::endl;
            iterations++;
        }
        displayRRTResults("");
    }else{
        displayRRTResults(" *** initial configuration already on the goal region");
    }
    
    std::cout << " *** RRT Completed" << std::endl;
    
    // if a solution is found, we take the path from goal to root and take control inputs for our robot
    if(goalRegionReached)
    {
        std::cout << " *** Reconstructing path from goal node to root node using built tree" << std::endl;
        Node* currentNode = goalNode;
        while(currentNode != NULL){
            pathPlannedRRT.push_back(currentNode);
            currentNode = currentNode->father;
        }
        //pathPlannedRRT.pop_back();
        
        std::reverse(pathPlannedRRT.begin(),pathPlannedRRT.end());//now form root to goal
        currentNodeIdx = 0;//start from first elem because root has non commands
        std::cout << " *** Total number of nodes from root to goal:" << (pathPlannedRRT.size()-1) << std::endl;
        
        std::string continua;
        std::cout << "Start simulation? (y/n)" << std::endl;
        std::cin >> continua;
        if(continua == "n" || continua == "N")
            goalRegionReached = false;
    }
      
}




//********************************************EXECUTION PART ***************************************************




//returns saturated inputs given the u_ref and the max and the min
Eigen::Vector2f computeControlInputsActuatorSat(float v, float omega, Eigen::Vector3f q_k){
	
	// cost function matrices 
	Eigen::MatrixXf costFunctionH = Eigen::MatrixXf::Identity(2, 2);
	Eigen::MatrixXf costFunctionF = Eigen::MatrixXf::Zero(2, 1);
	costFunctionF(0,0) = -v;
	costFunctionF(1,0) = -omega;
	
	// constraint matrices 
	Eigen::VectorXf bLimConstraintLeft(2); // lower bounds for v and omega 
	bLimConstraintLeft << 0.1, -M_PI/2.0; 
	Eigen::VectorXf bLimConstraintRight(2); // upper bounds for v and omega
	bLimConstraintRight << 0.6, M_PI/2.0;
	
	// set the QP 	
	int nVariables = costFunctionH.rows();
	int nConstraintsLim = bLimConstraintLeft.rows();

	qpOASES::QProblem qp;

	qpOASES::real_t H[nVariables*nVariables];
	qpOASES::real_t f[nVariables];
	qpOASES::real_t lb[nConstraintsLim];
	qpOASES::real_t ub[nConstraintsLim];

	for(int i = 0; i < nVariables; i++){
		for(int j = 0; j < nVariables; j++){
			H[i*nVariables+j] = costFunctionH(i,j);
		}
		f[i] = costFunctionF(i);
	}
	
	for(int i = 0; i < nConstraintsLim; i++){
		lb[i] = bLimConstraintLeft(i);
		ub[i] = bLimConstraintRight(i);
	}

	qpOASES::real_t uOpt[nVariables];
	qpOASES::int_t nWSR = 100;

	qp = qpOASES::QProblem(nVariables, 0);
	qp.init(H,f,NULL,lb,ub,NULL,NULL,nWSR);

	// solve the QP
	qp.getPrimalSolution(uOpt);

	// retrieve result
	Eigen::Vector2f u;
	u << uOpt[0], uOpt[1];

	//std::cout << "u:: " << u.transpose() << std::endl;
	
	return u;
}

//it returns the control input in the case of position regulation (target position is q_goal) (not used in the project)
Eigen::Vector2f positionRegulation(Eigen::Vector3f q_k){
    Eigen::Vector2f uNominal;   
 
    float k1 = 1;
    float k2 = 0.1;
    float v = k1*((q_goal(0)-q_k(0))*cos(q_k(2)) + (q_goal(1)-q_k(1))*sin(q_k(2)));
    float omega = k2*(atan2((q_k(1)-q_goal(1)), (q_k(0)-q_goal(0))) - q_k(2) + M_PI);
    
    uNominal(0) = v;
    uNominal(1) = omega;
    return uNominal;
}

//it returns the control input in the case of position regulation (target trajectory is specified by y_d and y_d_dot)(not used in the project)
Eigen::Vector2f trajectoryTracking(Eigen::Vector3f q_k, Eigen::Vector2f y_d_dot, Eigen::Vector2f y_d){
    Eigen::Vector2f uNominal;   
    
    Eigen::Matrix2f decoupMat;
    decoupMat(0,0) = cos(q_k(2));
    decoupMat(0,1) = sin(q_k(2));
    decoupMat(1,0) = -sin(q_k(2))/b;
    decoupMat(1,1) = cos(q_k(2))/b;
    
    Eigen::Vector2f y;
    y(0) = q_k(0) + b*cos(q_k(2));
    y(1) = q_k(1) + b*sin(q_k(2));
    
    uNominal = decoupMat*(y_d_dot + gainMatrix*(y_d - y));
    return uNominal;
}

//return the point at currT for a circular path
Eigen::Matrix2f getCircularPath(simFloat currT){
    Eigen::Matrix2f traj; 
    
    traj(0,0) = -1.5*cos(currT);//y_d1
    traj(1,0) = -1.5*sin(currT);//y_d2
        
    traj(0,1) = 1.5*sin(currT);//y_d_dot1
    traj(1,1) = -1.5*cos(currT);//y_d_dot2

    return traj;
}

//display info about which control is beeing applyed and at which node of the tree we are
void displayExecutionInfo(Eigen::Vector3f qstart, Eigen::Vector3f qgoal, Eigen::Vector2f inputApplied){
    std::cout << " *** current node: " << currentNodeIdx << std::endl;
    std::cout << " *** starting q: " << qstart.transpose() << std::endl;
    std::cout << " *** input applied: " << inputApplied.transpose() << std::endl;
    std::cout << " *** arriving q: " << qgoal.transpose() << std::endl<< std::endl; 
}

//this is the function which execute the regulation task for unicyle (at run time)(not used for the project, it was only a test)
void ExecutionRegulation(){
    
	Eigen::Vector3f q_k, q_kp1;
	float v_k, omega_k;
	
	q_k = getRobotPosition();
	
    bool tochange = false;
    
    if(getDistanceToGoal(q_k) < radiusTolerance){
        //Robot reached goal region
        v_k = 0;
        omega_k = 0;
    }else{
        ///     OLD
        Eigen::Vector2f uNominal = positionRegulation(q_k);
        

        if(actSaturation){
            // compute the control inputs (as close as possible to the nominal ones, but inside the bounds) 
            Eigen::Vector2f u = computeControlInputsActuatorSat(uNominal(0), uNominal(1), q_k);		
            v_k = u(0);
            omega_k = u(1);
        }else{
            Eigen::Vector2f u_filtered;
            if(isRRTplusCBF){
                u_filtered = getUcbf(q_k, uNominal);//u and w
//            Eigen::Vector2f u_filtered = getUcbf_v2(q_k, uNominal);//only w
            }else{
                u_filtered = uNominal;
            }
            
            v_k = u_filtered(0);
            omega_k = u_filtered(1);
        }
    }
	// integrate the system (to find the next configuration)
	q_kp1 = IntegrateSystem(q_k, v_k, omega_k, dt);
    

	// set the robot in the new configuration
    setRobotPosition(q_kp1);
}

void ExecutionTestPrimitives(){
    
    Eigen::Vector3f q_k, q_kp1;
	float v_k, omega_k;
	
	q_k = getRobotPosition();
	
    bool tochange = false;
    
    simFloat currTime = simGetSimulationTime();//current simulation time
    
    if(firstTime){
        std::cin.ignore();
        startTime = currTime;  
        firstTime = false;
        q_robot_start = q_k;
        
        std::cout << "Execution primitive " << (idx_v*w_primitives.size())+idx_w+1 << ": (v,w)=(" << v_primitives[idx_v] << "," << w_primitives[idx_w] << ")" << std::endl;
        std::cout << "Press enter to continue" << std::endl;
        std::cin.ignore();
    }
    if(currTime - startTime >= delta){
        startTime = currTime;
        tochange = true;
    }
    
    // integrate the system (to find the next configuration)
	if(tochange && (idx_v < v_primitives.size()-1 || idx_w < w_primitives.size()-1)){
        if(idx_w < w_primitives.size()-1){
            idx_w++;
        }else{
            idx_w = 0;
            idx_v++;
        }
        q_k = q_robot_start;
        setRobotPosition(q_robot_start);
        
        std::cout << "Execution primitive " << (idx_v*w_primitives.size())+idx_w+1 << ": (v,w)=(" << v_primitives[idx_v] << "," << w_primitives[idx_w] << ")" << std::endl;
        std::cout << "Press enter to continue" << std::endl;
        std::cin.ignore();
    }
    
    v_k = v_primitives[idx_v];
    omega_k = w_primitives[idx_w];
    
    q_kp1 = IntegrateSystem(q_k, v_k, omega_k, dt);
	// set the robot in the new configuration
    setRobotPosition(q_kp1);
}

//execution go the playback of the planned path
void Execution(){
    
	Eigen::Vector3f q_k, q_kp1;
	float v_k, omega_k;
	
	q_k = getRobotPosition();
	
    bool tochange = false;
    
    if(getDistanceToGoal(q_k) < radiusTolerance){
        //Robot reached goal region
        v_k = 0;
        omega_k = 0;
    }else{
        ///     OLD
        //Eigen::Vector2f uNominal = positionRegulation(q_k);
        /*simFloat currTime = simGetSimulationTime();//current time
        Eigen::Matrix2f traj =  getCircularPath(currTime);
        Eigen::Vector2f uNominal = trajectoryTracking(q_k, traj.col(1), traj.col(0));*/
        
        simFloat currTime = simGetSimulationTime();//current simulation time
        Eigen::Vector2f uNominal;
        
        if(firstTime){
            startTime = currTime;  
            firstTime = false;
        }
        
        if(goalRegionReached && currentNodeIdx < pathPlannedRRT.size()){
            if(currTime - startTime >= delta){
                startTime = currTime;
                currentNodeIdx++;
                tochange = true;
                displayExecutionInfo(pathPlannedRRT[currentNodeIdx]->q, pathPlannedRRT[currentNodeIdx+1]->q, pathPlannedRRT[currentNodeIdx+1]->uApplied);
            }
            uNominal = pathPlannedRRT[currentNodeIdx+1]->uApplied;
            
        }else{
            //Sono arrivato nella goal region o non ho trovato il path
            //std::cout << " *** Path non trovato" << std::endl;
            uNominal = Eigen::Vector2f::Zero();
        }

        if(actSaturation){
            // compute the control inputs (as close as possible to the nominal ones, but inside the bounds) 
            Eigen::Vector2f u = computeControlInputsActuatorSat(uNominal(0), uNominal(1), q_k);		
            v_k = u(0);
            omega_k = u(1);
        }else{
            v_k = uNominal(0);
            omega_k = uNominal(1);
        }
    }
	// integrate the system (to find the next configuration)
	if(tochange && currentNodeIdx >0){
        q_kp1 = pathPlannedRRT[currentNodeIdx]->q;//teleport robot in correct position (if this happen something is wrong)
        
    }else{
        q_kp1 = IntegrateSystem(q_k, v_k, omega_k, dt);
    }

	// set the robot in the new configuration
    setRobotPosition(q_kp1);
}


//********************************************V-REP library ***************************************************
// This is the plugin start routine (called just once, just after the plugin was loaded):

VREP_DLLEXPORT unsigned char v_repStart(void* reservedPointer, int reservedInt) {
    // Dynamically load and bind V-REP functions:
    // ******************************************
    // 1. Figure out this plugin's directory:
    char curDirAndFile[1024];
#ifdef _WIN32
    GetModuleFileName(NULL, curDirAndFile, 1023);
    PathRemoveFileSpec(curDirAndFile);
#elif defined (__linux) || defined (__APPLE__)
    getcwd(curDirAndFile, sizeof (curDirAndFile));
#endif
    std::string currentDirAndPath(curDirAndFile);
    // 2. Append the V-REP library's name:
    std::string temp(currentDirAndPath);
#ifdef _WIN32
    temp += "\\v_rep.dll";
#elif defined (__linux)
    temp += "/libv_rep.so";
#elif defined (__APPLE__)
    temp += "/libv_rep.dylib";
#endif /* __linux || __APPLE__ */
    // 3. Load the V-REP library:
    vrepLib = loadVrepLibrary(temp.c_str());
    if (vrepLib == NULL) {
        std::cout << "Error, could not find or correctly load the V-REP library. Cannot start 'PluginSkeleton' plugin.\n";
        return (0); // Means error, V-REP will unload this plugin
    }
    if (getVrepProcAddresses(vrepLib) == 0) {
        std::cout << "Error, could not find all required functions in the V-REP library. Cannot start 'PluginSkeleton' plugin.\n";
        unloadVrepLibrary(vrepLib);
        return (0); // Means error, V-REP will unload this plugin
    }
    // ******************************************

    // Check the version of V-REP:
    // ******************************************
    int vrepVer;
    simGetIntegerParameter(sim_intparam_program_version, &vrepVer);
    if (vrepVer < 20604) // if V-REP version is smaller than 2.06.04
    {
        std::cout << "Sorry, your V-REP copy is somewhat old. Cannot start 'PluginSkeleton' plugin.\n";
        unloadVrepLibrary(vrepLib);
        return (0); // Means error, V-REP will unload this plugin
    }
    // ******************************************

    simLockInterface(1);

    // Here you could handle various initializations
    // Here you could also register custom Lua functions or custom Lua constants
    // etc.

    return (PLUGIN_VERSION); // initialization went fine, we return the version number of this plugin (can be queried with simGetModuleName)
}

// This is the plugin end routine (called just once, when V-REP is ending, i.e. releasing this plugin):

VREP_DLLEXPORT void v_repEnd() {
    // Here you could handle various clean-up tasks

    unloadVrepLibrary(vrepLib); // release the library
}

// This is the plugin messaging routine (i.e. V-REP calls this function very often, with various messages):

VREP_DLLEXPORT void* v_repMessage(int message, int* auxiliaryData, void* customData, int* replyData) { // This is called quite often. Just watch out for messages/events you want to handle
    // Keep following 6 lines at the beginning and unchanged:
    simLockInterface(1);
    static bool refreshDlgFlag = true;
    int errorModeSaved;
    simGetIntegerParameter(sim_intparam_error_report_mode, &errorModeSaved);
    simSetIntegerParameter(sim_intparam_error_report_mode, sim_api_errormessage_ignore);
    void* retVal = NULL;

    // Here we can intercept many messages from V-REP (actually callbacks). Only the most important messages are listed here.
    // For a complete list of messages that you can intercept/react with, search for "sim_message_eventcallback"-type constants
    // in the V-REP user manual.

    if (message == sim_message_eventcallback_refreshdialogs)
        refreshDlgFlag = true; // V-REP dialogs were refreshed. Maybe a good idea to refresh this plugin's dialog too

    if (message == sim_message_eventcallback_menuitemselected) { // A custom menu bar entry was selected..
        // here you could make a plugin's main dialog visible/invisible
    }

    if (message == sim_message_eventcallback_instancepass) { // This message is sent each time the scene was rendered (well, shortly after) (very often)
        // It is important to always correctly react to events in V-REP. This message is the most convenient way to do so:

        int flags = auxiliaryData[0];
        bool sceneContentChanged = ((flags & (1 + 2 + 4 + 8 + 16 + 32 + 64 + 256)) != 0); // object erased, created, model or scene loaded, und/redo called, instance switched, or object scaled since last sim_message_eventcallback_instancepass message
        bool instanceSwitched = ((flags & 64) != 0);

        if (instanceSwitched) {
            // React to an instance switch here!!
        }

        if (sceneContentChanged) { // we actualize plugin objects for changes in the scene

            //...

            refreshDlgFlag = true; // always a good idea to trigger a refresh of this plugin's dialog here
        }
    }

    if (message == sim_message_eventcallback_mainscriptabouttobecalled) { // The main script is about to be run (only called while a simulation is running (and not paused!))
		
    }

    if (message == sim_message_eventcallback_simulationabouttostart) { // Simulation is about to start

		// code written here is run OFF-LINE (before the simulation actually starts)
		Initialize(); 

    }

    if (message == sim_message_eventcallback_simulationended) { // Simulation just ended

        ResetStructures();
		std::cout << "Simulation Concluded" << std::endl;
	
    }

    if (message == sim_message_eventcallback_moduleopen) { // A script called simOpenModule (by default the main script). Is only called during simulation.
        if ((customData == NULL) || (_stricmp("PluginSkeleton", (char*) customData) == 0)) // is the command also meant for this plugin?
        {
            // we arrive here only at the beginning of a simulation
        }
    }

    if (message == sim_message_eventcallback_modulehandle) { // A script called simHandleModule (by default the main script). Is only called during simulation.
	
		// code written here is run ON-LINE (the following function is invoked every dt seconds, then MUST complete in dt seconds)
        if(desiredTask == '1') 
            Execution();
        else if(desiredTask == '2')
            ExecutionTestPrimitives();
        else if(desiredTask == '3')
            ExecutionRegulation();
		
        if ((customData == NULL) || (_stricmp("PluginSkeleton", (char*) customData) == 0)) // is the command also meant for this plugin?
        {
            // we arrive here only while a simulation is running
        }
    }

    if (message == sim_message_eventcallback_moduleclose) { // A script called simCloseModule (by default the main script). Is only called during simulation.
        if ((customData == NULL) || (_stricmp("PluginSkeleton", (char*) customData) == 0)) // is the command also meant for this plugin?
        {
            // we arrive here only at the end of a simulation
        }
    }

    if (message == sim_message_eventcallback_instanceswitch) { // Here the user switched the scene. React to this message in a similar way as you would react to a full
        // scene content change. In this plugin example, we react to an instance switch by reacting to the
        // sim_message_eventcallback_instancepass message and checking the bit 6 (64) of the auxiliaryData[0]
        // (see here above)

    }

    if (message == sim_message_eventcallback_broadcast) { // Here we have a plugin that is broadcasting data (the broadcaster will also receive this data!)

    }

    if (message == sim_message_eventcallback_scenesave) { // The scene is about to be saved. If required do some processing here (e.g. add custom scene data to be serialized with the scene)

    }

    // You can add many more messages to handle here

    if ((message == sim_message_eventcallback_guipass) && refreshDlgFlag) { // handle refresh of the plugin's dialogs
        // ...
        refreshDlgFlag = false;
    }
   
	// Play button is pressed
    if (simGetSimulationState()==17){  
  		
    }
	// Stop button is pressed
	else if(simGetSimulationState()==sim_simulation_stopped){    
		
	}  


    // Keep following unchanged:
    simSetIntegerParameter(sim_intparam_error_report_mode, errorModeSaved); // restore previous settings
    simLockInterface(0);
    return retVal;
}
