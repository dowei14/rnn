#ifndef __ASLCONTROLLER_H
#define __ASLCONTROLLER_H


#include <selforg/abstractcontroller.h>
#include <selforg/controller_misc.h>

// DSW
#include <fourwheeledrpos_gripper.h>
#include <cmath> //pow sqrt
#include <vector>
#include <fstream>
#include "aslt.h"


/*********************************************************************
***  Parameters
*********************************************************************/
	#define number_relative_sensors 3
	#define number_ir_sensors 6


/**
 * Action-Sequence-Learning Controller for 
 * FourWheeldRPos_Gripper(Nimm4 with added sensors and gripper)
 * The controller gets a number of input sensor values each timestep
 * and has to generate a number of output motor values.
 *
 * Dominik Steven Weickgenannt (dowei14@student.sdu.dk 2015/2016)
 */
class ASLController : public AbstractController {
  public:

    //Define global parameters-begin//
    std::vector<double> parameter;

	// DSW reset variable
	bool reset;

	// DSW removeTmpObjects for gripper removal in callback
	bool dropStuff;

    // DSW
    lpzrobots::FourWheeledRPosGripper* vehicle;
    std::vector<lpzrobots::Primitive*> grippables;
	double distances [number_relative_sensors];
	double angles [number_relative_sensors];
	double irSmooth[number_ir_sensors];
	double smoothingFactor;
	
	// trigger detection FF NN
	ASLT* aslt;
	
	// new stuff for RNN
	float triggers[7];
	float triggersDecay[7];
	float neurons[7];
	float weights[7];	
	float neuronsPrev[7];

    
    // some globals used in functions
    bool haveTarget; // does the robot know have a target box? might be removed later
    int currentBox; // current target box
    int state; // state of system 
    int dropBoxCounter; //wait for robot to stand completly still before dropping the box


	// for training
	double prevMotorLeft;
	double prevMotorRight;
	double motorLeft;
	double motorRight;
	bool prevHaveTarget;
	int prevState;
	
	// files for storing
	std::ofstream in11;
	std::ofstream in12;
	std::ofstream in18;
	std::ofstream out1;
	std::ofstream out7;
	std::ofstream outT;
	std::ofstream outD;

	int runNumber;
	int counter;
	
	// sensor values
	double distanceCurrentBox, angleCurrentBox;
	double irLeftLong, irRightLong, irLeftShort, irRightShort;
	double irFront,touchGripper;
	// parameters
	double boxTouching;
	double irFloorDistance;
	double irFrontClearDistance;
	
    //Define global parameters-end//

    /// contructor (hint: use $ID$ for revision)
    ASLController(const std::string& name, const std::string& revision);

    /** initialization of the controller with the given sensor/ motornumber
      Must be called before use. The random generator is optional.
     */
    virtual void init(int sensornumber, int motornumber, RandGen* randGen = 0){
      number_sensors = sensornumber;
      number_motors = motornumber;
    };

    /** @return Number of sensors the controller
      was initialised with or 0 if not initialised */
    virtual int getSensorNumber() const {
      return number_sensors;
    };

    /** @return Number of motors the controller
      was initialised with or 0 if not initialised */
    virtual int getMotorNumber() const {
      return number_motors;
    };

	// perform one step with learning
	virtual void step(const sensor* sensors, int sensornumber, motor* motors, int motornumber);

	// perform one step without learning
    virtual void stepNoLearning(const sensor* , int number_sensors,motor* , int number_motors);

	// pass grippables and vehicle
	virtual void setGrippablesAndVehicle(lpzrobots::FourWheeledRPosGripper* vehicleIn, std::vector<lpzrobots::Primitive*> grippablesIn);

	// DSW Sensors to angle/distance functions
	virtual void calculateDistanceToGoals(const sensor* x_);
	virtual void calculateAnglePositionFromSensors(const sensor* x_);
		
	// Behaviours
	virtual void setTarget(bool& haveTarget);
	virtual void goToRandomBox(double boxDistance, double boxAngle, motor* motors);
	virtual void testBox(double boxDistance, motor* motors, bool& haveTarget);
	virtual void moveToEdge(double irLeft, double irRight, motor* motors);
	virtual void orientAtEdge(double irLeftLong, double irRightLong, double irLeftShort, double irRightShort, motor* motors);
	virtual void dropBox(lpzrobots::FourWheeledRPosGripper* vehicle, int& dropBoxCounter, bool& dropStuff, motor* motors);
	virtual void crossGap(motor* motors);

	// Controller Steps
	virtual void resetParameters();
	virtual void fsmStep(motor* motors);
	virtual void rnnStep(motor* motors);
	virtual void executeAction(motor* motors);

	// DSW return reset variable
	virtual bool getReset() {
		return reset;
	}
	
	virtual bool getDrop(){
		return dropStuff;
	}	


	virtual void setReset(bool input) {
		reset = input;
	}

    /********* STORABLE INTERFACE ******/
    /// @see Storable
    virtual bool store(FILE* f) const {
      Configurable::print(f,"");
      return true;
    }

    /// @see Storable
    virtual bool restore(FILE* f) {
      Configurable::parse(f);
      return true;
    }
    
    // store Data for learning
    virtual void store();
    virtual void storeTriggerBalance();
	virtual void storeDecayBalance();    
    virtual void storebyState();
    virtual void storeSingleTrigger(int action);

  protected:

    int number_sensors;
    int number_motors;

};

#endif
