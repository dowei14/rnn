#ifndef __ASLCONTROLLER_H
#define __ASLCONTROLLER_H


#include <selforg/abstractcontroller.h>
#include <selforg/controller_misc.h>

// DSW
#include <fourwheeledrpos_gripper.h>
#include <cmath> //pow sqrt
#include <vector>
#include <fstream>


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
	
	// new stuff for RNN
	float triggers[8];
	float triggersDecay[8];
	float neurons[8];
	float neuronsPrev[8];
	std::ofstream outTrigger;
	std::ofstream outTriggerDecay;
    
    // TODO: remove these, artifacts from previous iterations
    bool done;
    bool haveTarget;
    int currentBox;
    int state;
    bool boxGripped;
    int testBoxCounter;
    int dropBoxCounter;
	int crossGapCounter;
	bool atEdge;
	bool atBox;
	bool nearEdge;


	// for training
	double prevMotorLeft;
	double prevMotorRight;
	double motorLeft;
	double motorRight;
	bool prevhaveTarget;
	bool getTargetAction;
	int prevState;
	// files for storing
	std::ofstream inRNN;
	std::ofstream inRNN2;
	std::ofstream inRNN3;
	std::ofstream outRNN;
	std::ofstream outRNN3;
	std::ofstream inCSMTL;
	std::ofstream outCSMTL;

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
		
	// Actions
	// TODO: Make them void functions
	virtual void setTarget(bool& haveTarget);
	virtual bool goToRandomBox(double boxDistance, double boxAngle, motor* motors);
	virtual bool testBox(double boxDistance, motor* motors, int& testBoxCounter, bool& isGripped);
	virtual bool moveToEdge(double irLeft, double irRight, motor* motors);
	virtual bool orientAtEdge(double irLeftLong, double irRightLong, double irLeftShort, double irRightShort, motor* motors, bool& atEdge);
	virtual bool dropBox(lpzrobots::FourWheeledRPosGripper* vehicle, int& dropBoxCounter, bool& dropStuff, bool& isGripped);
	virtual bool crossGap(motor* motors, int& crossGapCounter);

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
    virtual void storeBySkillCSMTL();

  protected:

    int number_sensors;
    int number_motors;

};

#endif
