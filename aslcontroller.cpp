#include "aslcontroller.h"
#include <iomanip>
/**
 * Action-Sequence-Learning Controller for 
 * FourWheeldRPos_Gripper(Nimm4 with added sensors and gripper)
 * The controller gets a number of input sensor values each timestep
 * and has to generate a number of output motor values.
 *
 * Dominik Steven Weickgenannt (dowei14@student.sdu.dk 2015/2016)
 */
 

ASLController::ASLController(const std::string& name, const std::string& revision)
	: AbstractController(name, revision){

	// DSW
	for (int i=0;i<number_ir_sensors;i++) irSmooth[i]=0;
		
	dropBoxCounter = 0;
	crossGapCounter = 0;
	haveTarget = true;
	prevhaveTarget = false;
	boxGripped = false;
	dropStuff = false;
	done = false;
	reset = false;
	counter = 0;
	
	// parameters
	boxTouching = 0.00115;
	irFloorDistance = 0.5;
	irFrontClearDistance = 0.8;
	smoothingFactor = 1.0;
	state = -1;
	runNumber = 0;
	currentBox = 0;
	prevMotorLeft = 0;
	prevMotorRight = 0;

	// things for plotting
	parameter.resize(8);
	addInspectableValue("parameter1", &parameter.at(0),"parameter1");
	addInspectableValue("parameter2", &parameter.at(1),"parameter2");
	addInspectableValue("parameter3", &parameter.at(2),"parameter3");
	addInspectableValue("parameter4", &parameter.at(3),"parameter4");
	addInspectableValue("parameter5", &parameter.at(4),"parameter5");
	addInspectableValue("parameter6", &parameter.at(5),"parameter6");
	addInspectableValue("parameter7", &parameter.at(6),"parameter7");
	addInspectableValue("parameter8", &parameter.at(7),"parameter8");	
	
	// trigger detection FF NN
	aslt = new ASLT;
	
	// RNN
	for (int i=0; i<8; i++){
		triggers[i] = 0.0;
		triggersDecay[i] = 0.0;
		neurons[i] = 0.0;
		neuronsPrev[i] = 0.0;
	}


}


/*************************************************************************************************
*** performs one step (includes learning).
*** Calculates motor commands from sensor inputs.
***   @param sensors sensors inputs scaled to [-1,1]
***   @param sensornumber length of the sensor array
***   @param motors motors outputs. MUST have enough space for motor values!
***   @param motornumber length of the provided motor array
*************************************************************************************************/
void ASLController::step(const sensor* sensors, int sensornumber,
      motor* motors, int motornumber){
      assert(number_sensors == sensornumber);
      assert(number_motors == motornumber);


	/*****************************************************************************************/
	// motors 0-4
	// motor 0 = left front motor
	// motor 1 = right front motor
	// motor 2 = left hind motor
	// motor 3 = right hind motor

	// sensors 0-3: wheel velocity of the corresponding wheel
	// sensor 0 = wheel velocity left front
	// sensor 1 = wheel velocity right front
	// sensor 2 = wheel velocity left hind
	// sensor 3 = wheel velocity right hind

	// sensors 4-9: IR Sensors
	// sensor 4 = front middle IR
	// sensor 5 = front middle top IR
	// sensor 6 = front right long range IR
	// sensor 7 = front left long range IR
	// sensor 8 = front right short range IR
	// sensor 9 = front left short range IR

	// sensors 10-33: distance to obstacles local coordinates (x,y,z)
	// sensor 10 = x direction to the first object (goal detection sensor)
	// sensor 11 = y direction to the first object (goal detection sensor)
	// sensor 12 = z direction to the first object (goal detection sensor)
      
	// 10-12 : Box 1		(0)
	// 13-15 : Box 2		(1)
	// 16-18 : Box 3		(2)
      
	/*****************************************************************************************/
	// add grippables
	vehicle->addGrippables(grippables);
	
	// calculate relative distances and angles from sensor value, normalized to 0..1 for distance and -1..1 for angle
	calculateDistanceToGoals(sensors);
	calculateAnglePositionFromSensors(sensors);
			
	// smooth ir sensors
//	for (int i=0;i<number_ir_sensors;i++) irSmooth[i] += (sensors[4+i]-irSmooth[i])/smoothingFactor;
	for (int i=0;i<number_ir_sensors;i++) irSmooth[i] = sensors[4+i];



/********************************************************************************************
*** store previous values
********************************************************************************************/

	prevMotorLeft = motors[0];
	prevMotorRight = motors[1];
	prevState = state;
	prevhaveTarget = haveTarget;
	
/********************************************************************************************
*** set up parameters
********************************************************************************************/

	distanceCurrentBox = distances[currentBox];
	angleCurrentBox = angles[currentBox];
	irLeftLong = irSmooth[3];
	irRightLong = irSmooth[2];
	irLeftShort = irSmooth[5];
	irRightShort = irSmooth[4];
	irFront = irSmooth[0];
	if (irSmooth[0] > 0.89) touchGripper = 1.0;
	else touchGripper = 0.0;
	parameter.at(0) = irLeftLong;
	parameter.at(1) = irRightLong;
	parameter.at(2) = irLeftShort;
	parameter.at(3) = irRightShort;	
	parameter.at(4) = touchGripper;
	parameter.at(5) = irFront;	
	parameter.at(6) = distanceCurrentBox;	
	parameter.at(7) = angleCurrentBox;			

	if (reset) resetParameters();

/********************************************************************************************
*** Calculate triggers with NN
********************************************************************************************/
/*
	aslt->getASLT0()->setInput(  0 , prevMotorLeft);
	aslt->getASLT0()->setInput(  1 , prevMotorRight);
	aslt->getASLT0()->setInput(  2 , distanceCurrentBox);
	aslt->getASLT0()->setInput(  3 , angleCurrentBox);
	aslt->getASLT0()->setInput(  4 , irLeftLong);
	aslt->getASLT0()->setInput(  5 , irRightLong);
	aslt->getASLT0()->setInput(  6 , irLeftShort);
	aslt->getASLT0()->setInput(  7 , irRightShort);
	aslt->getASLT0()->setInput(  8 , irFront);
	aslt->getASLT0()->setInput(  9 , touchGripper);
	aslt->getASLT0()->setInput( 10 , boxGripped);									
	aslt->allSteps();
	double val = aslt->getASLT0()->getOutput(16);	
	cout<<abs(round(val))<<endl;
*/
/********************************************************************************************/
	
	
/********************************************************************************************
*** run controller step
********************************************************************************************/	
	rnnStep(motors);
//	fsmStep(motors);

	// if there is no target set target sensors accordingly	
	if (!haveTarget) { 
		distanceCurrentBox = -1.0;
		angleCurrentBox = 0.0;
	}

	executeAction(motors);





	// capping motor speed at +- 1.0
	for (int i=0;i<4;i++){
		if (motors[i]>1.0) motors[i]=1.0;
		if (motors[i]<-1.0) motors[i]=-1.0;	
	}

	/*** STOP FORREST
	for (int i=0;i<4;i++) motors[i]=0.0; // STOP ROBOT
	*************************/

	// store left and right motor values
	motorLeft = motors[0]; motorRight = motors[1]; 

	// Store Values for training
/*	if (!reset && runNumber>0 && counter>1) {
		store();
		storeTriggerBalance();
		storeDecayBalance();
		storebyState();
		storeSingleTrigger(0);
		storeSingleTrigger(1);
	}
*/		
	counter++; // increase counter

		
};


void ASLController::stepNoLearning(const sensor* , int number_sensors,motor* , int number_motors){
	
};

void ASLController::resetParameters(){
	haveTarget = true;
	distanceCurrentBox = -1.0;
	angleCurrentBox = -1.0;
	prevhaveTarget = false;
	boxGripped = false;
	dropStuff = false;
	dropBoxCounter = 0;
	crossGapCounter = 0;
	state = -1;
	counter = 0;

	// RNN reset
	for (int i=0; i<8; i++){
		triggers[i] = 0.0;
		triggersDecay[i] = 0.0;
		neurons[i] = 0.0;
		neuronsPrev[i] = 0.0;
	}
	triggers[0] = 1.0;
	triggersDecay[0] = 1.0;
}

void ASLController::rnnStep(motor* motors){
	/*********************
	** Hand designed RNN
	** ******************/
	if (!reset && (counter > 1)) {
		// create triggers

		// reset triggers to 0
		for (int i=0; i<7; i++)	triggers[i] = 0;

		// alternativelz slowly decay triggers
		for (int i=0; i<7; i++)	triggersDecay[i] *= 0.8;
		if (state==-1) {
			haveTarget = false;
			state = 0;
			triggers[0] = 1.0;
			triggersDecay[0] = 1.0;
		} else if (state==0) {
			if (haveTarget)	{
				state++;
				triggers[1] = 1.0;
				triggersDecay[1] = 1.0;
			}
		} else if (state==1){
			if (touchGripper){
				state++;
				triggers[2] = 1.0;
				triggersDecay[2] = 1.0;
			}
		} else if (state==2){
			if ( motorLeft < -0.5 ){
				if (touchGripper < 1){			
					boxGripped = false;
					haveTarget = false;
					state = 0;
					triggers[0] = 1.0;
					triggersDecay[0] = 1.0;
				} else {
					boxGripped = true;
					state++;
					triggers[3] = 1.0;
					triggersDecay[3] = 1.0;
				}	
			}
		} else if (state ==3){
			if (irLeftLong < irFloorDistance || irRightLong < irFloorDistance) {
				state++;
				triggers[4] = 1.0;
				triggersDecay[4] = 1.0;
			}
		} else if (state ==4){
			if ((irLeftLong < irFloorDistance) && (irRightLong < irFloorDistance) && ((irLeftShort > irFloorDistance) || (irRightShort > irFloorDistance))) {
				state++;
				triggers[5] = 1.0;
				triggersDecay[5] = 1.0;
				motors[0]=0.0; motors[2] = 0.0;
				motors[1]=0.0; motors[3] = 0.0;
			}
		} else if (state ==5){
			if (irFront < irFrontClearDistance){		
				boxGripped = false;
				triggers[6] = 1.0;
				triggersDecay[6] = 1.0;
				state++;
			}
		} else if (state ==6){
			if (irFront > irFrontClearDistance){
				triggers[7] = 1;
				triggersDecay[7] = 1.0;
				state++;
			}
		} else {	
			reset = true;
			runNumber++;
		}
		

		/* print trigger and trigger decay for debuggin purposes
		for (int i=0; i<7; i++) std::cout<<triggers[i]<<" ";
		std::cout<<"____";
		for (int i=0; i<7; i++) std::cout<<triggersDecay[i]<<" ";
		std::cout<<endl;
		*/
		
		for (int i=0; i<7; i++){
			neuronsPrev[i] = neurons[i];
			neurons[i] = neuronsPrev[i] * 0.99 + triggers[i];
		}				
		//for (int i=0; i<7; i++) std::cout<< std::setprecision(1) <<neurons[i]<<" ";
		//std::cout<<endl;

/* softmax
		int max = 0;
		float maxNum = 0;
		for (int i=0; i<7; i++) {
			if (neurons[i]>maxNum){
				max = i; maxNum = neurons[i];
			}
		}
		if (state != max) std::cout<<state<<" "<<max<<endl;					
*/		
	}
}

void ASLController::fsmStep(motor* motors){
	if (!reset &&(counter > 1)) {
		// determine new state
		if (state==-1) {
			haveTarget = false;
			state = 0;
		} else if (state==0) {
			if (prevhaveTarget)	state++;
		} else if (state==1){
			if (distanceCurrentBox <= boxTouching ){
				state++;
			}
		} else if (state==2){
			if ( motorLeft < -0.5 ){
				if (touchGripper < 1){	
					boxGripped = false;
					haveTarget = false;
					prevhaveTarget = false;
					state = 0;
				} else {
					boxGripped = true;
					state++;
				}
			}
		} else if (state ==3){
			if (irLeftLong < irFloorDistance || irRightLong < irFloorDistance) state++;
		} else if (state ==4){
			if ((irLeftLong < irFloorDistance) && (irRightLong < irFloorDistance) && ((irLeftShort > irFloorDistance) || (irRightShort > irFloorDistance))) {
				state++;
				motors[0]=0.0; motors[2] = 0.0;
				motors[1]=0.0; motors[3] = 0.0;
			}
		} else if (state ==5){
			if (irFront < irFrontClearDistance){		
				boxGripped = false;
				state++;
			}
		} else if (state ==6){
			if (irFront > irFrontClearDistance){
				state++;
			}
		} else {		
			reset = true;
			runNumber++;
		}
	}
}

void ASLController::executeAction(motor* motors){
	// execute action
	if (state==0) {
		getTargetAction = true;
		setTarget(haveTarget);
	} else if (state==1){
		getTargetAction = false;
		goToRandomBox(distanceCurrentBox,angleCurrentBox,motors);
	} else if (state==2){
		testBox(distanceCurrentBox,motors);
	} else if (state==3){
		moveToEdge(irLeftLong,irRightLong,motors);
	} else if (state==4){
		orientAtEdge(irLeftLong,irRightLong,irLeftShort,irRightShort,motors);
	} else if (state==5){
		dropBox(vehicle, dropBoxCounter, dropStuff, boxGripped);
	} else if (state==6){
		crossGap(motors, crossGapCounter);
	}
}

/*****************************************************************************************
*** Behaviours
*****************************************************************************************/
void ASLController::setTarget(bool& haveTarget){
	std::random_device rd; // obtain a random number from hardware
	std::mt19937 eng(rd()); // seed the generator
	std::uniform_int_distribution<> distr(0, 2); // define the range
	currentBox = distr(eng);
	haveTarget = true;
}

bool ASLController::goToRandomBox(double boxDistance, double boxAngle, motor* motors)
{
	double left,right;
	bool done = false;
	left = 0.5 + boxAngle;
	right = 0.5 - boxAngle;
	motors[0]=left; motors[2] = left;
	motors[1]=right; motors[3] = right;
	return done;
}

bool ASLController::testBox(double boxDistance, motor* motors){
	double speed;
	bool done = false;
	// stop and accelerate backwards
	if(motors[0] >0) speed = 0;
	else speed = motors[0] - 0.005;
	motors[0]=speed; motors[2] = speed;
	motors[1]=speed; motors[3] = speed;
	return done;
}

bool ASLController::moveToEdge(double irLeft, double irRight, motor* motors){
	bool done = false;
	double left,right;
	
	left = 0.3; right = 0.3;

	motors[0]=left; motors[2] = left;
	motors[1]=right; motors[3] = right;
	return done;
}

bool ASLController::orientAtEdge(double irLeftLong, double irRightLong, double irLeftShort, double irRightShort, motor* motors){
	bool done = false;
	double left = 0.0;
	double right = 0.0;
	double threshold = 0.5;
	double speed = 0.05;
	if( (irLeftLong < threshold) && (irRightLong < threshold) && ((irLeftShort < threshold) || (irRightShort < threshold)) ){
		left = -speed*2; right = -speed*2; // overshoot
	} else if ( (irLeftLong > threshold) && (irRightLong > threshold) && (irLeftShort > threshold) && (irRightShort > threshold) ){
		left = speed;		right = speed; // undershoot
	} else if ( (irLeftLong > threshold) && (irRightLong < threshold) && (irLeftShort > threshold) && (irRightShort < threshold) ){
		left = speed;		right = -speed; // turn right
	} else if ( (irLeftLong < threshold) && (irRightLong > threshold) && (irLeftShort < threshold) && (irRightShort > threshold)){
		left = -speed;	right = speed; // turn left
	} else if ( (irLeftLong < threshold) && (irRightLong > threshold) && (irLeftShort > threshold) && (irRightShort > threshold)){
		left = -speed/2;	right = speed; // turn left with slower back movement
	} else if ( (irLeftLong > threshold) && (irRightLong < threshold) && (irLeftShort > threshold) && (irRightShort > threshold)){
		left = speed;	right = -speed/2; // turn right with slower back movement
	} 

	motors[0]=left; motors[2] = left;
	motors[1]=right; motors[3] = right;
	return done;
}

bool ASLController::dropBox(lpzrobots::FourWheeledRPosGripper* vehicle, int& dropBoxCounter, bool& dropStuff, bool& isGripped){
	bool done = false;	

	// the counter is for the robot to stop moving before dropping, otherwise it will fuck up from time to time
	// TODO: replace this with a "sensor based option"
	dropBoxCounter++;
	if (dropBoxCounter > 50) {
		dropStuff = true;
		vehicle->removeGrippables(grippables);
	}
	return done;
}

bool ASLController::crossGap(motor* motors, int& crossGapCounter){
	bool done = false;
	double speed = 0.5;
	motors[0]=speed; motors[2] = speed;
	motors[1]=speed; motors[3] = speed;
	return done;
}


/*****************************************************************************************
*** Pass grippables and vehicle after reset
*****************************************************************************************/
void ASLController::setGrippablesAndVehicle(lpzrobots::FourWheeledRPosGripper* vehicleIn, std::vector<lpzrobots::Primitive*> grippablesIn){
	vehicle = vehicleIn;
	grippables = grippablesIn;
}

/*****************************************************************************************
*** Sensor Data => relative angle/distance
*****************************************************************************************/
//calculate distances
void ASLController::calculateDistanceToGoals(const sensor* x_)
{
	double distance_scale = 0.0005; // should normalize it to 0..1
	double distance;
	for (int i=0; i<number_relative_sensors; i++){
		distance = (sqrt(pow(x_[11+(i*3) ],2)+pow(x_[10+(i*3)],2)));
		distances[i] = distance * distance_scale;//85;//max_dis;
	}
}

//calculate relative angles
void ASLController::calculateAnglePositionFromSensors(const sensor* x_)
{
	int i=0;
	for (int counter=0; counter<number_relative_sensors; ++counter)
	{
		double alpha_value = 0;
		if (sign(x_[10 + i])>0)
			alpha_value = atan(x_[11 + i]/x_[10 + i]) * 180 / M_PI; // angle in degrees
		else {
			double alpha_value_tmp = -1*atan (x_[11 + i]/x_[10 + i]) * 180 / M_PI; // angle in degrees
			if (alpha_value_tmp<=0)
				alpha_value = (-90 + (-90-alpha_value_tmp));
			else alpha_value = ( 90 + ( 90-alpha_value_tmp));
		}
		angles[counter] = alpha_value/180.*M_PI / M_PI;
		i+=3;
	}
}

/*****************************************************************************************
*** Storing function used to create datasets
***
*** 	store 				-> standard without any seperation of data
*** 	storeTriggerBalance	-> seperates by trigger into + and - samples 
*** 	storeDecayBalance	-> seperates by decay into + and - samples
***		storebyState		-> seperates by state
*****************************************************************************************/
void ASLController::store(){
	// Open Files
	std::string in18name = "../data/in18.txt";
	in18.open (in18name.c_str(), ios::app);
	in18.precision(5);
	in18<<fixed;
	
	std::string in11name = "../data/in11.txt";
	in11.open (in11name.c_str(), ios::app);
	in11.precision(5);
	in11<<fixed;
	
	std::string in12name = "../data/in12.txt";
	in12.open (in12name.c_str(), ios::app);
	in12.precision(5);
	in12<<fixed;

	std::string out7name = "../data/out7.txt";	
	out7.open (out7name.c_str(), ios::app);
	out7.precision(5);
	out7<<fixed;

	std::string out1name = "../data/out1.txt";	
	out1.open (out1name.c_str(), ios::app);
	out1.precision(5);
	out1<<fixed;
	
	std::string outTname = "../data/outT.txt";	
	outT.open (outTname.c_str(), ios::app);
	outT.precision(5);
	outT<<fixed;
	
	std::string outDname = "../data/outD.txt";	
	outD.open (outDname.c_str(), ios::app);
	outD.precision(5);
	outD<<fixed;


	// add binary states to in18 and out7
	for (int i=0;i<7;i++){
		if (i == prevState)	in18<<"1";
		else in18<<"0";
		in18<<" ";
	
		if (i == state)	out7<<"1";
		else out7<<"0";
		if (i<6) out7<<" ";
		if (i==6) out7<<"\n";		
	}

	// add scalar state to in12 and out1
	double multiplier = 0.1;
	in12<<prevState*multiplier<<" ";
	out1<<state*multiplier<<"\n";
	
	// add sensor values to all input files 
	in18<<prevMotorLeft<<" "<<prevMotorRight;	
	in18<<" ";
	in18<<distanceCurrentBox<<" "<<angleCurrentBox;
	in18<<" ";
	in18<<irLeftLong<<" "<<irRightLong<<" "<<irLeftShort<<" "<<irRightShort<<" "<<irFront<<" "<<touchGripper;
	in18<<" ";
	if (boxGripped) in18<<"1";
	else in18<<"0";
	in18<<"\n";

	in11<<prevMotorLeft<<" "<<prevMotorRight;	
	in11<<" ";
	in11<<distanceCurrentBox<<" "<<angleCurrentBox;
	in11<<" ";
	in11<<irLeftLong<<" "<<irRightLong<<" "<<irLeftShort<<" "<<irRightShort<<" "<<irFront<<" "<<touchGripper;
	in11<<" ";
	if (boxGripped) in11<<"1";
	else in11<<"0";
	in11<<"\n";

	in12<<prevMotorLeft<<" "<<prevMotorRight;	
	in12<<" ";
	in12<<distanceCurrentBox<<" "<<angleCurrentBox;
	in12<<" ";
	in12<<irLeftLong<<" "<<irRightLong<<" "<<irLeftShort<<" "<<irRightShort<<" "<<irFront<<" "<<touchGripper;
	in12<<" ";
	if (boxGripped) in12<<"1";
	else in12<<"0";
	in12<<"\n";
	
	
	// add triggers to outT
	for (int i=0; i<7; i++) {
		outT<<triggers[i]<<" ";
	}
	outT<<"\n";

	// add decay to outD
	for (int i=0; i<7; i++) {
		outD<<triggersDecay[i]<<" ";
	}
	outD<<"\n";
	
	// close files	
  	in11.close();
  	in12.close();	
  	in18.close();	
	out1.close();
	out7.close();
	outT.close();
	outD.close();
}

void ASLController::storeTriggerBalance(){
	int p = 0; // positive sample
	for (int i=0; i<7; i++) {
		if (triggers[i] > 0) p = 1;
	}

	// Open Files
	std::string in18name = "../data/T/" + std::to_string(p) + "/in18.txt";
	in18.open (in18name.c_str(), ios::app);
	in18.precision(5);
	in18<<fixed;
	
	std::string in11name = "../data/T/" + std::to_string(p) + "/in11.txt";
	in11.open (in11name.c_str(), ios::app);
	in11.precision(5);
	in11<<fixed;
	
	std::string in12name = "../data/T/" + std::to_string(p) + "/in12.txt";
	in12.open (in12name.c_str(), ios::app);
	in12.precision(5);
	in12<<fixed;

	std::string outTname = "../data/T/" + std::to_string(p) + "/outT.txt";	
	outT.open (outTname.c_str(), ios::app);
	outT.precision(5);
	outT<<fixed;
	
	// add binary states to in18 and out7
	for (int i=0;i<7;i++){
		if (i == prevState)	in18<<"1";
		else in18<<"0";
		in18<<" ";		
	}

	// add scalar state to in12 and out1
	double multiplier = 0.1;
	in12<<prevState*multiplier<<" ";
	
	// add sensor values to all input files 
	in18<<prevMotorLeft<<" "<<prevMotorRight;	
	in18<<" ";
	in18<<distanceCurrentBox<<" "<<angleCurrentBox;
	in18<<" ";
	in18<<irLeftLong<<" "<<irRightLong<<" "<<irLeftShort<<" "<<irRightShort<<" "<<irFront<<" "<<touchGripper;
	in18<<" ";
	if (boxGripped) in18<<"1";
	else in18<<"0";
	in18<<"\n";

	in11<<prevMotorLeft<<" "<<prevMotorRight;	
	in11<<" ";
	in11<<distanceCurrentBox<<" "<<angleCurrentBox;
	in11<<" ";
	in11<<irLeftLong<<" "<<irRightLong<<" "<<irLeftShort<<" "<<irRightShort<<" "<<irFront<<" "<<touchGripper;
	in11<<" ";
	if (boxGripped) in11<<"1";
	else in11<<"0";
	in11<<"\n";

	in12<<prevMotorLeft<<" "<<prevMotorRight;	
	in12<<" ";
	in12<<distanceCurrentBox<<" "<<angleCurrentBox;
	in12<<" ";
	in12<<irLeftLong<<" "<<irRightLong<<" "<<irLeftShort<<" "<<irRightShort<<" "<<irFront<<" "<<touchGripper;
	in12<<" ";
	if (boxGripped) in12<<"1";
	else in12<<"0";
	in12<<"\n";
	
	
	// add triggers to outT
	for (int i=0; i<7; i++) {
		outT<<triggers[i]<<" ";
	}
	outT<<"\n";

	// close files	
  	in11.close();
  	in12.close();	
  	in18.close();	
	outT.close();
}


void ASLController::storeDecayBalance(){
	int p = 0; // positive sample
	for (int i=0; i<7; i++) {
		if (triggersDecay[i] > 0.01) p = 1;
	}

	// Open Files
	std::string in18name = "../data/D/" + std::to_string(p) + "/in18.txt";
	in18.open (in18name.c_str(), ios::app);
	in18.precision(5);
	in18<<fixed;
	
	std::string in11name = "../data/D/" + std::to_string(p) + "/in11.txt";
	in11.open (in11name.c_str(), ios::app);
	in11.precision(5);
	in11<<fixed;
	
	std::string in12name = "../data/D/" + std::to_string(p) + "/in12.txt";
	in12.open (in12name.c_str(), ios::app);
	in12.precision(5);
	in12<<fixed;

	std::string outDname = "../data/D/" + std::to_string(p) + "/outD.txt";	
	outD.open (outDname.c_str(), ios::app);
	outD.precision(5);
	outD<<fixed;
	
	// add binary states to in18 and out7
	for (int i=0;i<7;i++){
		if (i == prevState)	in18<<"1";
		else in18<<"0";
		in18<<" ";		
	}

	// add scalar state to in12 and out1
	double multiplier = 0.1;
	in12<<prevState*multiplier<<" ";
	
	// add sensor values to all input files 
	in18<<prevMotorLeft<<" "<<prevMotorRight;	
	in18<<" ";
	in18<<distanceCurrentBox<<" "<<angleCurrentBox;
	in18<<" ";
	in18<<irLeftLong<<" "<<irRightLong<<" "<<irLeftShort<<" "<<irRightShort<<" "<<irFront<<" "<<touchGripper;
	in18<<" ";
	if (boxGripped) in18<<"1";
	else in18<<"0";
	in18<<"\n";

	in11<<prevMotorLeft<<" "<<prevMotorRight;	
	in11<<" ";
	in11<<distanceCurrentBox<<" "<<angleCurrentBox;
	in11<<" ";
	in11<<irLeftLong<<" "<<irRightLong<<" "<<irLeftShort<<" "<<irRightShort<<" "<<irFront<<" "<<touchGripper;
	in11<<" ";
	if (boxGripped) in11<<"1";
	else in11<<"0";
	in11<<"\n";

	in12<<prevMotorLeft<<" "<<prevMotorRight;	
	in12<<" ";
	in12<<distanceCurrentBox<<" "<<angleCurrentBox;
	in12<<" ";
	in12<<irLeftLong<<" "<<irRightLong<<" "<<irLeftShort<<" "<<irRightShort<<" "<<irFront<<" "<<touchGripper;
	in12<<" ";
	if (boxGripped) in12<<"1";
	else in12<<"0";
	in12<<"\n";
	
	
	// add decay to outD
	for (int i=0; i<7; i++) {
		outD<<triggersDecay[i]<<" ";
	}
	outD<<"\n";

	// close files	
  	in11.close();
  	in12.close();	
  	in18.close();	
	outD.close();
}


void ASLController::storebyState(){

	// Open Files
	std::string in18name = "../data/S/" + std::to_string(prevState) + "/in18.txt";
	in18.open (in18name.c_str(), ios::app);
	in18.precision(5);
	in18<<fixed;
	
	std::string in11name = "../data/S/" + std::to_string(prevState) + "/in11.txt";
	in11.open (in11name.c_str(), ios::app);
	in11.precision(5);
	in11<<fixed;
	
	std::string in12name = "../data/S/" + std::to_string(prevState) + "/in12.txt";
	in12.open (in12name.c_str(), ios::app);
	in12.precision(5);
	in12<<fixed;

	std::string out7name = "../data/S/" + std::to_string(prevState) + "/out7.txt";	
	out7.open (out7name.c_str(), ios::app);
	out7.precision(5);
	out7<<fixed;

	std::string out1name = "../data/S/" + std::to_string(prevState) + "/out1.txt";	
	out1.open (out1name.c_str(), ios::app);
	out1.precision(5);
	out1<<fixed;
	
	std::string outTname = "../data/S/" + std::to_string(prevState) + "/outT.txt";	
	outT.open (outTname.c_str(), ios::app);
	outT.precision(5);
	outT<<fixed;
	
	std::string outDname = "../data/S/" + std::to_string(prevState) + "/outD.txt";	
	outD.open (outDname.c_str(), ios::app);
	outD.precision(5);
	outD<<fixed;


	// add binary states to in18 and out7
	for (int i=0;i<7;i++){
		if (i == prevState)	in18<<"1";
		else in18<<"0";
		in18<<" ";
	
		if (i == state)	out7<<"1";
		else out7<<"0";
		if (i<6) out7<<" ";
		if (i==6) out7<<"\n";		
	}

	// add scalar state to in12 and out1
	double multiplier = 0.1;
	in12<<prevState*multiplier<<" ";
	out1<<state*multiplier<<"\n";
	
	// add sensor values to all input files 
	in18<<prevMotorLeft<<" "<<prevMotorRight;	
	in18<<" ";
	in18<<distanceCurrentBox<<" "<<angleCurrentBox;
	in18<<" ";
	in18<<irLeftLong<<" "<<irRightLong<<" "<<irLeftShort<<" "<<irRightShort<<" "<<irFront<<" "<<touchGripper;
	in18<<" ";
	if (boxGripped) in18<<"1";
	else in18<<"0";
	in18<<"\n";

	in11<<prevMotorLeft<<" "<<prevMotorRight;	
	in11<<" ";
	in11<<distanceCurrentBox<<" "<<angleCurrentBox;
	in11<<" ";
	in11<<irLeftLong<<" "<<irRightLong<<" "<<irLeftShort<<" "<<irRightShort<<" "<<irFront<<" "<<touchGripper;
	in11<<" ";
	if (boxGripped) in11<<"1";
	else in11<<"0";
	in11<<"\n";

	in12<<prevMotorLeft<<" "<<prevMotorRight;	
	in12<<" ";
	in12<<distanceCurrentBox<<" "<<angleCurrentBox;
	in12<<" ";
	in12<<irLeftLong<<" "<<irRightLong<<" "<<irLeftShort<<" "<<irRightShort<<" "<<irFront<<" "<<touchGripper;
	in12<<" ";
	if (boxGripped) in12<<"1";
	else in12<<"0";
	in12<<"\n";
	
	
	// add triggers to outT
	for (int i=0; i<7; i++) {
		outT<<triggers[i]<<" ";
	}
	outT<<"\n";

	// add decay to outD
	for (int i=0; i<7; i++) {
		outD<<triggersDecay[i]<<" ";
	}
	outD<<"\n";
	
	// close files	
  	in11.close();
  	in12.close();	
  	in18.close();	
	out1.close();
	out7.close();
	outT.close();
	outD.close();
}

void ASLController::storeSingleTrigger(int action){

	// training data
	int p = 0; // positive sample
	if (triggers[action] > 0) p = 1;

	// Open Files
	std::string in11name = "../data/ST/" + std::to_string(action) + "/" + std::to_string(p) + "/in11.txt";
	in11.open (in11name.c_str(), ios::app);
	in11.precision(5);
	in11<<fixed;
	
	std::string outTname = "../data/ST/" + std::to_string(action) + "/"  + std::to_string(p) + "/outT.txt";	
	outT.open (outTname.c_str(), ios::app);
	outT.precision(5);
	outT<<fixed;
	

	in11<<prevMotorLeft<<" "<<prevMotorRight;	
	in11<<" ";
	in11<<distanceCurrentBox<<" "<<angleCurrentBox;
	in11<<" ";
	in11<<irLeftLong<<" "<<irRightLong<<" "<<irLeftShort<<" "<<irRightShort<<" "<<irFront<<" "<<touchGripper;
	in11<<" ";
	if (boxGripped) in11<<"1";
	else in11<<"0";
	in11<<"\n";
	
	// add triggers to outT
	outT<<triggers[action]<<" "<<"\n";

	// close files	
  	in11.close();
	outT.close();
	
	// testing data
	p = 0; // positive sample
	if (triggers[action] > 0) p = 1;

	// Open Files
	in11name = "../data/ST/" + std::to_string(action) + "/TEST/in11.txt";
	in11.open (in11name.c_str(), ios::app);
	in11.precision(5);
	in11<<fixed;
	
	outTname = "../data/ST/" + std::to_string(action) + "/TEST/outT.txt";	
	outT.open (outTname.c_str(), ios::app);
	outT.precision(5);
	outT<<fixed;
	

	in11<<prevMotorLeft<<" "<<prevMotorRight;	
	in11<<" ";
	in11<<distanceCurrentBox<<" "<<angleCurrentBox;
	in11<<" ";
	in11<<irLeftLong<<" "<<irRightLong<<" "<<irLeftShort<<" "<<irRightShort<<" "<<irFront<<" "<<touchGripper;
	in11<<" ";
	if (boxGripped) in11<<"1";
	else in11<<"0";
	in11<<"\n";
	
	// add triggers to outT
	outT<<triggers[action]<<" "<<"\n";

	// close files	
  	in11.close();
	outT.close();	
}
