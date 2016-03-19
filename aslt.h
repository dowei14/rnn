#ifndef ASLT_H_
#define ASLT_H_

#include "utils/ann-framework/ann.h"

/************************************************
*** Trigger 0 detection FF NN
************************************************/
class ASLT0 : public ANN {
public:
    ASLT0();
    Neuron* getNeuronOutput();
};

/************************************************
*** Accumulation of all FF NN
************************************************/
class ASLT : public ANN {
public:

	ASLT();
    const double& getOutputNeuronOutput(const int& index)
    {
      return getOutput(outputNeurons[index]);
    }

    void setInputNeuronInput(const int& index, const double& value)
    {
      setInput(inputNeurons[index], value);
    }
    
    ASLT0* getASLT0(){
    	return aslt0;
    }

	//seems to only do one layer per step, therefor the 5 steps
    void allSteps(){
    	step(); step(); step(); step();	step();
    }

private:
	ASLT0*  aslt0;
	std::vector<Neuron*> inputNeurons;
	std::vector<Neuron*> outputNeurons;
};



#endif /* ASLT_H_ */
