#include "aslt.h"

ASLT::ASLT() {
		//setDefaultTransferFunction(ANN::identityFunction());

		// create 11 input neurons
//		for (int i=0; i<11; i++) inputNeurons.push_back(addNeuron());
//		for (int i=0; i<11; i++) setTransferFunction(inputNeurons[i],ANN::identityFunction());
		// create 1 output neurons
//		for (int i=0; i<1; i++) outputNeurons.push_back(addNeuron());
//		setTransferFunction(outputNeurons[0],ANN::identityFunction());

		// create the sub networks
		aslt0 = new ASLT0();

		// add the sub networks to this network
		addSubnet(aslt0);

		// connect synapses originating at input neurons
//		w(aslt0->n(0),      inputNeurons[0], 1.0);
//		w(aslt0->n(1),      inputNeurons[1], 1.0);
//		w(aslt0->n(2),      inputNeurons[2], 1.0);      
//		w(aslt0->n(3),      inputNeurons[3], 1.0);  
//		w(aslt0->n(4),      inputNeurons[4], 1.0);  
//		w(aslt0->n(5),      inputNeurons[5], 1.0);  
//		w(aslt0->n(6),      inputNeurons[6], 1.0);  
//		w(aslt0->n(7),      inputNeurons[7], 1.0);  
//		w(aslt0->n(8),      inputNeurons[8], 1.0);  
//		w(aslt0->n(9),      inputNeurons[9], 1.0);  
//		w(aslt0->n(10),     inputNeurons[10], 1.0);  

		// create synapses to output neurons
//		w(outputNeurons[0], aslt0->getNeuronOutput(),  1.0);	

	}

ASLT0::ASLT0()
{  
	setDefaultTransferFunction(ANN::tanhFunction()); 
    setNeuronNumber(17);
w(11, 0, -0.777334);
w(11, 1, -0.393426);
w(11, 2, -0.0981024);
w(11, 3, -0.384039);
w(11, 4, -0.137481);
w(11, 5, -0.0870342);
w(11, 6, 0.478205);
w(11, 7, -0.613451);
w(11, 8, -0.865463);
w(11, 9, 0.757576);
w(11, 10, 0.960966);
w(12, 0, 0.356891);
w(12, 1, 1.05982);
w(12, 2, 2.72346);
w(12, 3, 0.53293);
w(12, 4, -0.810885);
w(12, 5, -0.0827674);
w(12, 6, 0.710014);
w(12, 7, 0.457012);
w(12, 8, 0.581924);
w(12, 9, -0.317574);
w(12, 10, 0.956759);
w(13, 0, 0.342429);
w(13, 1, -0.629427);
w(13, 2, 0.311025);
w(13, 3, -0.372067);
w(13, 4, 0.0841113);
w(13, 5, 0.424983);
w(13, 6, -0.309503);
w(13, 7, 0.944965);
w(13, 8, 0.192123);
w(13, 9, -0.877546);
w(13, 10, 0.428025);
w(14, 0, -0.619986);
w(14, 1, 0.57802);
w(14, 2, -0.858955);
w(14, 3, 0.75911);
w(14, 4, -0.500869);
w(14, 5, 0.408983);
w(14, 6, 0.231457);
w(14, 7, -0.39552);
w(14, 8, -0.409299);
w(14, 9, 0.383154);
w(14, 10, 0.0721382);
w(15, 0, -0.968284);
w(15, 1, 0.314798);
w(15, 2, 0.304677);
w(15, 3, 0.729811);
w(15, 4, 0.554227);
w(15, 5, 0.337312);
w(15, 6, 0.492974);
w(15, 7, -0.543603);
w(15, 8, -0.495253);
w(15, 9, -0.81797);
w(15, 10, -0.309615);
w(16, 11, -0.0213993);
w(16, 12, -1.56472);
w(16, 13, -0.0210647);
w(16, 14, 0.0261986);
w(16, 15, -0.0265257);

b(0, 0.035448);
b(1, 0.113356);
b(2, 0.528928);
b(3, 0.197312);
b(4, -0.441733);
b(5, -0.337915);
b(6, -0.105511);
b(7, -0.411685);
b(8, -0.278782);
b(9, 0.197155);
b(10, 0.0852481);
b(11, 0.161245);
b(12, 0.168028);
b(13, -0.0922643);
b(14, 0.132881);
b(15, -0.0334853);
b(16, 1.52251);


}

Neuron* ASLT0::getNeuronOutput()
{
    return getNeuron(16);
}
