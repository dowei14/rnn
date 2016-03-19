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
	w(11, 0, 0.469757);
	w(11, 1, -0.227267);
	w(11, 2, 0.190844);
	w(11, 3, 0.704808);
	w(11, 4, 0.370661);
	w(11, 5, -0.174339);
	w(11, 6, 0.599892);
	w(11, 7, 0.0101737);
	w(11, 8, -0.388615);
	w(11, 9, 0.353103);
	w(11, 10, 0.404393);
	w(12, 0, 0.680304);
	w(12, 1, 0.201307);
	w(12, 2, 0.75257);
	w(12, 3, 0.453325);
	w(12, 4, -0.0116046);
	w(12, 5, -0.161296);
	w(12, 6, 0.0478768);
	w(12, 7, -0.492696);
	w(12, 8, -0.792558);
	w(12, 9, -0.809539);
	w(12, 10, 0.848551);
	w(13, 0, -0.0201317);
	w(13, 1, -0.157367);
	w(13, 2, -0.0743779);
	w(13, 3, -0.609628);
	w(13, 4, -0.603022);
	w(13, 5, 0.743066);
	w(13, 6, 0.146262);
	w(13, 7, -0.597269);
	w(13, 8, 0.291497);
	w(13, 9, 0.716206);
	w(13, 10, -0.87106);
	w(14, 0, 0.268904);
	w(14, 1, 0.48648);
	w(14, 2, 2.32779);
	w(14, 3, 0.343021);
	w(14, 4, 0.298197);
	w(14, 5, 0.231184);
	w(14, 6, 0.320903);
	w(14, 7, -0.551835);
	w(14, 8, 0.219566);
	w(14, 9, 0.24519);
	w(14, 10, 0.186502);
	w(15, 0, 0.733329);
	w(15, 1, 0.996301);
	w(15, 2, 0.701075);
	w(15, 3, 0.711787);
	w(15, 4, 0.436682);
	w(15, 5, 0.571963);
	w(15, 6, -0.273559);
	w(15, 7, -0.232188);
	w(15, 8, -0.195221);
	w(15, 9, -0.190355);
	w(15, 10, 0.384961);
	w(16, 11, 0.176979);
	w(16, 12, -0.0894817);
	w(16, 13, -0.0707152);
	w(16, 14, -1.69464);
	w(16, 15, 0.178324);

	b(0, 0.0644174);
	b(1, 0.00329133);
	b(2, 0.500495);
	b(3, 0.233574);
	b(4, 0.0888461);
	b(5, -0.206874);
	b(6, 0.0156421);
	b(7, 0.085458);
	b(8, -0.161838);
	b(9, 0.20251);
	b(10, 0.166181);
	b(11, 0.444889);
	b(12, -0.0098021);
	b(13, -0.204575);
	b(14, 0.091816);
	b(15, -0.116123);
	b(16, 1.29108);
}

Neuron* ASLT0::getNeuronOutput()
{
    return getNeuron(16);
}
