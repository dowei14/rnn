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
		aslt1 = new ASLT1();
		aslt2 = new ASLT2();
		aslt3 = new ASLT3();
		aslt4 = new ASLT4();
		aslt5 = new ASLT5();
		aslt6 = new ASLT6();
		
		// add the sub networks to this network
		addSubnet(aslt0);
		addSubnet(aslt1);
		addSubnet(aslt2);
		addSubnet(aslt3);
		addSubnet(aslt4);
		addSubnet(aslt5);
		addSubnet(aslt6);
		

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
w(11, 0, 0.992504);
w(11, 1, 0.188043);
w(11, 2, 0.573636);
w(11, 3, -1.01478);
w(11, 4, -0.780922);
w(11, 5, -0.464284);
w(11, 6, 0.673451);
w(11, 7, 1.14929);
w(11, 8, 0.1127);
w(11, 9, 0.173603);
w(11, 10, 0.484);
w(12, 0, 0.249308);
w(12, 1, 0.513563);
w(12, 2, -0.228124);
w(12, 3, 0.590807);
w(12, 4, 0.0533537);
w(12, 5, -0.0543489);
w(12, 6, -0.250748);
w(12, 7, 0.561694);
w(12, 8, 1.24367);
w(12, 9, 0.194306);
w(12, 10, -0.671585);
w(13, 0, -0.890489);
w(13, 1, -0.733094);
w(13, 2, 0.582164);
w(13, 3, -0.110192);
w(13, 4, 0.352778);
w(13, 5, -0.747887);
w(13, 6, 0.489431);
w(13, 7, -0.623584);
w(13, 8, -0.172892);
w(13, 9, -1.05883);
w(13, 10, -0.402804);
w(14, 0, -5.32352);
w(14, 1, -5.38947);
w(14, 2, -1.45282);
w(14, 3, 0.13631);
w(14, 4, -0.0807866);
w(14, 5, 0.291246);
w(14, 6, -0.141268);
w(14, 7, -1.08264);
w(14, 8, -3.2133);
w(14, 9, -2.40963);
w(14, 10, -0.47941);
w(15, 0, 0.350228);
w(15, 1, 1.10962);
w(15, 2, -0.533684);
w(15, 3, 0.436781);
w(15, 4, 0.506744);
w(15, 5, -0.20662);
w(15, 6, 0.659796);
w(15, 7, 1.07875);
w(15, 8, -0.665782);
w(15, 9, 0.0156026);
w(15, 10, 0.606496);
w(16, 11, 0.765345);
w(16, 12, -0.00677159);
w(16, 13, -0.443683);
w(16, 14, 3.96641);
w(16, 15, 1.00273);

b(0, 0.534938);
b(1, 0.549533);
b(2, 0.264516);
b(3, -0.510079);
b(4, -0.392066);
b(5, -0.396711);
b(6, -0.0937216);
b(7, 0.727467);
b(8, -0.608303);
b(9, -0.514188);
b(10, 0.152521);
b(11, 0.477181);
b(12, -0.154244);
b(13, -0.651941);
b(14, -0.642604);
b(15, 0.565103);
b(16, 1.78044);



}

Neuron* ASLT0::getNeuronOutput()
{
    return getNeuron(16);
}

ASLT1::ASLT1()
{  
	setDefaultTransferFunction(ANN::tanhFunction()); 
    setNeuronNumber(17);
w(11, 0, 2.22506);
w(11, 1, 2.64799);
w(11, 2, -2.47455);
w(11, 3, 0.435478);
w(11, 4, 0.314575);
w(11, 5, -0.0656062);
w(11, 6, 0.925553);
w(11, 7, 0.971903);
w(11, 8, 1.27168);
w(11, 9, 0.987468);
w(11, 10, -1.17308);
w(12, 0, 0.63493);
w(12, 1, 1.01205);
w(12, 2, -0.723462);
w(12, 3, 0.37189);
w(12, 4, 0.928506);
w(12, 5, 1.11397);
w(12, 6, 0.484866);
w(12, 7, 0.160982);
w(12, 8, 0.628164);
w(12, 9, -0.293049);
w(12, 10, -1.05098);
w(13, 0, -0.29081);
w(13, 1, 0.0726666);
w(13, 2, -0.445598);
w(13, 3, 0.367634);
w(13, 4, 0.119923);
w(13, 5, 0.301671);
w(13, 6, 0.860643);
w(13, 7, 0.966448);
w(13, 8, 0.931801);
w(13, 9, 0.407266);
w(13, 10, 0.0649873);
w(14, 0, 0.47356);
w(14, 1, 0.473764);
w(14, 2, 0.165295);
w(14, 3, -1.00524);
w(14, 4, 0.585855);
w(14, 5, 0.792799);
w(14, 6, 0.313118);
w(14, 7, -0.0957267);
w(14, 8, -0.508412);
w(14, 9, -0.173904);
w(14, 10, -0.533149);
w(15, 0, 0.23495);
w(15, 1, 0.274872);
w(15, 2, 1.23004);
w(15, 3, 0.103877);
w(15, 4, -0.420873);
w(15, 5, -0.103537);
w(15, 6, 0.525633);
w(15, 7, -0.899323);
w(15, 8, -1.06692);
w(15, 9, -0.854554);
w(15, 10, 0.915599);
w(16, 11, -2.62825);
w(16, 12, -0.485029);
w(16, 13, 1.19782);
w(16, 14, 0.0566992);
w(16, 15, -1.20702);

b(0, -0.702168);
b(1, -0.564765);
b(2, 0.218794);
b(3, -0.451974);
b(4, 0.373735);
b(5, 0.228769);
b(6, 0.433181);
b(7, 0.690654);
b(8, 0.355988);
b(9, 0.789068);
b(10, -0.702834);
b(11, 0.858324);
b(12, 0.229907);
b(13, 0.696446);
b(14, 0.201243);
b(15, -0.252674);
b(16, 0.658505);
}

Neuron* ASLT1::getNeuronOutput()
{
    return getNeuron(16);
}

ASLT2::ASLT2()
{  
	setDefaultTransferFunction(ANN::tanhFunction()); 
    setNeuronNumber(17);
w(11, 0, 0.052071);
w(11, 1, -0.626385);
w(11, 2, 0.730619);
w(11, 3, 0.479323);
w(11, 4, 0.934948);
w(11, 5, -0.533048);
w(11, 6, 0.0406279);
w(11, 7, -0.71813);
w(11, 8, -0.687061);
w(11, 9, -0.701611);
w(11, 10, 0.993803);
w(12, 0, -0.147297);
w(12, 1, 0.43688);
w(12, 2, 0.193001);
w(12, 3, -0.926046);
w(12, 4, -0.243725);
w(12, 5, -0.507037);
w(12, 6, -0.572837);
w(12, 7, -0.686652);
w(12, 8, 0.11623);
w(12, 9, 0.44959);
w(12, 10, -0.958992);
w(13, 0, 0.0126896);
w(13, 1, -0.559331);
w(13, 2, -0.788626);
w(13, 3, 0.92686);
w(13, 4, -0.054147);
w(13, 5, 0.0908646);
w(13, 6, -0.871968);
w(13, 7, 0.320963);
w(13, 8, -0.173861);
w(13, 9, -0.696874);
w(13, 10, 0.664529);
w(14, 0, -0.157855);
w(14, 1, 0.450156);
w(14, 2, 1.46999);
w(14, 3, -0.381727);
w(14, 4, -0.322484);
w(14, 5, -1.01239);
w(14, 6, 0.147093);
w(14, 7, 0.215951);
w(14, 8, 0.0852362);
w(14, 9, 2.7629);
w(14, 10, 0.834584);
w(15, 0, -0.182822);
w(15, 1, 0.231383);
w(15, 2, -0.306082);
w(15, 3, -0.165614);
w(15, 4, -0.417407);
w(15, 5, -1.04238);
w(15, 6, 0.158011);
w(15, 7, -0.101082);
w(15, 8, 0.975781);
w(15, 9, 0.791539);
w(15, 10, -0.499307);
w(16, 11, -0.0752399);
w(16, 12, -0.471373);
w(16, 13, 0.27069);
w(16, 14, 2.39866);
w(16, 15, -1.18273);

b(0, 0.278639);
b(1, -0.309798);
b(2, 0.3616);
b(3, 0.455704);
b(4, 0.0556001);
b(5, 0.776464);
b(6, -0.219679);
b(7, 0.506508);
b(8, -0.791969);
b(9, -0.591588);
b(10, -0.0933182);
b(11, -0.141871);
b(12, -0.158115);
b(13, 0.122594);
b(14, -0.553825);
b(15, -0.482139);
b(16, 0.526657);

}

Neuron* ASLT2::getNeuronOutput()
{
    return getNeuron(16);
}

ASLT3::ASLT3()
{  
	setDefaultTransferFunction(ANN::tanhFunction()); 
    setNeuronNumber(17);
w(11, 0, -0.299348);
w(11, 1, 0.315398);
w(11, 2, 0.281477);
w(11, 3, -0.958934);
w(11, 4, -0.886277);
w(11, 5, 0.413232);
w(11, 6, -0.622776);
w(11, 7, -0.18198);
w(11, 8, -0.951828);
w(11, 9, 0.0979363);
w(11, 10, 0.735722);
w(12, 0, -1.35505);
w(12, 1, -0.314004);
w(12, 2, -0.516894);
w(12, 3, -0.447163);
w(12, 4, -0.722555);
w(12, 5, -0.0758501);
w(12, 6, -0.215632);
w(12, 7, -0.274574);
w(12, 8, 0.758118);
w(12, 9, -1.18321);
w(12, 10, -0.117635);
w(13, 0, 0.402248);
w(13, 1, -0.654776);
w(13, 2, -0.0165534);
w(13, 3, -0.282206);
w(13, 4, -0.708249);
w(13, 5, 0.00863925);
w(13, 6, -0.0342227);
w(13, 7, 0.314657);
w(13, 8, 0.0470943);
w(13, 9, 0.00220896);
w(13, 10, 0.249101);
w(14, 0, -3.46153);
w(14, 1, -2.48073);
w(14, 2, -0.249169);
w(14, 3, -0.961793);
w(14, 4, -0.0045397);
w(14, 5, 0.0296194);
w(14, 6, -0.304553);
w(14, 7, -0.720468);
w(14, 8, 1.25022);
w(14, 9, 1.26341);
w(14, 10, -0.934651);
w(15, 0, 0.354872);
w(15, 1, 0.0148961);
w(15, 2, -0.290138);
w(15, 3, -0.438021);
w(15, 4, -0.979744);
w(15, 5, 0.884945);
w(15, 6, 0.229423);
w(15, 7, 0.0440447);
w(15, 8, 0.92501);
w(15, 9, -0.75858);
w(15, 10, -0.00469512);
w(16, 11, 0.0134902);
w(16, 12, -1.65106);
w(16, 13, -0.0244478);
w(16, 14, 2.65975);
w(16, 15, 0.0371339);

b(0, 0.448469);
b(1, 0.432237);
b(2, 0.422477);
b(3, 0.391314);
b(4, 0.259899);
b(5, 0.0413498);
b(6, 0.0632432);
b(7, 0.122061);
b(8, -0.853268);
b(9, -0.0268469);
b(10, 0.331599);
b(11, 0.0676709);
b(12, -0.627046);
b(13, -0.042177);
b(14, -0.240128);
b(15, 0.0223873);
b(16, 1.01545);

}

Neuron* ASLT3::getNeuronOutput()
{
    return getNeuron(16);
}

ASLT4::ASLT4()
{  
	setDefaultTransferFunction(ANN::tanhFunction()); 
    setNeuronNumber(17);
w(11, 0, -0.10879);
w(11, 1, 1.12255);
w(11, 2, -0.688828);
w(11, 3, 0.158159);
w(11, 4, -0.116714);
w(11, 5, 0.5683);
w(11, 6, -0.662263);
w(11, 7, -0.546956);
w(11, 8, -0.144569);
w(11, 9, -0.0972805);
w(11, 10, -0.795107);
w(12, 0, -0.574707);
w(12, 1, 0.213159);
w(12, 2, 0.458464);
w(12, 3, 0.591524);
w(12, 4, 1.23937);
w(12, 5, 0.781353);
w(12, 6, -0.187154);
w(12, 7, 0.441459);
w(12, 8, 0.206233);
w(12, 9, 1.10724);
w(12, 10, 0.68596);
w(13, 0, 0.930758);
w(13, 1, -0.806446);
w(13, 2, 0.495629);
w(13, 3, -0.223519);
w(13, 4, 0.825695);
w(13, 5, -0.114642);
w(13, 6, 0.115357);
w(13, 7, 0.291921);
w(13, 8, 0.232609);
w(13, 9, -0.808563);
w(13, 10, -0.463399);
w(14, 0, 1.99665);
w(14, 1, 1.26348);
w(14, 2, 0.72449);
w(14, 3, -0.66277);
w(14, 4, 1.74025);
w(14, 5, 1.37123);
w(14, 6, -1.07845);
w(14, 7, -0.555085);
w(14, 8, 0.980312);
w(14, 9, 1.78967);
w(14, 10, -0.327326);
w(15, 0, -2.96393);
w(15, 1, -2.79614);
w(15, 2, -0.422025);
w(15, 3, 0.434718);
w(15, 4, 1.18277);
w(15, 5, 1.10494);
w(15, 6, -0.21318);
w(15, 7, 0.650361);
w(15, 8, -1.38658);
w(15, 9, -2.22439);
w(15, 10, 0.568188);
w(16, 11, 0.309806);
w(16, 12, 0.454768);
w(16, 13, 0.245472);
w(16, 14, -1.77393);
w(16, 15, -2.57896);

b(0, -0.197557);
b(1, -0.156753);
b(2, -0.60465);
b(3, 0.140269);
b(4, -0.506651);
b(5, -0.250481);
b(6, 0.000707364);
b(7, 0.738362);
b(8, -0.854492);
b(9, -0.721137);
b(10, -0.157799);
b(11, 0.0794529);
b(12, 0.234606);
b(13, 0.0293);
b(14, -0.531508);
b(15, 0.599025);
b(16, 0.630636);

}

Neuron* ASLT4::getNeuronOutput()
{
    return getNeuron(16);
}

ASLT5::ASLT5()
{  
	setDefaultTransferFunction(ANN::tanhFunction()); 
    setNeuronNumber(17);
w(11, 0, 0.423226);
w(11, 1, -0.616929);
w(11, 2, -1.13129);
w(11, 3, 0.254655);
w(11, 4, -0.778949);
w(11, 5, -1.03553);
w(11, 6, 0.33536);
w(11, 7, 0.118644);
w(11, 8, 0.603825);
w(11, 9, 0.679068);
w(11, 10, -0.0328387);
w(12, 0, -0.255351);
w(12, 1, -0.600192);
w(12, 2, -0.587018);
w(12, 3, 0.438913);
w(12, 4, -1.33304);
w(12, 5, 1.25588);
w(12, 6, 0.0108289);
w(12, 7, 0.326152);
w(12, 8, 0.270645);
w(12, 9, 0.806235);
w(12, 10, -0.0256336);
w(13, 0, 0.540696);
w(13, 1, 0.717525);
w(13, 2, 0.117028);
w(13, 3, 0.641222);
w(13, 4, -1.19414);
w(13, 5, 1.47543);
w(13, 6, -0.0637415);
w(13, 7, 0.227223);
w(13, 8, 0.0276857);
w(13, 9, -1.21946);
w(13, 10, 0.888875);
w(14, 0, -0.923579);
w(14, 1, -1.05151);
w(14, 2, -0.333096);
w(14, 3, 0.500216);
w(14, 4, 1.09481);
w(14, 5, -1.16385);
w(14, 6, 0.0451827);
w(14, 7, -0.182252);
w(14, 8, 0.410363);
w(14, 9, 1.03237);
w(14, 10, -1.05393);
w(15, 0, 0.993389);
w(15, 1, 1.03989);
w(15, 2, 0.814402);
w(15, 3, 0.139983);
w(15, 4, -1.29364);
w(15, 5, -1.0378);
w(15, 6, 0.632396);
w(15, 7, 0.460314);
w(15, 8, 0.0842404);
w(15, 9, -1.27407);
w(15, 10, 0.013999);
w(16, 11, -1.02806);
w(16, 12, 0.950383);
w(16, 13, -1.15175);
w(16, 14, 0.933647);
w(16, 15, 1.97738);

b(0, -0.409286);
b(1, -0.161398);
b(2, 0.507198);
b(3, -0.123152);
b(4, -0.378089);
b(5, -0.414859);
b(6, 0.0144945);
b(7, 0.211315);
b(8, 0.445103);
b(9, -0.420747);
b(10, -0.144532);
b(11, -0.170568);
b(12, -0.229268);
b(13, -0.203604);
b(14, -0.0914006);
b(15, 0.199825);
b(16, -0.122098);

}

Neuron* ASLT5::getNeuronOutput()
{
    return getNeuron(16);
}

ASLT6::ASLT6()
{  
	setDefaultTransferFunction(ANN::tanhFunction()); 
    setNeuronNumber(17);
w(11, 0, -0.935468);
w(11, 1, -0.476027);
w(11, 2, -0.259562);
w(11, 3, 0.541755);
w(11, 4, 1.29759);
w(11, 5, 1.15357);
w(11, 6, 0.304496);
w(11, 7, -0.00479943);
w(11, 8, -0.220181);
w(11, 9, -0.574904);
w(11, 10, -0.181614);
w(12, 0, -0.847907);
w(12, 1, -0.212844);
w(12, 2, 0.918671);
w(12, 3, 0.475447);
w(12, 4, 0.96026);
w(12, 5, 0.504188);
w(12, 6, 0.432896);
w(12, 7, -0.28009);
w(12, 8, -0.601771);
w(12, 9, -0.666841);
w(12, 10, -0.103229);
w(13, 0, 1.50201);
w(13, 1, 0.378801);
w(13, 2, -0.35765);
w(13, 3, -0.324256);
w(13, 4, 1.34402);
w(13, 5, 1.73661);
w(13, 6, 0.526796);
w(13, 7, -0.275971);
w(13, 8, -0.0958237);
w(13, 9, 1.27547);
w(13, 10, -0.0527399);
w(14, 0, 0.348826);
w(14, 1, -0.480809);
w(14, 2, -0.714568);
w(14, 3, -0.447898);
w(14, 4, 0.148908);
w(14, 5, 0.00565636);
w(14, 6, 0.0442623);
w(14, 7, 0.557631);
w(14, 8, 0.663454);
w(14, 9, 0.321487);
w(14, 10, -0.838834);
w(15, 0, 0.416315);
w(15, 1, 0.87936);
w(15, 2, 0.369992);
w(15, 3, -0.157515);
w(15, 4, 1.14542);
w(15, 5, 0.843719);
w(15, 6, -0.332833);
w(15, 7, 0.234852);
w(15, 8, 0.796723);
w(15, 9, 0.968237);
w(15, 10, -0.395284);
w(16, 11, 1.278);
w(16, 12, -0.747323);
w(16, 13, -1.75504);
w(16, 14, -0.395689);
w(16, 15, -0.846206);

b(0, -0.274738);
b(1, -0.139746);
b(2, -0.214833);
b(3, 0.036525);
b(4, -0.192971);
b(5, -0.131154);
b(6, 0.0688172);
b(7, -0.126323);
b(8, 0.545936);
b(9, 0.0244702);
b(10, -0.106073);
b(11, 0.425258);
b(12, -0.487898);
b(13, 0.293913);
b(14, -0.178714);
b(15, 0.357574);
b(16, 1.14293);

}

Neuron* ASLT6::getNeuronOutput()
{
    return getNeuron(16);
}
