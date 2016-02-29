
#ifndef IA_H
#define IA_H

#define MAX_IMPEDANCE_SAMPLES 30

#define START_FREQ 5000

class IA
{
public:
	IA();
	~IA();
	
	double calibrateIA(unsigned long calibImpedance);
	
	//double getEEPROMGainFactor(int address);
	
	void configIA();
	
	void readImpedanceSamples();
	
private:

	double m_gainFactor;
	
	float m_impedanceArray[MAX_IMPEDANCE_SAMPLES];
		short  temperature ;        

double startFreq ;        
double incFreq ;           
short  incNum ;            
short  cyclesNum ;

unsigned char range;   

unsigned char pga;
unsigned char clock;

long   calibImped ;            


double gainFactor ;         
double impedance ;           
double currentFreq ; 

//macros definition
#define  Test_Param  range, pga, clock
#define  Sweep_Param  startFreq, incFreq, incNum, cyclesNum  

//! Temporary variables 
char  tempString [10]  ;
unsigned short temp;
unsigned char status ;
double  doubleTemp;
double  MultiplyGain;

bool startReading ;

int i;

};

#endif