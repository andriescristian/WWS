
#include "IA.h"
#include "RFInterface.h"
#include "AD5933.h"
#include "Wire.h"
#include "WProgram.h"
#include "EEPROMAnything.h"
AD5933 myAD5933;

IA::IA()
{
	m_gainFactor = 0;
	
	m_impedanceArray = { 0 };
	
	
	
	temperature = 0;
startFreq = 5000;        
 incFreq = 100;           
  incNum = 10;            
 cyclesNum = 25;

 range=AD5933_RANGE_200mVpp;   

 pga=AD5933_PGA_1;
 clock=AD5933_CONTROL_INT_SYSCLK;

   calibImped = 98200;            


 gainFactor = 0;         
 impedance = 0;           
 currentFreq = startFreq; 

//macros definition
#define  Test_Param  range, pga, clock
#define  Sweep_Param  startFreq, incFreq, incNum, cyclesNum  

//! Temporary variables 
  tempString [10]  = {0};
 temp=0;
 status = 0;
 doubleTemp=0;
  MultiplyGain=0;

 startReading = false;
}

IA::~IA()
{
	
}

double IA::calibrateIA(unsigned long calibImpedance)
{
 if(myAD5933.Init())
  {
      Serial.println("AD5933 OK");
  }
  else
  {
      Serial.println("AD5933 Error");
  }
   myAD5933.ConfigSweep(Sweep_Param);
  /*! Start the sweep operation */

  myAD5933.StartSweep(Test_Param);
  temp=myAD5933.GetRegisterValue(AD5933_REG_CONTROL_HB,2);
 // Serial.print("AD5933_REG_CONTROL_HB_LB=");
 // Serial.println(temp,HEX);
 
  gainFactor = myAD5933.CalculateGainFactor(calibImped,
                                            AD5933_FUNCTION_REPEAT_FREQ);
  MultiplyGain=1000000*gainFactor;                                          
  doubleTemp=1/(gainFactor*calibImped); 
  sprintf(tempString, "%.3f", (double)doubleTemp);
 // Serial.print("Mcal=");
 // Serial.println(tempString);
  
  sprintf(tempString, "%f", (double)MultiplyGain);
 // Serial.print("Multiply Gain (^6)=");
 // Serial.println(tempString);
  //attachCoreTimerService(timerCallback);
  
EEPROM_writeAnything( 0x00, gainFactor );
  
}
 


 
	 
 /*double IA::getEEPROMGainFactor(int address)
{
	
 EEPROM_readAnything(0x00, gainFactor)
}*/

void IA::configIA()
{
	 

     double currentFreq = startFreq;   
  

  myAD5933.ConfigSweep(Sweep_Param);
  /*! Start the sweep operation */
  myAD5933.StartSweep(Test_Param);

}

void IA::readImpedanceSamples()
{
	 EEPROM_readAnything(0x00, gainFactor);
		while( startReading )
  {
   
    impedance = myAD5933.CalculateImpedance(gainFactor,
                                                        AD5933_FUNCTION_INC_FREQ);
    //! Send the requested value to user 
    //sprintf(tempString, "%.3f", (double)impedance / 1000);
   // Serial.print("impedance=");
   // Serial.print(tempString);
   // Serial.print(",");
                
    //Serial.print("currentFreq=");
    
                
    doubleTemp=1/(gainFactor*impedance); 
  //  sprintf(tempString, "%.3f", (double)doubleTemp);
  
                
    //! Update the currentFrequency 
    currentFreq = currentFreq + incFreq;
    status = myAD5933.GetRegisterValue(AD5933_REG_STATUS,1);
    if( !(status & AD5933_STAT_SWEEP_DONE) == 0 )
    {
      startReading = false;
    }
  }
  



/*uint32_t timerCallback( uint32_t currentTime )
{
  startReading = true;
  
  return (currentTime + (CORE_TICK_RATE * 5000)); 
}*/
//Array impedance

float m_impedanceArray [30]  ;
for( i=0; i<=incNum ; i++)
	m_impedanceArray[i]=impedance;
	// read sweep samples
	
	sendImpedance( m_impedanceArray, 30 );
}