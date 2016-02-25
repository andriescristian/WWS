/******************************************************************************/
/***************************** Include Files **********************************/
/******************************************************************************/
#include <AD5933.h>
#include <Wire.h>

//SEL=0 Rfb=100 kohm, Sel=1 Rfb=20 ohm
/******************************************************************************/
/************************ Constants Definitions *******************************/
/******************************************************************************/

/******************************************************************************/
/************************ Variables Definitions *******************************/
/******************************************************************************/
AD5933 myAD5933;

/*! Variables holding information about the device */
short  temperature = 0;         /*!< Last temperature read from the device */

double startFreq = 5000;        /*!< Start frequency sweep */
double incFreq = 100;            /*!< Increment frequency */
short  incNum = 10;             /*!< Number of increments */
short  cyclesNum = 25;
//unsigned char range=AD5933_RANGE_2000mVpp; //test Rfb=100 kohm
//unsigned char range=AD5933_RANGE_1000mVpp;
//unsigned char range=AD5933_RANGE_400mVpp;
unsigned char range=AD5933_RANGE_200mVpp;   //test Rfb=20 ohm

unsigned char pga=AD5933_PGA_1;
unsigned char clock=AD5933_CONTROL_INT_SYSCLK;

long   calibImped = 98200;            //Rcalib for Rfb=100 kohm
//long   calibImped = 22;            //Rcalib for Rfb=20 ohm

double gainFactor = 0;          /*!< Stores the value of the gain factor */
double impedance = 0;           /*!< Measured impedance */
double currentFreq = startFreq; /*!< Signal frequency used during a measurement */

//macros definition
#define  Test_Param  range, pga, clock
#define  Sweep_Param  startFreq, incFreq, incNum, cyclesNum  

//! Temporary variables 
char   tempString [10]  = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0};
unsigned short temp=0;
unsigned char status = 0;
double  doubleTemp=0;
double  MultiplyGain=0;

bool startReading = false;
/***************************************************************************//**
 * @brief Reads one command from UART.
 *
 * @param command - Read command.
 *
 * @return None.
*******************************************************************************/

/***************************************************************************//** 
 * @param receivedCommand - Received command.
 * @param expectedCommand - Expected command.
 * @param commandParameter - Command parameter.
 *
 * @return commandType - Type of the command.
 *                       Example: 0 - Commands don't match.
 *                                1 - Write command.
 *                                2 - Read command.
*******************************************************************************/



/***************************************************************************//**
 * @brief Setup function.
 *
 * @return none.
*******************************************************************************/    
void setup()
{
  Serial.begin(9600);
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
  Serial.print("AD5933_REG_CONTROL_HB_LB=");
  Serial.println(temp,HEX);
 
  gainFactor = myAD5933.CalculateGainFactor(calibImped,
                                            AD5933_FUNCTION_REPEAT_FREQ);
  MultiplyGain=1000000*gainFactor;                                          
  doubleTemp=1/(gainFactor*calibImped); 
  sprintf(tempString, "%.3f", (double)doubleTemp);
  Serial.print("Mcal=");
  Serial.println(tempString);
  
  sprintf(tempString, "%f", (double)MultiplyGain);
  Serial.print("Multiply Gain (^6)=");
  Serial.println(tempString);
  attachCoreTimerService(timerCallback);
}

/***************************************************************************//**
 * @brief Loop function.
 *
 * @return none.
*******************************************************************************/
void loop()
{
 double currentFreq = startFreq;   //!< Signal frequency used during a measurement 
  
  /*! Configure the sweep parameters */
  myAD5933.ConfigSweep(Sweep_Param);
  /*! Start the sweep operation */
  myAD5933.StartSweep(Test_Param);
              
  while( startReading )
  {
    
    impedance = myAD5933.CalculateImpedance(gainFactor,
                                                        AD5933_FUNCTION_INC_FREQ);
    //! Send the requested value to user 
    sprintf(tempString, "%.3f", (double)impedance / 1000);
    Serial.print("impedance=");
    Serial.print(tempString);
    Serial.print(",");
                
    Serial.print("currentFreq=");
    Serial.print(currentFreq);
    Serial.print(",");
                
    doubleTemp=1/(gainFactor*impedance); 
    sprintf(tempString, "%.3f", (double)doubleTemp);
    Serial.print("Mu=");
    Serial.println(tempString);
                
    //! Update the currentFrequency 
    currentFreq = currentFreq + incFreq;
    status = myAD5933.GetRegisterValue(AD5933_REG_STATUS,1);
    if( !(status & AD5933_STAT_SWEEP_DONE) == 0 )
    {
      startReading = false;
    }
    
  }
     
}

uint32_t timerCallback( uint32_t currentTime )
{
  startReading = true;
  
  return (currentTime + (CORE_TICK_RATE * 5000)); 
}

