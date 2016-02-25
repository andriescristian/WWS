#include <DTWI.h>
#include <Wire.h>

#include "AD5933.h"


#include "RFInterface.h"
#include "RFWrapper.h"
#include "IA.h"

IA ia;
void setup()
{
     Serial.begin(9600);
     ia.readImpedanceSamples();
}

void loop()
{

}
