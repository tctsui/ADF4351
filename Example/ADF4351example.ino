/*!
   ADF4351 example program
*/

#include <Arduino.h>
#include "ADF4351.h"

#define SWVERSION "1.0"

#define PIN_SS 10  ///< SPI slave select pin, default value

int triggerIn = 0 ;
int ReverseScanDir=0;
unsigned long StartFrequency = 1500000000UL;
unsigned long StopFrequency = 1500000000UL;//StopFrequency needs to be larger than the StartFrequency,since unsigned long is used.
unsigned long FrequencyScanStep = 10;
unsigned long FrequencyScanCounter = 0 ;
volatile unsigned long FrequncyRequested = StartFrequency;
ADF4351  vfo(PIN_SS, SPI_MODE0, 300000UL , MSBFIRST) ;


void setup()
{
  Serial.begin(115200) ;
  Serial.print("Hello adf4351 demo v") ;
  Serial.println(SWVERSION) ;
  Wire.begin() ;
  
  /*!
     setup the chip (for a 10 mhz ref freq)
     most of these are defaults
  */
  vfo.pwrlevel = 0 ; ///< sets to -4 dBm output
  vfo.RD2refdouble = 0 ; ///< ref doubler off
  vfo.RD1Rdiv2 = 0 ;   ///< ref divider off
  vfo.ClkDiv = 150 ;
  vfo.BandSelClock = 200 ;
  vfo.RCounter = 1 ;  ///< R counter to 1 (no division)
  vfo.ChanStep = 10000 ;  ///< set to 10 kHz steps


  /*!
     sets the reference frequency to 25 Mhz
  */
  delay(500);
  if ( vfo.setrf(25000000UL) ==  0 )
    Serial.println("ref freq set to 25 Mhz") ;
    else
      Serial.println("ref freq set error") ;
      /*!
         initialize the chip
      */
      
      vfo.init() ;

  /*!
     enable frequency output
  */
  vfo.enable() ;


vfo.setf(StartFrequency);
/*
Because FastScanSetF only update one of the register when the frequency is changed, so FastscanabilityCheck needs to be run to set the remaining 5 registers.
*/

//vfo.FastscanabilityCheck(StartFrequency,StopFrequency,FrequencyScanStep);


}



void loop()
{
//vfo.setf(StartFrequency);
    Serial.println("Freq requested");
    Serial.println(StartFrequency);
    Serial.println("RD1Rdiv2");
    Serial.println(vfo.RD1Rdiv2);
    Serial.println("BandSelClock");
    Serial.println(vfo.BandSelClock);
    Serial.println(" ClkDiv");
    Serial.println(vfo.ClkDiv);
    Serial.println("PFD");
    Serial.println(vfo.PFDFreq);
    Serial.println("RCounter");
    Serial.println(vfo.RCounter);
    Serial.println("Output divider");
    Serial.println(vfo.outdiv);
    Serial.println("Frac");
    Serial.println(vfo.Frac) ;
    Serial.println("Mod");
    Serial.println(vfo.Mod) ;
    Serial.println("Int");
    Serial.println(vfo.N_Int);
    Serial.println("cfrequency");
    Serial.println(vfo.cfreq);
    Serial.println("Output frequency");
    Serial.println(vfo.PFDFreq*(vfo.N_Int+((float)vfo.Frac)/((float)vfo.Mod))/vfo.outdiv);
delay(10000);
//
//if(FrequencyScanCounter <= FrequencyScanStep){
// FrequncyRequested = StartFrequency + (unsigned long)(FrequencyScanCounter*(StopFrequency-StartFrequency)/FrequencyScanStep);
//
//  unsigned long StartTime = micros();
//  //vfo.setf(FrequncyRequested);
//  vfo.FastScanSetF(FrequncyRequested);
//  unsigned long ElapsedTime =  micros()-StartTime;
//  Serial.println("Computing Time us");
//  Serial.println(ElapsedTime);
//  if(ReverseScanDir==0)
//    FrequencyScanCounter = FrequencyScanCounter + 1;
//  else
//    FrequencyScanCounter = FrequencyScanCounter - 1;
//    
//  if(FrequencyScanCounter ==FrequencyScanStep) 
//  ReverseScanDir=1;
//  else if (FrequencyScanCounter ==0)
//  ReverseScanDir=0;
//
//  }
//  Serial.println("Frac");
// Serial.println(vfo.Frac) ;
//delay(1000);
//delayMicroseconds(1000);
 
  
}
