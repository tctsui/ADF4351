/*
ADF4351.cpp -ADF4351 PLL communication library

Most of this code is based on the code written by David Fannin (dfannin).
The register writing part of this code is adapted from the code written by Neal Pisenti (npisenti)

I have added two functions FastscanabilityCheck and FastScanSetF. Instead of updating all the 6 registers, FastScanSetF only update the register 0 when called, thus saving some computation time.
FastscanabilityCheck makes sure the other 5 registers remain the same during the frequency scan.


   This program is free software: you can redistribute it and/or modify
   it under the terms of the GNU General Public License as published by
   the Free Software Foundation, either version 3 of the License, or
   (at your option) any later version.
   This program is distributed in the hope that it will be useful,
   but WITHOUT ANY WARRANTY; without even the implied warranty of
   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
   GNU General Public License for more details.
   You should have received a copy of the GNU General Public License
   along with this program.  If not, see <http://www.gnu.org/licenses/>.
   
*/

#include "ADF4351.h"

uint32_t steps[] = { 1000, 5000, 10000, 50000, 100000 , 500000, 1000000 }; ///< Array of Allowed Step Values (Hz)




// ADF4351 settings


ADF4351::ADF4351(byte pin, uint8_t mode, unsigned long  speed, uint8_t order )
{
  spi_settings = SPISettings(speed, order, mode) ;
  pinSS = pin ;
  // settings for 100 mhz internal
  reffreq = REF_FREQ_DEFAULT ;
  enabled = false ;
  cfreq = 0 ;
  ChanStep = steps[2] ;
  RD2refdouble = 0 ;
  RCounter = 1 ;
  RD1Rdiv2 = 0 ;
  BandSelClock = 200 ;
  ClkDiv = 80 ;
  Prescaler = 1 ;
  pwrlevel = 0 ;
  PowerDown = 0;
}

void ADF4351::init()
{
  pinMode(pinSS, OUTPUT) ;
  digitalWrite(pinSS, LOW) ;
  pinMode(PIN_CE, OUTPUT) ;
  pinMode(PIN_LD, INPUT) ;
  SPI.begin();
} ;

int ADF4351::FastscanabilityCheck(uint32_t StartF, uint32_t StopF,uint32_t ScanStep)
{
	if(StartF > ADF_FREQ_MAX || StopF > ADF_FREQ_MAX || StartF < ADF_FREQ_MIN || StopF < ADF_FREQ_MIN) {
	  Serial.println("The upper or lower bound is outside the frequency range. The maximum frequency is 4294967295");
	  return 1;
	}
	
	int LocalOscRStart =   2200000000UL / StartF;
	int LocalOscRStop =   2200000000UL / StopF ;
	
	if (LocalOscRStart != LocalOscRStop ) {
	  Serial.println("Scan range too large. Only Register 0 should be updated in this mode. Current scan range requires updating other registers as well. ");
	  return 1;
	}
	if ( abs(StartF-StopF)/ScanStep<ChanStep) {
	  Serial.println(F("ScanStep too large. The ScanStep being used is larger than the step size being used. "));
	  return 1;
	}
	
	outdiv = 1 ;
	RfDivSel = 0 ;

  // select the output divider
	while (  outdiv <=  LocalOscRStart   && outdiv <= 64 ) {
      outdiv *= 2 ;
      RfDivSel++  ;
	}
  
	if(StartF> 3600000000UL )
      Prescaler = 1 ;
	else
      Prescaler = 0 ; 

	RD1Rdiv2 = 1;// 1 is needed for Fast Lock to ensure 50% duty cycle
	
  PFDFreq = (float) reffreq  * ( (float) ( 1.0 + RD2refdouble) / (float) (RCounter * (1.0 + RD1Rdiv2)));  // find the loop freq
  BigNumber::begin(10) ;
  char tmpstr[20] ;
  // kludge - BigNumber doesn't like leading spaces
  // so you need to make sure the string passed doesnt
  // have leading spaces.
  int cntdigits = 0 ;
  uint32_t num = (uint32_t) ( PFDFreq / 10000 ) ;

  while ( num != 0 )  {
    cntdigits++ ;
    num /= 10 ;
  }

  dtostrf(PFDFreq, cntdigits + 8 , 3, tmpstr) ;
  // end of kludge
  BigNumber BN_PFDFreq = BigNumber(tmpstr) ;
  
  BigNumber BN_N = ( BigNumber(StartF) * BigNumber(outdiv) ) /BN_PFDFreq ;
  N_Int =  (uint16_t) ( (uint32_t)  BN_N ) ;
  BigNumber BN_Mod = BigNumber(((uint32_t)PFDFreq)) / (BigNumber(ChanStep)* BigNumber(outdiv)) ;
  Mod =((uint32_t) BN_Mod) ;
  

  if (Mod>=4095)
  {
	  uint32_t RCheck =ceil((float) reffreq  * ( (float) ( 1.0 + RD2refdouble) / (float) (4094*ChanStep* (1.0 + RD1Rdiv2)*outdiv)));
	/*
	  When the Mod is larger than the upper bound for the current choice of R, we can overcome this issue by using a larger R.
	*/
	  if(RCheck <1023)
	  {
		  if(RCheck %2==0)
		  {
			  RCounter=RCheck ;
		  }
		  else
		  {
			  RCounter=RCheck + 1;
		  }
		  PFDFreq =(float) reffreq  * ( (float) ( 1.0 + RD2refdouble) / (float) (RCounter * (1.0 + RD1Rdiv2))); 
		  cntdigits = 0 ;
		  num = (uint32_t) ( PFDFreq / 10000 ) ;

		  while ( num != 0 )  {
          cntdigits++ ;
          num /= 10 ;
          }
		  dtostrf(PFDFreq, cntdigits + 8 , 3, tmpstr) ;
		  BN_PFDFreq = BigNumber(tmpstr) ;
		  BN_N = ( BigNumber(StartF) * BigNumber(outdiv) ) / BN_PFDFreq ;
		  N_Int =  (uint16_t) ( (uint32_t)  BN_N ) ;
		  BN_Mod = BN_PFDFreq / (BigNumber(ChanStep)* BigNumber(outdiv)) ;
		  Mod = ((uint32_t) BN_Mod);

	  }
		else 
		{
		Serial.println(F("Change step size too small")) ;
		return 1;
		}
	  
  } 
    BigNumber BN_Diff= BigNumber(StartF)* BigNumber(outdiv)-BigNumber(N_Int)*BN_PFDFreq;


	BigNumber BN_Frac = BN_Diff * BN_Mod/ BN_PFDFreq;
	
	
 	if(((uint32_t)PFDFreq)/5000000UL==0)
		BandSelClock=1;
	else{
		BandSelClock=((uint32_t)(PFDFreq/((float)5000000))) + 1; 
		if(BandSelClock > 255)	BandSelClock=255;
	}
	
	/*
	The BandSelClock divider should be set to a value such that PFDFreq/BandSelClock is smaller than 500kHz (DBit23 in Register 3 is set to 1)
	,125kHz (DBit23 in Register 3 is set to 0)
	*/
	
	ClkDiv = (int)(((10+(20e-6)*PFDFreq)/Mod) + 1);
	/*
    The value of ClkDiv determines the duration of wide bandiwdth mode used in fast lock mode. 10 cycles of PFDFreq of band selection of VCO and 20 us for PLL in wide bandwidth mode.
    */
	
	Frac =((uint32_t) BN_Frac);
	if ( Mod < 2 || Mod > 4095) {
    Serial.println(F("Mod out of range")) ;
    return 1 ;
  }

  if ( (uint32_t) Frac > (Mod - 1) ) {
    Serial.println(F("Frac out of range")) ;
    return 1 ;
  }

  if ( Prescaler == 0 && ( N_Int < 23  || N_Int > 65535)) {
    Serial.println(F("N_Int out of range")) ;
    return 1;

  } else if ( Prescaler == 1 && ( N_Int < 75 || N_Int > 65535 )) {
    Serial.println(F("N_Int out of range")) ;
    return 1;
  }

  // setting the registers to default values
 


	ADF4351::setR5();
	ADF4351::setR4();
	ADF4351::setR3FastScan();
	ADF4351::setR2FastScan();
	ADF4351::setR1();
  	ADF4351::setR0();

	ADF4351::writeRegister(_r5);

	ADF4351::writeRegister(_r4);

	ADF4351::writeRegister(_r3);

	ADF4351::writeRegister(_r2);

	ADF4351::writeRegister(_r1);

	ADF4351::writeRegister(_r0); 
	
	



  return 0 ;  // ok
}

int ADF4351::FastScanSetF(uint32_t freq){

 	BigNumber::begin(10) ;  
	BigNumber BN_N = ( BigNumber(freq) * BigNumber(outdiv) ) /BigNumber(((uint32_t)PFDFreq)) ;
	N_Int =  (uint16_t) ( (uint32_t)  BN_N ) ;
	BigNumber BN_Diff= BigNumber(freq)* BigNumber(outdiv)-BigNumber(N_Int)*BigNumber(((uint32_t)PFDFreq));
	Frac = (uint32_t) ((((float)((uint32_t)BN_Diff) )/ PFDFreq)*Mod + 0.5);
	/*
	I try to avoid using BigNumber in this funcitn to reduce the computation time.
	*/
	ADF4351::setR0();
	ADF4351::writeRegister(_r0); 
	BigNumber::finish() ;
	
	return 0;
	
}

int  ADF4351::setf(uint32_t freq)
{
  //  calculate settings from freq
  if ( freq > ADF_FREQ_MAX ) return 1 ;

  if ( freq < ADF_FREQ_MIN ) return 1 ;

  int localosc_ratio =   2200000000UL / freq ;
  outdiv = 1 ;
  RfDivSel = 0 ;

  // select the output divider
  while (  outdiv <=  localosc_ratio   && outdiv <= 64 ) {
    outdiv *= 2 ;
    RfDivSel++  ;
  }

  if(freq> 3600000000UL )
    Prescaler = 1 ;
  else
    Prescaler = 0 ; 

 
  RD1Rdiv2 = 1 ;// 1 is needed for Fast Lock to ensure 50% duty cycle
  PFDFreq = (float) reffreq  * ( (float) ( 1.0 + RD2refdouble) / (float) (RCounter * (1.0 + RD1Rdiv2)));  // find the loop freq
  BigNumber::begin(10) ;
  char tmpstr[20] ;
  // kludge - BigNumber doesn't like leading spaces
  // so you need to make sure the string passed doesnt
  // have leading spaces.
  int cntdigits = 0 ;
  uint32_t num = (uint32_t) ( PFDFreq / 10000 ) ;

  while ( num != 0 )  {
    cntdigits++ ;
    num /= 10 ;
  }

  dtostrf(PFDFreq, cntdigits + 8 , 3, tmpstr) ;
  // end of kludge
  BigNumber BN_PFDFreq = BigNumber(tmpstr) ;
  
  /*
	Converting the PFDFreq from floating number to integer 
  */
  
  BigNumber BN_N = ( BigNumber(freq) * BigNumber(outdiv) ) /BN_PFDFreq ;
  N_Int =  (uint16_t) ( (uint32_t)  BN_N ) ;
  BigNumber BN_Mod = BigNumber(((uint32_t)PFDFreq)) / (BigNumber(ChanStep)* BigNumber(outdiv)) ;
  /*
	Depending on the required resolution, we obtain a value 
  */
  
  Mod =((uint32_t) BN_Mod) ;
  
  
  
  
  if (Mod>=4095)
  {		
	uint32_t RCheck =ceil((float) reffreq  * ( (float) ( 1.0 + RD2refdouble) / (float) (4094*ChanStep * (1.0 + RD1Rdiv2)*outdiv)));
	/*
	  When the Mod is larger than the upper bound for the current choice of R, we can overcome this issue by using a larger R.
	*/
	  if(RCheck <1023)
	  {
		  if(RCheck %2==0)
		  {
			  RCounter=RCheck ;
		  }
		  else
		  {
			  RCounter=RCheck+1 ;
		  }
		  /*
		    A even R is preferred
		  */
		  
		  PFDFreq =(float) reffreq  * ( (float) ( 1.0 + RD2refdouble) / (float) (RCounter * (1.0 + RD1Rdiv2))); 
		  cntdigits = 0 ;
		  num = (uint32_t) ( PFDFreq / 10000 ) ;

		  while ( num != 0 )  {
          cntdigits++ ;
          num /= 10 ;
          }
		  dtostrf(PFDFreq, cntdigits + 8 , 3, tmpstr) ;
		  BN_PFDFreq = BigNumber(tmpstr) ;
		  BN_N = ( BigNumber(freq) * BigNumber(outdiv) ) / BN_PFDFreq ;
		  N_Int =  (uint16_t) ( (uint32_t)  BN_N ) ;
		  BN_Mod = BN_PFDFreq / (BigNumber(ChanStep)* BigNumber(outdiv)) ;
		  Mod = ((uint32_t) BN_Mod);

	  }
		else 
		{
		  Serial.println(F("Change step size too small")) ;
		  return 1;
		}
	  
  } 

  BigNumber BN_Diff= BigNumber(freq)* BigNumber(outdiv)-BigNumber(N_Int)*BN_PFDFreq;
  BigNumber BN_Frac = BN_Diff * BN_Mod/ BN_PFDFreq+ BigNumber("0.5");//Always round up
  Frac =((uint32_t) BN_Frac);

  if ( Frac != 0  ) {
    uint32_t gcd = gcd_iter(Frac, Mod) ;

    if ( gcd > 1 ) {
      Frac /= gcd ;
      BN_Frac = BigNumber(Frac) ;
      Mod /= gcd ;
      BN_Mod = BigNumber(Mod) ;
    }
  }

 	if(((uint32_t)PFDFreq)/5000000UL==0)
	  BandSelClock=1;
	else{
	  BandSelClock=((uint32_t)(PFDFreq/((float)5000000))) + 1; 
	  if(BandSelClock > 255)	BandSelClock=255;
	}
	
	/*
	The BandSelClock divider should be set to a value such that PFDFreq/(10*BandSelClock) is smaller than 500kHz (DBit23 in Register 3 is set to 1)
	,125kHz (DBit23 in Register 3 is set to 0)
	*/


  cfreq = PFDFreq* (((float) N_Int)+((float) Frac )/((float) Mod))/outdiv;
  ClkDiv = (int)(((10+(20e-6)*PFDFreq)/Mod) + 1);
  /*
    The value of ClkDiv determines the duration of wide bandiwdth mode used in fast lock mode. 10 cycles of PFDFreq of band selection of VCO and 20 us for PLL in wide bandwidth mode.
  */


  if ( cfreq != freq ) Serial.println(F("output freq diff than requested")) ;

  BigNumber::finish() ;

  if ( Mod < 2 || Mod > 4095) {
    Serial.println(F("Mod out of range")) ;
    return 1 ;
  }

  if ( (uint32_t) Frac > (Mod - 1) ) {
    Serial.println(F("Frac out of range")) ;
    return 1 ;
  }

  if ( Prescaler == 0 && ( N_Int < 23  || N_Int > 65535)) {
    Serial.println(F("N_Int out of range")) ;
    return 1;

  } else if ( Prescaler == 1 && ( N_Int < 75 || N_Int > 65535 )) {
    Serial.println(F("N_Int out of range")) ;
    return 1;
  }

  // setting the registers to default values
 


	ADF4351::setR5();
	ADF4351::setR4();
	ADF4351::setR3FastScan();
	ADF4351::setR2FastScan();
	ADF4351::setR1();
  	ADF4351::setR0();

	ADF4351::writeRegister(_r5);

	ADF4351::writeRegister(_r4);

	ADF4351::writeRegister(_r3);

	ADF4351::writeRegister(_r2);

	ADF4351::writeRegister(_r1);

	ADF4351::writeRegister(_r0); 




  return 0 ;  // ok
}

int ADF4351::setrf(uint32_t f)
{
  if ( f > ADF_REFIN_MAX ) return 1 ;

  if ( f < 100000UL ) return 1 ;

  float newfreq  =  (float) f  * ( (float) ( 1.0 + RD2refdouble) / (float) (RCounter * (1.0 + RD1Rdiv2)));  // check the loop freq

  if ( newfreq > ADF_PFD_MAX ) return 1 ;

  if ( newfreq < ADF_PFD_MIN ) return 1 ;

  reffreq = f ;
  return 0 ;
}

void ADF4351::enable()
{
  enabled = true ;
  digitalWrite(PIN_CE, HIGH) ;
}

void ADF4351::disable()
{
  enabled = false ;
  digitalWrite(PIN_CE, LOW) ;
}




void ADF4351::setR0(){
    unsigned long r0 = (N_Int << 15)+ // Set INT value
						(Frac<<3)//Set Frac value	
						+0;	
    byte r0Ary[] = { lowByte(r0 >> 24), lowByte(r0 >> 16), lowByte(r0 >> 8), lowByte(r0) };
	//Serial.println("r0");
	//Serial.println(r0);
    memcpy(&_r0, &r0Ary, sizeof(r0Ary));
}

void ADF4351::setR1(){
    unsigned long r1 =  (Prescaler <<27) + //Set Prescaler
			(1 << 15) + // phase value = 1 recommended by the data sheet
            (Mod << 3) + // modulus value 
             1; // register value
	//Serial.println("r1");
	//Serial.println(r1);
    byte r1Ary[] = { lowByte(r1 >> 24), lowByte(r1 >> 16), lowByte(r1 >> 8), lowByte(r1) };
    memcpy(&_r1, &r1Ary, sizeof(r1Ary));
}

void ADF4351::setR2(){
	unsigned long r2;
	if (Frac == 0){
			r2 =   (0 << 26) +  // muxout not used right now
			(RD2refdouble<<25)+//reference doubler
			(RD1Rdiv2<<24)+//RDIV2
            (RCounter << 14) +  // r-counter = 1
            (8 << 9) +  // charge pump = 1.56
			(1 << 8) + // Set LDF to int-N mode
			(1 << 7) + // set LDP to 6 ns for int-N mode
            (1 << 6) +  // digital lock detect + polarity
            (PowerDown << 5) +   // powerdown 0 = false; 1 = true
            2; // register value
    }
	else{
			r2 =   (0 << 26) +  // muxout not used right now
			(RD2refdouble<<25)+//reference doubler
			(RD1Rdiv2<<24)+//RDIV2
            (RCounter << 14) +  // r-counter = 1
            (8 << 9) +  // charge pump = 1.56
			(0 << 8) + // Set LDF to Frac mode
			(0 << 7) + // Set LDP to Frac mode
            (1 << 6) +  // digital lock detect + polarity
            (PowerDown << 5) +   // powerdown 0 = false; 1 = true
            2; // register value	
	}
	//Serial.println("r2");
	//Serial.println(r2);
    byte r2Ary[] =  { lowByte(r2 >> 24), lowByte(r2 >> 16), lowByte(r2 >> 8), lowByte(r2) };
    memcpy(&_r2, &r2Ary, sizeof(r2Ary));
}

void ADF4351::setR2FastScan(){
	unsigned long r2;
	if (Frac == 0){
			r2 =   
			(0 << 30) +  //Low noise mode
			(0 << 29) + //Low noise mode
			(0 << 26) +  // muxout not used right now
			(RD2refdouble<<25)+//reference doubler
			(RD1Rdiv2<<24)+//RDIV2
            (RCounter << 14) +  // r-counter = 1
            (10 << 9) +  // charge pump = 2.5
			(1 << 8) + // Set LDF to int-N mode
			(1 << 7) + // set LDP to 6 ns for int-N mode
            (1 << 6) +  // digital lock detect + polarity
            (PowerDown << 5) +   // powerdown 0 = false; 1 = true
            2; // register value
    }
	else{
			r2 =
			(0 << 30) +  //Low noise mode
			(0 << 29) + //Low noise mode			
			(0 << 26) +  // muxout not used right now
			(RD2refdouble<<25)+//reference doubler
			(RD1Rdiv2<<24)+//RDIV2
            (RCounter << 14) +  // r-counter = 1
            (10 << 9) +  // charge pump = 2.5
			(0 << 8) + // Set LDF to Frac mode
			(0 << 7) + // Set LDP to Frac mode
            (1 << 6) +  // digital lock detect + polarity
            (PowerDown << 5) +   // powerdown 0 = false; 1 = true
            2; // register value	
	}
	//Serial.println("r2");
	//Serial.println(r2);
    byte r2Ary[] =  { lowByte(r2 >> 24), lowByte(r2 >> 16), lowByte(r2 >> 8), lowByte(r2) };
    memcpy(&_r2, &r2Ary, sizeof(r2Ary));
}

void ADF4351::setR3(){
	unsigned long r3;
	if (Frac == 0){
				r3 = (0 << 23) + // Band Select Clock Mode
				(1 << 22) +  //  ABP, int-n
				(1 << 21) + //  charge cancel, reduces pfd spurs
				( ClkDiv << 3) + // Clock divder value
				3; // (all zero, except register control value = 3);
	}
	else{
				r3 = (0 << 23) + // Band Select Clock Mode
				(0 << 22) +  //  ABP, Frac
				(0 << 21) + //  charge cancel, reduces pfd spurs
				( ClkDiv << 3) + // Clock divder value
				3; // (all zero, except register control value = 3);	
	}
	//Serial.println("r3");
	//Serial.println(r3);
   byte r3Ary[] = { lowByte(r3 >> 24), lowByte(r3 >> 16), lowByte(r3 >> 8), lowByte(r3) };
   memcpy(&_r3, &r3Ary, sizeof(r3Ary));
}

 void ADF4351::setR3FastScan(){
	unsigned long r3;
	if (Frac == 0){
				r3 = (1 << 23) + // Band Select Clock Mode
				(1 << 22) +  //  ABP, int-n
				(1 << 21) + //  charge cancel, reduces pfd spurs
				(1 << 18) + //Cycle slips reduction on
				(0 << 16) + // Enable Fast Lock;
				(1 << 15) + //Enable Fast Lock;
				( ClkDiv << 3) + // Clock divder value
				3; // (all zero, except register control value = 3);
	}
	else{
				r3 = (1 << 23) + // Band Select Clock Mode
				(0 << 22) +  //  ABP, Frac
				(0 << 21) + //  charge cancel, reduces pfd spurs
				(1 << 18) + //Cycle slips reduction on
				(0 << 16) + // Enable Fast Lock;
				(1 << 15) + //Enable Fast Lock;
				( ClkDiv << 3) + // Clock divder value
				3; // (all zero, except register control value = 3);	
	}
	//Serial.println("r3");
	//Serial.println(r3);
   byte r3Ary[] = { lowByte(r3 >> 24), lowByte(r3 >> 16), lowByte(r3 >> 8), lowByte(r3) };
   memcpy(&_r3, &r3Ary, sizeof(r3Ary));
} 

void ADF4351::setR4(){

    unsigned long r4 = (1 << 23) + // fundamental feedback
        (RfDivSel << 20) + // rf divider select
        (BandSelClock << 12) + // band select clock divider
        (0 << 9) + // vco powerdown = false; MTLD = 1; aux output = divided;
        (0 << 8) + // AUX OUTPUT enable/disable
        (0 << 6) + // aux output power = {-4, -1, 2, 5dbm}
        (1 << 5) + // RF OUTPUT ENABLED
        (pwrlevel << 3) + // RF output power 
        4;  // register select
	//Serial.println("r4");
	//Serial.println(r4);
   byte r4Ary[] = { lowByte(r4 >> 24), lowByte(r4 >> 16), lowByte(r4 >> 8), lowByte(r4) };
   memcpy(&_r4, &r4Ary, sizeof(r4Ary));


}

void ADF4351::setR5(){

    unsigned long r5 = (1 << 22) + (3<<19) + 5; // lock detect pin mode = digital lock detect
	//Serial.println("r5");
	//Serial.println(r5);
   byte r5Ary[] = { lowByte(r5 >> 24), lowByte(r5 >> 16), lowByte(r5 >> 8), lowByte(r5) };
   memcpy(&_r5, &r5Ary, sizeof(r5Ary));

}


uint32_t ADF4351::gcd_iter(uint32_t u, uint32_t v)
{
  uint32_t t;

  while (v) {
    t = u ;
    u = v ;
    v = t % v ;
  }

  return u ;
  
}

void ADF4351::writeRegister(byte data[]){


    digitalWrite(pinSS, LOW);
	//delayMicroseconds(10) ;
    // Writes the data
    for(int i = 0; i < 4 ; i++){
        SPI.transfer(data[i]);
    }

    digitalWrite(pinSS, HIGH);
  //delayMicroseconds(5) ;
  //digitalWrite(pinSS, LOW) ;


}