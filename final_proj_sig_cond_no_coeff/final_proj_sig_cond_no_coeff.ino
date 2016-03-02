//this version added a windowed std dev for the input signal
//this is used to normalize the other two sd's: HPF_sd / iNSD.  this effectively normalizes it
//then a decision making path: is the normalized sd greater than 50%? if so set off that alarm (either alarm_low_bpm or alarm_high_bpm)

#include <MsTimer2.h>

const int TSAMP_MSEC = 5, NUMSAMP = 1024;
volatile boolean sampleFlag = false;
int smplCount = 0;
float xv;

//Dither Constants
const int pinDither_bit_2 = A5;  // A5, 47K resistor, dither DAC MSB
const int pinDither_bit_1 = A4;  // A4, 100K resistor 
const int pinDither_bit_0 = A3;  // A3, 220K resistor, dither DAC LSB
const int pinLM61 = A0;         // A0 ADC input for LM61

float dithAvgXv, avgXv;
//*******************************************************************************
void setup()
{
  Serial.begin(115200);

  //Added for Dithering
  pinMode(pinDither_bit_0, OUTPUT); digitalWrite(pinDither_bit_0, LOW); // set pins as outputs
  pinMode(pinDither_bit_1, OUTPUT); digitalWrite(pinDither_bit_1, LOW); 
  pinMode(pinDither_bit_2, OUTPUT); digitalWrite(pinDither_bit_2, LOW);  
  
  initializeADC(); 
  startSampleTimerISR();   
}
 
//*******************************************************************************
void loop()
{  
  awaitSampleStart();
  
  // ------------  Input acquisition  -------------------------
//xv = ((smplCount == 0) ? 1.0 : 0.0);          // impulse input
//xv = fetchSimulatedSampleDegC(smplCount);     // simulated data
//xv = sample2floatDegC(fetchAveragedSample()); // sensor data
 //xv = sample2floatDegC(fetchDitheredAveragedSample()); // sensor data (Dithered)

  
  xv = (float)analogRead(0);
  xv = sample2floatDegC(xv);
  dithAvgXv = sample2floatDegC(fetchDitheredAveragedSample() );
  avgXv = sample2floatDegC(fetchAveragedSample() );

 
 
  
  // --------------  IIR processing  --------------------------
  //yv_HPF1 = sensorCondition_FIR(xv);
//  yv_HPF1 = HPF_IIR_order_1_1(xv);
//  yv_LPF1 = LPF_IIR_order_4_150(yv_HPF1);
//  xv_SD = winSD_xv(yv_LPF1);
//  yv_HPF2 = HPF_IIR_order_4_25(yv_LPF1);
//  HPF_SD = winSD_HP(yv_HPF2);
//  yv_LPF2 = LPF_IIR_order_4_11(yv_LPF1);
//  LPF_SD = winSD_LP(yv_LPF2);
  displayFilterInOut(smplCount);

//    //Decision Making Section
//  float norm = HPF_SD/xv_SD;
//  if(norm > 0.5) {highBPM_flag = true;}
//  else {highBPM_flag = false;}
  
  // --------------  loop processing  -------------------------  
  smplCount++;  
  if (smplCount == NUMSAMP) stopSampleTimerISR();
 
} // loop()

//*******************************************************************************
void awaitSampleStart(void)
{
  while (!sampleFlag);
  sampleFlag = false;
//  if (smplCount%5 == 0) Serial.print("*"); // quell impatience
} 
//*******************************************************************************
//Simulated Sample Data with Freq Sweep Options
float fetchSimulatedSampleDegC(int tick)
{
  float num = (float) NUMSAMP;
  float brthPerMin;
  float swp1 = num*(0.25);
  float swp2 = num*(0.5);
  float swp3 = num*(0.75);
  
  if(tick < swp1){         // [0, swp1) 
     brthPerMin = 15;           
  }
  else if(tick < swp2){   // [swp1, swp2)
    brthPerMin = 5;
  }
  else if(tick < swp3){  //  [swp2, swp3)
    brthPerMin = 30;
  }
  else {                 //  [swp3, inf)
    brthPerMin = 20;
  }

  // const float brthPerMin = 17;
  
  const float degC_DC = 20.0;
  const float degC_pk2pk = 2.0;
  
  float brthRadPerSec = (2.0*PI)*(brthPerMin/60.0);
  const float cornerRadPerSec = 0.06;
  const float packageGainAC = 1.0; // cornerRadPerSec/brthRadPerSec;
  const float TSIM = 0.1;
   
  float dieDegC, deltaDegC;
   
  // compute thermal mass LPF attenuated degC st sensor die  
  deltaDegC = (degC_pk2pk/2.0)*sin(brthRadPerSec*tick*TSIM);
  dieDegC = degC_DC + packageGainAC*deltaDegC;
 
  return dieDegC;
}

//*******************************************************************************
float fetchAveragedSample()
{  
  // ADC conversion time is 25 usec.  See initializeADC()
  // Collect 400 samples in 10 msec. Average to produce one datapoint
  
  const int NUM_SAMPLES_AVG = 400;
  float sampleADC = 0.0;
  for (int i = 0; i < NUM_SAMPLES_AVG; i++)
  {
    sampleADC +=(float)analogRead(0);
  }
  sampleADC = sampleADC/NUM_SAMPLES_AVG;
  
  return sampleADC;
}

//*******************************************************************************
//Dithered Averaging
float fetchDitheredAveragedSample()
{
  const int NUM_SAMPLES_AVG = 400;
  float sum = 0.0;
  int rand_number = 0;
  
  for (int i=0;i<NUM_SAMPLES_AVG;i++) { // get all the repeated readings and sum them up
      rand_number = i % 8; // creates a ramping value between 0 and 7 using Modulo function
      digitalWrite(pinDither_bit_0, (rand_number & B00000001)); //Bit mask
      digitalWrite(pinDither_bit_1, (rand_number & B00000010)); //Bit mask
      digitalWrite(pinDither_bit_2, (rand_number & B00000100)); //Bit mask
      sum=sum+analogRead(pinLM61);
  }
  sum = sum/NUM_SAMPLES_AVG; // calculate averageReading   

  return sum;
}

//*******************************************************************************
float sample2floatDegC(float sampleADC)
{
  // Convert 1.1 Vref ADC float value to unsigned int milli-degC
  
  const float countsPerVolt = 1024/1.1; // 10 bits over 1.1V reference range
  const float voltsSensorOffset = 0.600;
  const float degCperVolt = 100.0; // 10 mV/degC
  
  float degC = ((sampleADC/countsPerVolt) - voltsSensorOffset)*degCperVolt;
  
  return degC;
}
//*******************************************************************************
void displayFilterInOut(int smplCount)
{
  if (smplCount == 0) Serial.print(F("\nsmpl\txv\tAVG_ONLY\tDTH_AVG_SAMP\n"));
  Serial.print(smplCount); Serial.print('\t');
  Serial.print(xv,4); Serial.print('\t');
  Serial.print(avgXv,4); Serial.print('\t');
  Serial.print(dithAvgXv,4); Serial.println('\t');
  
}

//*******************************************************************************
void initializeADC(void)
{
  // Set ADC Prescaler for 500 kHz conversion clock -> 25 usec/conversion
  // (default is 125 kHz -> 100 usec/conversion)
  ADCSRA &= 0b11111000; // clear prescaler select bits
  ADCSRA |= 0b00000101; // prescaler = 32 -> 25 usec/conv 

  //analogReference(DEFAULT);   // for 5 volt power supply reference
  analogReference(INTERNAL);   // for 1.1 volt power supply reference
  analogRead(0); // prime ADC timing
}

//*******************************************************************************
// --------------- Sample timing ISRs ------------------
//*******************************************************************************
void Interrupt_To_Set_Sample_Flag()
{
  sampleFlag = true;
}

//*******************************************************************************
void startSampleTimerISR()
{
  MsTimer2::set(TSAMP_MSEC, Interrupt_To_Set_Sample_Flag); // set sample period msec
  MsTimer2::start();
//  Serial.println(F("\nTimer Started"));
}
 
//*******************************************************************************
void stopSampleTimerISR()
{
   MsTimer2::stop();
//   Serial.println( F("\nTimer Stopped")); 
}

