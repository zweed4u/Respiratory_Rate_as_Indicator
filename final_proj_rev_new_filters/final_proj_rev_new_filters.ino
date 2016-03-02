
//****************************************************************************
/*
this version implements the new fir which replaces the two 1st order filters.


*/
//****************************************************************************


#include <MsTimer2.h>

const int TSAMP_MSEC = 5;
int const NUMSAMP = 3600;
volatile boolean sampleFlag = false;
int smplCount = 0;
float xv, yv_LPF1, yv_LPF2, yv_SCon, yv_HPF2;

//RunSD Variables
const int LEN_SIGMA_BUFF = 60;
boolean lowBPM_flag=false, highBPM_flag=false;
float xv_SD, LPF_SD, HPF_SD;

//Dither Constants
const int pinDither_bit_2 = A5;  // A5, 47K resistor, dither DAC MSB
const int pinDither_bit_1 = A4;  // A4, 100K resistor 
const int pinDither_bit_0 = A3;  // A3, 220K resistor, dither DAC LSB
const int pinLM61 = A0;         // A0 ADC input for LM61
    
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
xv = fetchSimulatedSampleDegC(smplCount);     // simulated data
//xv = sample2floatDegC(fetchAveragedSample()); // sensor data
//xv = sample2floatDegC(fetchDitheredAveragedSample()); // sensor data (Dithered)
  
  // --------------  IIR processing  --------------------------
  yv_SCon = sensorCondition_FIR(xv);
  xv_SD = winSD_xv(yv_SCon,smplCount);
  yv_HPF2 = HPF_IIR_order_4_25(yv_SCon);
  HPF_SD = winSD_HP(yv_HPF2,smplCount);
  yv_LPF2 = LPF_IIR_order_4_11(yv_SCon);
  LPF_SD = winSD_LP(yv_LPF2,smplCount);
  decisionMaker();
  displayFilterInOut(smplCount);

  
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
//Decision Maker: Is the normalized stdev > 0.5?
void decisionMaker()
{
  float HPF_norm = 0, LPF_norm = 0;

  HPF_norm = HPF_SD/xv_SD;
  LPF_norm = LPF_SD/xv_SD;

  if(HPF_norm > 0.5) {highBPM_flag = true;}
  else {highBPM_flag = false;}

  if(LPF_norm > 0.5) {lowBPM_flag = true;}
  else {lowBPM_flag = false;}
  
}

//*******************************************************************************
//Windowed Standard Deviation for Input Signal
float winSD_xv(float x, int n)
{
  // const int LEN_SIGMA_BUFF = 60;  
  static float xv_Win[LEN_SIGMA_BUFF] = {0};
  static float sd = 0;
  static float avg = 0;
  int i;
  
  avg += (x*x);
  if( (n % 5) == 0 ) 
  {   
    for(i=(LEN_SIGMA_BUFF-1); i>0; i--)  //Shifting
    {
      xv_Win[i] = xv_Win[i-1];
    }
    xv_Win[0] = avg;
    avg = 0;
    for(i=0; i<LEN_SIGMA_BUFF; i++)
    {
      avg += xv_Win[i];
    }
    avg = avg/(LEN_SIGMA_BUFF-1);
    sd = sqrt(avg);
    avg = 0; 
  }

  if(smplCount < 100) {return 0;}
  else {return sqrt(sd);} //Standard Deviation
}

//*******************************************************************************
//Windowed Standard Deviation for HPF
float  winSD_HP(float x, int n)
{
  static float xv_Win[LEN_SIGMA_BUFF] = {0};
  static float sd = 0;
  static float avg = 0;
  int i;
  
  avg += (x*x);
  if( (n % 5) == 0 ) 
  {
    for(i=(LEN_SIGMA_BUFF-1); i>0; i--)  //Shifting
    {
      xv_Win[i] = xv_Win[i-1];
    }
    xv_Win[0] = avg;
    avg = 0;
    for(i=0; i<LEN_SIGMA_BUFF; i++)
    {
      avg += xv_Win[i];
    }
    avg = avg/(LEN_SIGMA_BUFF-1);
    sd = sqrt(avg);
    avg = 0; 
  }

  if(smplCount < 100) {return 0;}
  else {return sqrt(sd);} //Standard Deviation
}


//*******************************************************************************
//Windowed Standard Deviation for LPF
float  winSD_LP(float x, int n)
{
  static float xv_Win[LEN_SIGMA_BUFF] = {0};
  static float sd = 0;
  static float avg = 0;
  int i;
  
  avg += (x*x);
  if( (n % 5) == 0 ) 
  {
    for(i=(LEN_SIGMA_BUFF-1); i>0; i--)  //Shifting
    {
      xv_Win[i] = xv_Win[i-1];
    }
    xv_Win[0] = avg;
    avg = 0;
    for(i=0; i<LEN_SIGMA_BUFF; i++)
    {
      avg += xv_Win[i];
    }
    avg = avg/(LEN_SIGMA_BUFF-1);
    sd = sqrt(avg);
    avg = 0; 
  }

  if(smplCount < 100) {return 0;}
  else {return sqrt(sd);} //Standard Deviation
}

//*******************************************************************************
//Simulated Sample Data with Freq Sweep Options
float fetchSimulatedSampleDegC(int tick)
{
 
  float brthPerMin;
  float bpmBuff = 20;

   float swp1 = 0.20*NUMSAMP;
  float swp2 = 0.40*NUMSAMP;
  float swp3 = 0.60*NUMSAMP;
  float swp4 = 0.80*NUMSAMP;
  
  if(tick < swp1){         // [0, swp1) 
     brthPerMin = bpmBuff;           
  }
  else if(tick < swp2){   // [swp1, swp2) NUMSAMP = (bpm1/60)*(bpmBuff/60)*(bpm2/60)*1000
    brthPerMin = 5;
  }
  else if(tick < swp3){   // [swp1, swp2)
    brthPerMin = bpmBuff;
  }
  else if(tick < swp4){  //  [swp2, swp3)
    brthPerMin = 60;
  }
  else {                 //  [swp3, inf)
    brthPerMin = bpmBuff;
  }
  
  const float degC_DC = 20.0;
  const float degC_pk2pk = 2.0;
  
  float brthRadPerSec = (2.0*PI)*(brthPerMin/60.0);
  const float cornerBPM = 1.0;
  const float packageGainAC = cornerBPM/brthPerMin;
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
float sensorCondition_FIR(float xv)
{ 
// FIR Thermal Compensator: LM61 tau = 6.7 seconds
  const float h_tcomp[] = {-0.285317,
   0.117739, 3.380663, 7.590242, 4.456542, -4.456558, 
   -7.590244, -3.380658, -0.117737, 0.285317};
  const int DCV_KRNL_LEN = 10;
  
  // Declare static float xbuff[] for keeping rolling horizon
  // of input variable history.
  static float xbuff[DCV_KRNL_LEN] = {0};
  static boolean isFirstPass = true;
 
  if (isFirstPass)
 { 
   for (int i = 0; i < DCV_KRNL_LEN; i++)
  {
    xbuff[i] = xv;
  }
  isFirstPass = false;
 }
 
  // Shift input variable history and make room for new xv input.
  for (int i = (DCV_KRNL_LEN-1); i > 0; i--) {xbuff[i] = xbuff[i-1];}
  xbuff[0] = xv;

  float floatSum = 0.0;

    for (int i = 0; i < DCV_KRNL_LEN; i++)
    {
      floatSum += h_tcomp[i]*xbuff[i];
    } 
    
  return floatSum;
}

//*******************************************************************************
float LPF_IIR_order_4_11(float x)
{ 
// LPF, 6 pole, R = 0.50, 11.0 BPM
  const float b1[3] = {1.000000000000000, 2.005732082260212, 1.005743028217340};
  const float a1[3] = {1.000000000000000, -1.933337083557243, 0.935356382436755};
  const float b2[3] = {1.000000000000000, 2.000006555492989, 1.000017520368095};
  const float a2[3] = {1.000000000000000, -1.944686853082009, 0.952331117141776};
  const float b3[3] = {1.000000000000000, 1.994261362246800, 0.994272345801208};
  const float a3[3] = {1.000000000000000, -1.968870818610703, 0.982309478918726};
  const float GAIN = 0.0014518;
 
  static float xv1M1 = 0.0, xv1M2 = 0.0, yv1M1 = 0.0, yv1M2 = 0.0;
  static float xv2M1 = 0.0, xv2M2 = 0.0, yv2M1 = 0.0, yv2M2 = 0.0;
  static float xv3M1 = 0.0, xv3M2 = 0.0, yv3M1 = 0.0, yv3M2 = 0.0;
  static float xv4M1 = 0.0, xv4M2 = 0.0, yv4M1 = 0.0, yv4M2 = 0.0;
  static float xv5M1 = 0.0, xv5M2 = 0.0, yv5M1 = 0.0, yv5M2 = 0.0;
  static float xv6M1 = 0.0, xv6M2 = 0.0, yv6M1 = 0.0, yv6M2 = 0.0;
//  static float xv7M1 = 0.0, xv7M2 = 0.0, yv7M1 = 0.0, yv7M2 = 0.0;
//  static float xv8M1 = 0.0, xv8M2 = 0.0, yv8M1 = 0.0, yv8M2 = 0.0;
//  static float xv9M1 = 0.0, xv9M2 = 0.0, yv9M1 = 0.0, yv9M2 = 0.0;
  
  x = GAIN*x;
  float yv = -a1[1]*yv1M1 - a1[2]*yv1M2 + b1[0]*x + b1[1]*xv1M1 + b1[2]*xv1M2;
  yv1M2 = yv1M1; yv1M1 = yv;
  xv1M2 = xv1M1; xv1M1 = x;
  
  x = GAIN*yv;
  yv = -a2[1]*yv2M1 - a2[2]*yv2M2 + b2[0]*x + b2[1]*xv2M1 + b2[2]*xv2M2;
  yv2M2 = yv2M1; yv2M1 = yv;
  xv2M2 = xv2M1; xv2M1 = x;

  x = GAIN*yv;
  yv = -a3[1]*yv3M1 - a3[2]*yv3M2 + b3[0]*x + b3[1]*xv3M1 + b3[2]*xv3M2;
  yv3M2 = yv3M1; yv3M1 = yv;
  xv3M2 = xv3M1; xv3M1 = x;

//  x = GAIN*yv;
//  yv = -a4[1]*yv4M1 - a4[2]*yv4M2 + b4[0]*x + b4[1]*xv4M1 + b4[2]*xv4M2;
//  yv4M2 = yv4M1; yv4M1 = yv;
//  xv4M2 = xv4M1; xv4M1 = x;
//
//  x = GAIN*yv;
//  yv = -a5[1]*yv5M1 - a5[2]*yv5M2 + b5[0]*x + b5[1]*xv5M1 + b5[2]*xv5M2;
//  yv5M2 = yv5M1; yv5M1 = yv;
//  xv5M2 = xv5M1; xv5M1 = x;
//
//
//    x = GAIN*yv;
//  yv = -a6[1]*yv6M1 - a6[2]*yv6M2 + b6[0]*x + b6[1]*xv6M1 + b6[2]*xv6M2;
//  yv6M2 = yv6M1; yv6M1 = yv;
//  xv6M2 = xv6M1; xv6M1 = x;
  
//  xv = GAIN*yv;
//  yv = -a7[1]*yv7M1 - a7[2]*yv7M2 + b7[0]*xv + b7[1]*xv7M1 + b7[2]*xv7M2;
//  yv7M2 = yv7M1; yv7M1 = yv;
//  xv7M2 = xv7M1; xv7M1 = x;
  
//  xv = GAIN*yv;
//  yv = -a8[1]*yv8M1 - a8[2]*yv8M2 + b8[0]*xv + b8[1]*xv8M1 + b8[2]*xv8M2;
//  yv8M2 = yv8M1; yv8M1 = yv;
//  xv8M2 = xv8M1; xv8M1 = x;
  
//  xv = GAIN*yv;
//  yv = -a9[1]*yv9M1 - a9[2]*yv9M2 + b9[0]*xv + b9[1]*xv9M1 + b9[2]*xv9M2;
//  yv9M2 = yv9M1; yv9M1 = yv;
//  xv9M2 = xv9M1; xv9M1 = x;
  
  return yv;
}
//*************************************************************************************
float HPF_IIR_order_4_25(float x)
{ 
    // HPF, 14 pole, R = 0.10, 39 BPM
  const float b1[3] = {1.000000000000000, -2.022093802369678, 1.000259324768207};
  const float a1[3] = {1.000000000000000, -0.047751459331497, 0.081313469169566};
  const float b2[3] = {1.000000000000000, -2.279106475279344, 1.303990086757544};
  const float a2[3] = {1.000000000000000, -0.777046763891544, 0.440151421540589};
  const float b3[3] = {1.000000000000000, -2.176363027792016, 1.200034080004262};
  const float a3[3] = {1.000000000000000, -1.291026325613942, 0.694453503176919};
  const float b4[3] = {1.000000000000000, -1.748032623429234, 0.767065268227059};
  const float a4[3] = {1.000000000000000, -1.556502949635831, 0.828171655613515};
  const float b5[3] = {1.000000000000000, -1.813465946791463, 0.833164468629980};
  const float a5[3] = {1.000000000000000, -1.696773308608107, 0.902226542264819};
  const float b6[3] = {1.000000000000000, -2.044556736501715, 1.066716801692669};
  const float a6[3] = {1.000000000000000, -1.776311555501438, 0.948986914509463};
  const float b7[3] = {1.000000000000000, -1.916381387836546, 0.937146152494435};
  const float a7[3] = {1.000000000000000, -1.824155799529684, 0.983992383808879};
  const float GAIN = 0.6945;
 
    static float xv1M1 = 0.0, xv1M2 = 0.0, yv1M1 = 0.0, yv1M2 = 0.0;
  static float xv2M1 = 0.0, xv2M2 = 0.0, yv2M1 = 0.0, yv2M2 = 0.0;
  static float xv3M1 = 0.0, xv3M2 = 0.0, yv3M1 = 0.0, yv3M2 = 0.0;
  static float xv4M1 = 0.0, xv4M2 = 0.0, yv4M1 = 0.0, yv4M2 = 0.0;
  static float xv5M1 = 0.0, xv5M2 = 0.0, yv5M1 = 0.0, yv5M2 = 0.0;
  static float xv6M1 = 0.0, xv6M2 = 0.0, yv6M1 = 0.0, yv6M2 = 0.0;
  static float xv7M1 = 0.0, xv7M2 = 0.0, yv7M1 = 0.0, yv7M2 = 0.0;
//  static float xv8M1 = 0.0, xv8M2 = 0.0, yv8M1 = 0.0, yv8M2 = 0.0;
//  static float xv9M1 = 0.0, xv9M2 = 0.0, yv9M1 = 0.0, yv9M2 = 0.0;
  
  x = GAIN*x;
  float yv = -a1[1]*yv1M1 - a1[2]*yv1M2 + b1[0]*x + b1[1]*xv1M1 + b1[2]*xv1M2;
  yv1M2 = yv1M1; yv1M1 = yv;
  xv1M2 = xv1M1; xv1M1 = x;
  
  x = GAIN*yv;
  yv = -a2[1]*yv2M1 - a2[2]*yv2M2 + b2[0]*x + b2[1]*xv2M1 + b2[2]*xv2M2;
  yv2M2 = yv2M1; yv2M1 = yv;
  xv2M2 = xv2M1; xv2M1 = x;

  x = GAIN*yv;
  yv = -a3[1]*yv3M1 - a3[2]*yv3M2 + b3[0]*x + b3[1]*xv3M1 + b3[2]*xv3M2;
  yv3M2 = yv3M1; yv3M1 = yv;
  xv3M2 = xv3M1; xv3M1 = x;

  x = GAIN*yv;
  yv = -a4[1]*yv4M1 - a4[2]*yv4M2 + b4[0]*x + b4[1]*xv4M1 + b4[2]*xv4M2;
  yv4M2 = yv4M1; yv4M1 = yv;
  xv4M2 = xv4M1; xv4M1 = x;

  x = GAIN*yv;
  yv = -a5[1]*yv5M1 - a5[2]*yv5M2 + b5[0]*x + b5[1]*xv5M1 + b5[2]*xv5M2;
  yv5M2 = yv5M1; yv5M1 = yv;
  xv5M2 = xv5M1; xv5M1 = x;


    x = GAIN*yv;
  yv = -a6[1]*yv6M1 - a6[2]*yv6M2 + b6[0]*x + b6[1]*xv6M1 + b6[2]*xv6M2;
  yv6M2 = yv6M1; yv6M1 = yv;
  xv6M2 = xv6M1; xv6M1 = x;
  
  x = GAIN*yv;
  yv = -a7[1]*yv7M1 - a7[2]*yv7M2 + b7[0]*x + b7[1]*xv7M1 + b7[2]*xv7M2;
  yv7M2 = yv7M1; yv7M1 = yv;
  xv7M2 = xv7M1; xv7M1 = x;
  
//  xv = GAIN*yv;
//  yv = -a8[1]*yv8M1 - a8[2]*yv8M2 + b8[0]*xv + b8[1]*xv8M1 + b8[2]*xv8M2;
//  yv8M2 = yv8M1; yv8M1 = yv;
//  xv8M2 = xv8M1; xv8M1 = x;
  
//  xv = GAIN*yv;
//  yv = -a9[1]*yv9M1 - a9[2]*yv9M2 + b9[0]*xv + b9[1]*xv9M1 + b9[2]*xv9M2;
//  yv9M2 = yv9M1; yv9M1 = yv;
//  xv9M2 = xv9M1; xv9M1 = x;
  
  return yv;
}

void displayFilterInOut(int smplCount)
{
  if (smplCount == 0) Serial.print(F("\nsmpl\txv\tyv FIR\tyv LPF2\tyv HPF 2\tSD_xv\tSD_LPF2\tLPF2_FLAG\tSD_HPF2\tHPF2_FLAG\n"));
  Serial.print(smplCount); Serial.print('\t');
  Serial.print(xv,4); Serial.print('\t');
  Serial.print(yv_SCon,4); Serial.print('\t');
  Serial.print(yv_LPF2,4); Serial.print('\t');
  Serial.print(yv_HPF2,4); Serial.print('\t');
  Serial.print(xv_SD, 4); Serial.print('\t');
  Serial.print(LPF_SD,4); Serial.print('\t');
  Serial.print(lowBPM_flag); Serial.print('\t');
  Serial.print(HPF_SD,4); Serial.print('\t'); 
  Serial.print(highBPM_flag); Serial.println('\t');
  
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

