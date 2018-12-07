#include <Wire.h>
#include "WM8731_AudioMod.h"
#include "arduinoFFT.h"
#include <LoRa.h>

arduinoFFT FFT = arduinoFFT(); /* Create FFT object */

// fixed parameters
#define FREQUENCY         915E6   // 915MHz
#define BANDWIDTH         125000  // 125kHz bandwidth
#define SLEEPTIME         4000    // 4 seconds

// vary these parameters
#define TX_POWER          8   // valid values are from 6 to 20
#define SPREADING_FACTOR  7    // valid values are 7, 8, 9 or 10

#define CONFIDENCE_TIME   250
#define RUNS_TO_SAVE      2
#define RECEIVE_WAIT_TIME 300

/*
These values can be changed in order to evaluate the functions
*/
const uint16_t SAMPLES = OUT_BUFF; //This value MUST NOT be modified, ALWAYS be a power of 2 
const double SAMP_FREC = 88200.0; //This value MUST NOT be modified

unsigned long startTime;

/*
These are the input and output vectors
Input vectors receive computed results from FFT
*/
double vReal[SAMPLES]; //= out_buf;
double vImag[SAMPLES]; // = {0.0};

double lastRuns[RUNS_TO_SAVE*2]= {0,0,0,0};

#define SCL_INDEX 0x00
#define SCL_TIME 0x01
#define SCL_FREQUENCY 0x02
#define SCL_PLOT 0x03

unsigned long prevTime, elapsedTime;

int location = 0;

boolean firstRun = true;

int max1 = 0;
int max2 = 0;

void setup(void) {
  // Set the LED pins low
  pinMode(6,OUTPUT);
  pinMode(7,OUTPUT);
  pinMode(8,OUTPUT);
  digitalWrite(6,LOW);
  digitalWrite(7,LOW);
  digitalWrite(8,LOW);

  SerialUSB.begin(115200);
  while(!SerialUSB); 
  // Set the LED pins high once serial communication has opened
  digitalWrite(6,HIGH);
  digitalWrite(7,HIGH);
  digitalWrite(8,HIGH);

  SerialUSB.println("Started");

  // Note start time
  startTime = millis();
  Codec.begin();
   
//  SerialUSB.print("out buffer at: ");
//  SerialUSB.println((long)out_buf);
  
  //SerialUSB.println("waiting");
  prevTime = millis();
  while(!out_buf_ready);
  elapsedTime = millis() - prevTime;
  //SerialUSB.println(elapsedTime);
    
  // Set up LoRa params
  LoRa.setPins(22, 59, 51);
  LoRa.begin(FREQUENCY);
  LoRa.enableCrc();
  
  LoRa.setTxPower(TX_POWER);
  LoRa.setSpreadingFactor(SPREADING_FACTOR);
  LoRa.setSignalBandwidth(BANDWIDTH);

  //print out the audio signal values in the buffer
//  for(uint16_t i=0; i<OUT_BUFF; ++i){
//    SerialUSB.println(out_buf[i]);
//  }  
 
}

void loop()
{
  prevTime = millis();
  for (uint16_t i = 0; i < SAMPLES; i++)
  {
    vReal[i] = (double)out_buf[i];
    vImag[i] = 0.0; //Imaginary part must be zeroed in case of looping to avoid wrong calculations and overflows
  }

  /* Print the results of the simulated sampling according to time */
  //SerialUSB.println("Data:");
  //PrintVector(vReal, SAMPLES, SCL_TIME);
  FFT.Windowing(vReal, SAMPLES, FFT_WIN_TYP_BLACKMAN, FFT_FORWARD);  /* Weigh data */
  //SerialUSB.println("Weighed data:");
  //PrintVector(vReal, SAMPLES, SCL_TIME);
  FFT.Compute(vReal, vImag, SAMPLES, FFT_FORWARD); /* Compute FFT */
  //SerialUSB.println("Computed Real values:");
  //PrintVector(vReal, SAMPLES, SCL_INDEX);
  //SerialUSB.println("Computed Imaginary values:");
  //PrintVector(vImag, SAMPLES, SCL_INDEX);
  FFT.ComplexToMagnitude(vReal, vImag, SAMPLES); /* Compute magnitudes */

  
  // Logging for debugging
  //SerialUSB.println(max1);
  //SerialUSB.println(max2);
  
  elapsedTime = millis() - prevTime;

  // Find odd frequencies
  discoverFrequencies(1);

  // Determine if we have found an answer
  boolean found = idetifiedFrequencies();

  // If we do not, go again on the even values
  if (!found){
    discoverFrequencies(2);
    found = idetifiedFrequencies();
  }

  // Logging for debugging
  //SerialUSB.println(found);
  //SerialUSB.println(max1);
  //SerialUSB.println(max2);

  // If no frequency found, skip this and loop. If frequency is found, stop looking and send.
  while (found){
    LoRa.beginPacket();
    // Passcode values with binary higher than 19k
    char w = 'W';
    char q = 'Q';
    
    // Divide frequencies into 4 bytes
    char v1 = char(max1 >> 8 & 0xFF);
    char v2 = char(max1 & 0xFF);
    char v3 = char(max2 >> 8 & 0xFF);
    char v4 = char(max2 & 0xFF);

    // Send the bytes to Lora 
    LoRa.print(w);
    LoRa.print(q);
    LoRa.print(v1);
    LoRa.print(v2);
    LoRa.print(v3);
    LoRa.print(v4);
    LoRa.endPacket();

    
    long timeDelay = millis();
    boolean receivedResponse = false;

    // Check for a receive message. If it we go beyond our wait time, try sending again
    while (!receivedResponse && millis() - timeDelay <= RECEIVE_WAIT_TIME){
      int size = LoRa.parsePacket();

      // Should just receive a one byte response
      if (size == 1){
        uint8_t response = (uint8_t)LoRa.read();

        // If the response is "200" shut down
        if (response = 200){
          receivedResponse = true;
          SerialUSB.println("received");
          SerialUSB.println(millis() - startTime);
          RTT->RTT_SR;
          pmc_enable_backupmode();
        }
      }
    }

    // On 2nd attempt and beyond, up the TX power
    LoRa.setTxPower(20);
  }
  
}

// Calculate max values given a starting point
void discoverFrequencies(int startPoint) {
  uint16_t max1Add = 0;
  uint16_t max2Add = 0;

  // Grab half the points and determine the one with the two maximums
  for (uint16_t i = startPoint; i < ((SAMPLES >> 1) +1); i+=2){
    if (vReal[i-1] < vReal[i] && vReal[i] > vReal[i+1]){
      if (vReal[i] > max1){
        max1 = vReal[i];
        max1Add = i;
      }
      else if (vReal[i] > max2){
        max2 = vReal[i];
        max2Add = i;
      }
    } 
  }

  double delta = 0.5 * ((vReal[max1Add-1] - vReal[max1Add+1]) / (vReal[max1Add-1] - (2.0 * vReal[max1Add]) + vReal[max1Add+1]));
  max1 = ((max1Add + delta)  * SAMP_FREC) / (SAMPLES-1);
  if(max1Add==(SAMPLES >> 1)) //To improve calculation on edge values
    max1 = ((max1Add + delta)  * SAMP_FREC) / (SAMPLES);

  delta = 0.5 * ((vReal[max2Add-1] - vReal[max2Add+1]) / (vReal[max2Add-1] - (2.0 * vReal[max2Add]) + vReal[max2Add+1]));
  max2 = ((max2Add + delta)  * SAMP_FREC) / (SAMPLES-1);
  if(max2Add==(SAMPLES >> 1)) //To improve calculation on edge values
    max2 = ((max2Add + delta)  * SAMP_FREC) / (SAMPLES);
}

// Determine if the fequencies have been found - majorly changed for final comp.
boolean idetifiedFrequencies() {
  // Must be in frequency range. If not, ignore the result.
  if (max1 < 200 || max1 > 19010 || max2 < 200 || max2 > 19010) {
    return false;
  }
  
  // If it is, send it! (Final competition change)
  else {
    return true;
  }

  // Original code: Hold the last two valid runs and compare is the new frequencies are within +-10. 
  // If they are, take the average. If not save and continue. 
  // Don't check the array if this is the first run. Only add it to the array.
  
//  if (!firstRun){
//    for (int i=0; i< RUNS_TO_SAVE*2; i= i+2){
//      if ((lastRuns[i] >= max1 - 10 && lastRuns[i] <= max1 + 10) && (lastRuns[i+1] >= max2 -10 && lastRuns[i+1] <=max2 +10)){
//        max1 = (max1 + lastRuns[i]) / 2;
//        max2 = (max2 + lastRuns[i+1]) /2; 
//        return true;
//      }
//    }
//  } else{
//    firstRun = false;
//  }

  // Always remove the oldest value in the array
  
//  lastRuns[location] = max1;
//  lastRuns[location+1] = max2;
//  if (location == 0){
//    location = 1;
//  }
//  else {
//    location = 0;
//  }
//  return false;
}


