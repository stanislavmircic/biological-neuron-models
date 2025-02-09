// BackyardBrains 31. July 2019
// Hodgkin–Huxley model of neuron
// V0.3
// Results Vm is sent through serial port to Spike Recorder software
// Written by Stanislav Mircic
// Made for Arduino UNO
// Based on python code written by Dr Alexander Borst
// It can do aprox. 2160 simulations per second

#define USE_DIGITAL_INPUTS 1
#define MAKE_SOUND 1
// set up pins
#define PhotoDiodePin A0 // Photodiode
#define LEDOutPin 9      // LED
#define ButtonPin 2      // Push button to switch spike modes

#define currentPotPin A3      // Resting membrane potential
#define gNamaxPotPin A7    // efficacy synapse 1
#define gKmaxPotPin A5    // efficacy synapse 2
#define NoisePotPin A6   // scaling of Noise level 

#define DigitalIn1Pin 4  // Synapse 1 Input - expects 5V pulses
#define DigitalIn2Pin 5  // Synapse 2 input - expects 5V pulses
#define AnalogInPin A2   // Analog in- takes 0-5V (positive only)
#define DigitalOutPin 3  // "Axon" - generates 5V pulses
#define AnalogOutPin 11  // Analog out for full spike waveform
#define DEBUG_PIN 12  // Analog out for full spike waveform

// defines for setting and clearing register bits
#ifndef cbi
#define cbi(sfr, bit) (_SFR_BYTE(sfr) &= ~_BV(bit))
#endif
#ifndef sbi
#define sbi(sfr, bit) (_SFR_BYTE(sfr) |= _BV(bit))
#endif

volatile int numberOfChannels = 1;
//#define CURRENT_SHIELD_TYPE "HWT:SPIKELIN;"
//#define CURRENT_SHIELD_TYPE "HWT:MUSCLESS;"
#define SIZE_OF_COMMAND_BUFFER 30 //command buffer size
char commandBuffer[SIZE_OF_COMMAND_BUFFER];//receiving command buffer
float analogInputGain;
int analogInMaxAmplitude = 500;
byte outputFrameBuffer[12];   // 6ch * 2 bytes
float deltat=0.1;
float memcap=1.0;
float gleak=1.0;
int NoiseAmpl = 100;
float mmidv=28.0;
float mslope=0.35;
float mtau=1.0;

float hmidv=17.0;
float hslope=-0.15;
float htau=2.0;

float nmidv=28.0;
float nslope=0.15;
float ntau=3.0;
uint16_t positiveVm=0;
uint16_t photodiodeInputCurrent=0;
float digitalInputCurrentAmplitude = 100;
float currentSum = 0.0;

//float mss[201];
//float hss[201];
//float nss[201];

float current=0.0;

float randomCurrent = 0;

float gNamax=100;
float gKmax=30;


float Vm = 0.0;
float gNa = 0.0;
float gK = 0.0;
    
float m=0;
float n=0;
float h=0;

float curramp = 10;
boolean spike = false;
float mss[101];
float mconstDiv;
float hss[101];
float hconstDiv;
float nss[101];
float nconstDiv;

int noisePotValue = 0;
int potCountdown =  0;
float randNormConst = 0;
float analogInNormConst = 0;
void setup() {
      Serial.begin(230400);
      Serial.setTimeout(2);
      //myx=np.linspace(0,200,201)-100
      //mss=1.0/(1.0+np.exp((mmidv-myx)*mslope))
      int myx = -100;
      for (int i=0;i<101;i++)
      {
          mss[i] = 1.0/(1.0+exp((mmidv-myx)*mslope));
          hss[i] = 1.0/(1.0+exp((hmidv-myx)*hslope));
          nss[i] = 1.0/(1.0+exp((nmidv-myx)*nslope));
          //because we don't have enough memory we will 
          //pre-calulate voltage for every even mV and if we 
          //get odd value of mV we will approximate with even 
          myx+=2;
      }
      mconstDiv = 1.0/(1.0*mtau/deltat);
      hconstDiv = 1.0/(1.0*htau/deltat);
      nconstDiv = 1.0/(1.0*ntau/deltat);
      //Make ADC sample faster. Change ADC clock
      //Change prescaler division factor to 16
      sbi(ADCSRA,ADPS2);//1
      cbi(ADCSRA,ADPS1);//0
      cbi(ADCSRA,ADPS0);//0 
      //pinMode(LED_BUILTIN, OUTPUT); // 13 digital
      pinMode(AnalogOutPin, OUTPUT); // 11 "digital" PWM
      pinMode(DigitalOutPin, OUTPUT); // 3 digital
      pinMode(LEDOutPin, OUTPUT); // 9 digital PWM
      pinMode(DigitalIn1Pin, INPUT_PULLUP); // 4 digital
      pinMode(DigitalIn2Pin, INPUT_PULLUP); // 5 digital
      pinMode(ButtonPin, INPUT); // 2 digital
      pinMode(PhotoDiodePin, INPUT); // 0 analog
      pinMode(AnalogInPin, INPUT); // 5 analog
      
      pinMode(currentPotPin,INPUT); // 3 analog VmPotPin
      pinMode(NoisePotPin,INPUT); // 6 analog NoisePotPin
      pinMode(gKmaxPotPin,INPUT); // 5 analog Syn2PotPin // this one also controls the Analog In gain!
      pinMode(gNamaxPotPin,INPUT); // 7 analog Syn1PotPin
      
      
      pinMode(DEBUG_PIN,OUTPUT);

      randNormConst = NoiseAmpl/1024.0;
      analogInNormConst = analogInMaxAmplitude/1024.0;

}

void calc_Vm()
{
      uint16_t voltageIndex = Vm+100;
      voltageIndex = voltageIndex>>1;//divide with 2 to get mV index //20us
      m=((mss[voltageIndex])-m)*mconstDiv+m;//60us
      n=((nss[voltageIndex])-n)*nconstDiv+n;//80us
      h=((hss[voltageIndex])-h)*hconstDiv+h;//35us
      gNa=gNamax*(m*m*m)*h;//40us
      gK =gKmax *(n*n*n*n);//50us
      Vm=currentSum +100*gNa-30*gK+Vm*10;///memcap/deltat;//70us
      Vm=Vm/(gleak+gNa+gK+10);//memcap/deltat);//50us
      //This below takes ~0us
      if(Vm<-100)
      {
        Vm = -100;  
      }
      if(Vm>100)
      {
        Vm = 100;  
      } 
}

//
// Create a random integer from 0 - 65535
//
unsigned int rng() {
  static unsigned int y = 0;
  y += micros(); // seeded with changing number
  y ^= y << 2; y ^= y >> 7; y ^= y << 7;
  return (y);
} 

void loop() {
      
      
      
      //------------------------ Reading inputs ---------------------------------
      if(potCountdown==0)
      {
          potCountdown = 100;
          gNamax = analogRead(gNamaxPotPin)*0.25;
          gKmax = analogRead(gKmaxPotPin)*0.25;
          analogInputGain = (analogRead(currentPotPin)-512)*0.001953125;// 1/512 = 0.001953125
          noisePotValue = analogRead(NoisePotPin);
      }
      else
      {
          potCountdown--;  
      }
      currentSum = 0.0;

      //get input gain and current 
      
      current = analogRead(AnalogInPin)*analogInNormConst*analogInputGain;//30us
      currentSum += current;
      
      //get voltage from photodiode/photoreceptor
      photodiodeInputCurrent = analogRead(PhotoDiodePin);//10us
      currentSum += photodiodeInputCurrent*4;//5us
      
      //add random component
      randomCurrent = ((rng())/32767.0)-1;
      randomCurrent *= randNormConst;
      randomCurrent *= float(noisePotValue);
      currentSum += randomCurrent;

      
      //check if we have stimmulation through digital input
      if(USE_DIGITAL_INPUTS)
      {
          if(digitalRead(DigitalIn1Pin)== LOW){currentSum+=digitalInputCurrentAmplitude;}
          if(digitalRead(DigitalIn2Pin)== LOW){currentSum+=digitalInputCurrentAmplitude;}
      }
    
      //digitalWrite(DigitalOutPin, HIGH);//debug
      //------------------------ Calculate model ---------------------------------
      calc_Vm();
      //digitalWrite(DigitalOutPin, LOW);//debug
    
      
      //------------------------ Writting outputs --------------------------------
      positiveVm = Vm+100;
      analogWrite(AnalogOutPin,positiveVm);

      if (MAKE_SOUND) 
      {
          if  (Vm>50 && spike==false) 
          {
              spike=true;
              digitalWrite(DigitalOutPin, HIGH);
          } 
          else
          {
              digitalWrite(DigitalOutPin, LOW);
              if(Vm<30)
              {
                spike=false;
              }
          }  
      }   
      // check if there has been a spike for digi out routine (below)
      // trigger audio click and Digi out 5V pulse if there has been a spike

      //Serial.println(Vm);
    
     
      
      //------------------ Sending data through serial ---------------------------
      uint16_t voltage = 4*(positiveVm);
      //convert data to frame according to protocol
      outputFrameBuffer[0]= (voltage>>7)| 0x80;           
      outputFrameBuffer[1]=  voltage & 0x7F;  
      
      //uint16_t total_current = (uint16_t)(currentSum+100);
      //convert data to frame according to protocol
     // outputFrameBuffer[2]= (total_current>>7)& 0x7F;           
     // outputFrameBuffer[3]=  total_current & 0x7F;  
      Serial.write(outputFrameBuffer,2);
    
    
      
      //----------------- Reading data from serial ------------------------------
      if(Serial.available()>0)
      {         
          String inString = Serial.readStringUntil('\n');
        
          //convert string to null terminate array of chars
          inString.toCharArray(commandBuffer, SIZE_OF_COMMAND_BUFFER);
          commandBuffer[inString.length()] = 0;
          //Serial.print(commandBuffer);
          
          // breaks string str into a series of tokens using delimiter ";"
          // Namely split strings into commands
          char* command = strtok(commandBuffer, ";");
          while (command != 0)
          {
              // Split the command in 2 parts: name and value
              char* separator = strchr(command, ':');
              if (separator != 0)
              {
                  // Actually split the string in 2: replace ':' with 0
                  *separator = 0;
                  --separator;
                  if(*separator == 'c')//if we received command for number of channels
                  {
                    separator = separator+2;
                    numberOfChannels = atoi(separator);//read number of channels
                  }
                  if(*separator == 'b')//if we received command for impuls
                  {
                    //sendMessage(CURRENT_SHIELD_TYPE);
                  }
              }
              // Find the next command in input string
              command = strtok(0, ";");
          }
      }//end of reading data from serial
                
}//end of main loop
