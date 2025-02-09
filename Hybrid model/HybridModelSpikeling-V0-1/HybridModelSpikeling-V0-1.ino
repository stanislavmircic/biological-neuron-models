// BackyardBrains 14. Avg. 2019
// Hybrid neuron model of neuron
// with parameter gs, which controls the amplitude and sign of
// the slow conductance
// V0.1
// Results of membrane potential is sent through serial port to Spike Recorder software
// Written by Stanislav Mircic
// Made for Arduino UNO
// Based on python code written by kathleen.coutisse@uliege.be
// It can do aprox. 2500 simulations per second

#define USE_DIGITAL_INPUTS 1
#define MAKE_SOUND 1
// set up pins
#define PhotoDiodePin A0 // Photodiode
#define LEDOutPin 9      // LED
#define ButtonPin 2      // Push button to switch spike modes

#define currentPotPin A3      // Resting membrane potential
#define gsPotPin A7    // efficacy synapse 1
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
byte outputFrameBuffer[12];   // 6ch * 2 bytes
volatile int numberOfChannels = 1;
//#define CURRENT_SHIELD_TYPE "HWT:SPIKELIN;"
//#define CURRENT_SHIELD_TYPE "HWT:MUSCLESS;"
#define SIZE_OF_COMMAND_BUFFER 30 //command buffer size
char commandBuffer[SIZE_OF_COMMAND_BUFFER];//receiving command buffer

// Initial conditions
float V=-60.0;//-10.*(rand(1)[1]-0.5)
float Vprev;
float xs;
float xu;
long Tstart1;

// Simulation parameters
const int T = 800;
//const int Ttransient = 300;
const float dt = 0.01;
 long Tdt;
//const long Tlow = convert(Int64,Ttransient/dt)


//const t = linspace(dt,T,Tdt)

// Model fixed parameters
const float Vth = 40;     // spike cut-off
const float as = 0.1;    // slope of the x_s nullcline
const float au = as;    // ultra slow
const float b = -2;      // 2 * geometrical parameters to let the asymptotes of the v-nullclines asymetric
const float c = -45;     // voltage reset
const float d = 30;       // x_s reset (slow variable)
const float dz = 20;      // amount of increase of slow variable following an AP
const float Vsyn = -75.0;
const float taus = 1.0;
const float asyn = 0.1;
const float Vss = -2.0;
const float gL = 0.0;//0.4    // leak current
const float VL = -40;
const float Vshift = -70;   // shift in Vm to have physiological values

// Model flexible parameters
const float epss = 1.0;
const float epsu = 0.025;

const float gs1 = 10.0;
const float gs2 = -30.0;
const float gu = 1.0;
float Iapp = 15.0;
float gs;

const int Tstep = 400;

uint16_t photodiodeInputCurrent=0;
float digitalInputCurrentAmplitude = 100;
float VV;

void setup() 
{
  Serial.begin(230400);
  Tdt = T/dt;
  Vprev=V;
  xs=as*V;
  xu=au*V;
  VV = V;
  Tstart1 = Tstep/dt;
  //Make ADC sample faster. Change ADC clock
  //Change prescaler division factor to 16
  sbi(ADCSRA,ADPS2);//1
  cbi(ADCSRA,ADPS1);//0
  cbi(ADCSRA,ADPS0);//0 

  pinMode(AnalogOutPin, OUTPUT); // 11 "digital" PWM
  pinMode(DigitalOutPin, OUTPUT); // 3 digital
  pinMode(LEDOutPin, OUTPUT); // 9 digital PWM
  pinMode(DigitalIn1Pin, INPUT_PULLUP); // 4 digital
  pinMode(DigitalIn2Pin, INPUT_PULLUP); // 5 digital
  pinMode(ButtonPin, INPUT); // 2 digital
  pinMode(PhotoDiodePin, INPUT); // 0 analog
  pinMode(AnalogInPin, INPUT); // 5 analog
  
  pinMode(currentPotPin,INPUT); // 3 analog VmPotPin
  pinMode(gsPotPin,INPUT); // 7 analog Syn1PotPin
  //pinMode(gKmaxPotPin,INPUT); // 5 analog Syn2PotPin // this one also controls the Analog In gain!
  pinMode(NoisePotPin,INPUT); // 6 analog NoisePotPin
  
  pinMode(DEBUG_PIN,OUTPUT);
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

// Hybrid model
float dV(float V, float xs, float xu, float gs, float gu, float Iapp)
{
  return (dt)*(gL*((V-Vshift)-VL) + sq((V-Vshift)) - sq(xs) + b*(V-Vshift)*xs - gs*xs - gu*xu + Iapp);
}
float dxs(float V, float xs, float epss)
{
  return (dt*epss)*(as*(V-Vshift) - xs);
}
float dxu(float V, float xu, float epsu)
{
  return (dt*epsu)*(au*(V-Vshift) - xu);
}

void loop() {
      //gs = gs1 *(z<Tstart1) + gs2 *(z>=Tstart1)
      Iapp = 0;
      //get voltage from photodiode/photoreceptor
      photodiodeInputCurrent = analogRead(PhotoDiodePin);
      Iapp += photodiodeInputCurrent*4;//5us

      gs = analogRead(gsPotPin)*0.25-128;
      
      float Iss = (-sq(-2*Vss*(1-as*as+as*b)-as*(gs+gu))+as*as*sq(gs+gu))/(4*(1-as*as+as*b));
      V += dV(Vprev,xs,xu,gs,gu,Iss+Iapp);
      xs += dxs(Vprev,xs,epss);
      xu += dxu(Vprev,xu,epsu);

      if ((V-Vshift) > Vth)
      {
          VV = Vth;
          V = c;
          xs = d;
          xu += dz;
      }
      else
      {
        VV = Vprev;
      }

      Vprev=V;
      
      uint16_t voltage = 4*(VV+100);
      //convert data to frame according to protocol
      outputFrameBuffer[0]= (voltage>>7)| 0x80;           
      outputFrameBuffer[1]=  voltage & 0x7F;  
     // VV[z] = copy(V)
     // II[z] = copy(Iss+Iapp)
     Serial.write(outputFrameBuffer,2);
}
