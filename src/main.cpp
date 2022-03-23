#include <Arduino.h>


class PID{
public:
  
  double error;
  double sample; 
  double lastSample;  
  double kP, kI, kD;     
  
  double P, I, D;
  double pid;
  
  double setPoint = 3; // Variável de controle (Valor que queremos de tensão )
  long lastProcess;
  
  PID(double _kP, double _kI, double _kD){
    kP = _kP;
    kI = _kI;
    kD = _kD;
  }
  
  void addNewSample(double _sample){
    sample = _sample;
  }
  
  void setSetPoint(double _setPoint){
    setPoint = _setPoint;
  }
  
  double process(){
    // Implementação do PID

    error = setPoint - sample;
    float deltaTime = (millis() - lastProcess) / 1000.0;
    lastProcess = millis();
    
    //P
    P = error * kP;
    
    //I
    I += (error * kI) * deltaTime;
    
    //D
    D = (lastSample - sample) * kD / deltaTime;
    lastSample = sample;
    
     
    pid = P + I + D;
    
    return pid;
  }
};

#define pSENSOR         A1
#define pCONTROLE       3


PID PID_Control(1.0, 0, 0);

void setup() {
  Serial.begin(9600);
  
  pinMode(pSENSOR, INPUT);
  pinMode(pCONTROLE, OUTPUT);
}

int controlePwm = 0;

void loop() {
  
  // ADC   
  double sample = map(analogRead(pSENSOR), 0, 1023, 0, 100);

  PID_Control.addNewSample(sample);
   
  controlePwm += PID_Control.process() ;

  // Saída do controle
  analogWrite(pCONTROLE, controlePwm);
  
}