#include <Arduino.h>

class PID
{
public:
  double commande[3] = {};
  double consigne[3] = {};
  double mesure[3] = {};
  double alpha, beta, gama;
  double alpha1, beta1, gama1;

  PID(double _alpha, double _beta, double _gama, double _alpha1, double _beta1, double _gama1)
  {
    alpha = _alpha;
    beta = _beta;
    gama = _gama;
    alpha1 = _alpha1;
    beta1 = _beta1;
    gama1 = _gama1;
  }

  void addNewMesure(double _mesure)
  {
    mesure[2] = mesure[1];
    mesure[1] = mesure[0];
    mesure[0] = _mesure;
  }

  void addNewConsigne(double _consigne)
  {
    consigne[2] = consigne[1];
    consigne[1] = consigne[0];
    consigne[0] = _consigne;
  }

  void addNewCommande()
  {
    commande[2] = commande[1];
    commande[1] = commande[0];
    commande[0] = Function_controled();
  }

  double Function_controled()
  {
    double Tcommande = 1 / alpha1 * (alpha * consigne[0] + beta * consigne[1] + gama * consigne[2] - alpha * mesure[0] - beta * mesure[1] - gama * mesure[2] - beta1 * commande[1] - gama1 * commande[2]);

    return Tcommande;
  }

  double Function_system()
  {
    double Tmesure = 1 / alpha1 * (alpha*commande[0], beta*commande[1] + gama*commande[2]
      - beta*mesure[1] - gama1*mesure[2];
    
    return Tmesure;
  }
};

#define Sensor A1
#define Control 3

PID pid_control(477.6, -952.4, 474.8, 1, -1.905, 0.9048);

void setup()
{
  Serial.begin(9600);
  pinMode(Sensor, INPUT);
  pinMode(Control, OUTPUT);
}

void loop()
{

  double mesure = map(analogRead(Sensor), 0, 1023, 0, 3);
  pid_control.addNewMesure(mesure);
}
