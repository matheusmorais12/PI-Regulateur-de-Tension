#include <Arduino.h>

class PID
{
public:
  double commande;
  double consigne;
  double mesure[2] = {};
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
  // Precisamos de tres variaveis para cada um, antes, o atual e o calculado.
  double addNewMesure(double _mesure)
  {
    mesure[0] = mesure[1];
    mesure[1] = _mesure;

    return mesure
  }

  void addNewConsigne(double _consigne)
  {
    consigne = _consigne; // saida do controlador
  }

  void addNewCommande(double _commande)
  {
    commande = _commande; // sinal entre PI e sistema
  }

  double Function_controled(double _mesure)
  {
    commande = 1 / alpha1 * (alpha * consigne - beta * consigne + gama * consigne - alpha * _mesure - beta * mesure[0] + gama * mesure[1] + beta1 * commande - gama1 * commande);
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
  pid_control.addNewCommande(mesure);
  pid_control.addNewMesure(mesure);
}
