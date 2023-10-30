#include <Arduino.h>
#include <Servo.h>
#include <QTRSensors.h>
#define PinInterrupcion1 2
#define Sensor1 A15
#define Sensor2 A14
#define Sensor3 A13
#define Sensor4 A12
#define Sensor5 A11
#define Sensor6 A10
#define LedRojo 33
#define LedVerde 35
#define LedAzul 37
#define SensorOUT 50
#define M1Adelante 45
#define M1Atras 47
#define M2Adelante 49
#define M2Atras 51
#define PWM1 10
#define PWM2 9
long duracion;
long distancia;
const int s0 = 22;    //Ultrasonico
const int s1 = 24;    //Ultrasonico
const int s2 = 34;    //Ultrasonico
const int s3 = 36;    //Ultrasonico
const int out = 32;   //Ultrasonico
int velocidad1 = 140; //PWM para el motor uno
int velocidad2 = 240; //PWM para el motor dos
int Trig = 30;        //Ultrasonico
int Echo = 31;        //Ultrasonico
int pinservo = 44;    //Envio de señal para el Servomotor
int pulsomin = 520;   //520  Configuración valor minimo del ServoMotor
int pulsomax = 2650;  //2650 Configuración valor maximo del ServoMotor
volatile int contador1 = 0;
int NumeroDePulsosEncoder = 360;
char dato;

Servo servo1;
QTRSensors qtr;
byte countRed = 0;
byte countGreen = 0;
byte countBlue = 0;
const uint8_t SensorCount = 6;
uint16_t sensorValues[SensorCount];

void interrupcion1()
{
  contador1++;
}

void setup()
{
  //Serial.begin(115200);
  Serial1.begin(115200);
  attachInterrupt(digitalPinToInterrupt(PinInterrupcion1), interrupcion1, RISING);
  servo1.attach(pinservo, pulsomin, pulsomax);
  pinMode(M1Adelante, OUTPUT);
  pinMode(M1Atras, OUTPUT);
  pinMode(M2Adelante, OUTPUT);
  pinMode(M2Atras, OUTPUT);
  pinMode(PWM1, OUTPUT);
  pinMode(PWM2, OUTPUT);
  pinMode(LedRojo, OUTPUT);
  pinMode(LedVerde, OUTPUT);
  pinMode(LedAzul, OUTPUT);
  pinMode(Trig, OUTPUT);
  pinMode(Echo, INPUT);
  pinMode(s0, OUTPUT);
  pinMode(s1, OUTPUT);
  pinMode(s2, OUTPUT);
  pinMode(s3, OUTPUT);
  pinMode(out, INPUT);
  digitalWrite(s0, HIGH);
  digitalWrite(s1, HIGH);
  qtr.setTypeAnalog();
  qtr.setSensorPins((const uint8_t[]){Sensor1, Sensor2, Sensor3, Sensor4, Sensor5, Sensor6}, SensorCount);
  qtr.setEmitterPin(SensorOUT);

  delay(500);

  for (uint16_t i = 0; i < 400; i++)
  {
    qtr.calibrate();
  }

  for (uint8_t i = 0; i < SensorCount; i++)
  {
    Serial.print(qtr.calibrationOn.minimum[i]);
    Serial.print(' ');
  }

  Serial.println();

  for (uint8_t i = 0; i < SensorCount; i++)
  {
    Serial.print(qtr.calibrationOn.maximum[i]);
    Serial.print(' ');
  }

  Serial.println();
  Serial.println();
  analogWrite(PWM1, velocidad1);
  analogWrite(PWM2, velocidad2);
  delay(1000);
}

void getColor()
{
  digitalWrite(s2, LOW);
  digitalWrite(s3, LOW);
  countRed = pulseIn(out, digitalRead(out) == HIGH ? LOW : HIGH);
  digitalWrite(s3, HIGH);
  countBlue = pulseIn(out, digitalRead(out) == HIGH ? LOW : HIGH);
  digitalWrite(s2, HIGH);
  countGreen = pulseIn(out, digitalRead(out) == HIGH ? LOW : HIGH);
}

void Adelante()
{
  digitalWrite(M1Adelante, HIGH);
  digitalWrite(M1Atras, LOW);
  digitalWrite(M2Adelante, HIGH);
  digitalWrite(M2Atras, LOW);
}
void Atras()
{
  digitalWrite(M1Adelante, LOW);
  digitalWrite(M1Atras, HIGH);
  digitalWrite(M2Adelante, LOW);
  digitalWrite(M2Atras, HIGH);
}
void Derecha()
{
  digitalWrite(M1Adelante, HIGH);
  digitalWrite(M1Atras, LOW);
  digitalWrite(M2Adelante, LOW);
  digitalWrite(M2Atras, HIGH);
}
void Izquierda()
{
  digitalWrite(M1Adelante, LOW);
  digitalWrite(M1Atras, HIGH);
  digitalWrite(M2Adelante, HIGH);
  digitalWrite(M2Atras, LOW);
}

void Stop()
{
  digitalWrite(M1Adelante, LOW);
  digitalWrite(M1Atras, LOW);
  digitalWrite(M2Adelante, LOW);
  digitalWrite(M2Atras, LOW);
}

void Grado20()
{
  NumeroDePulsosEncoder = 20;
}

void Grado45()
{
  NumeroDePulsosEncoder = 42;
}

void Grado90()
{
  NumeroDePulsosEncoder = 84;
}

void Grado135()
{
  NumeroDePulsosEncoder = 126;
}

void Grado180()
{
  NumeroDePulsosEncoder = 168;
}

void Grado225()
{
  NumeroDePulsosEncoder = 210;
}

void Grado270()
{
  NumeroDePulsosEncoder = 252;
}

void Grado315()
{
  NumeroDePulsosEncoder = 294;
}

void Grado360()
{
  NumeroDePulsosEncoder = 336;
}
/*
void Manual()
{
  Stop();
  contador1 = 0;
  analogWrite(PWM1, velocidad1);
  analogWrite(PWM2, velocidad2);
  uint16_t position = qtr.readLineBlack(sensorValues);

  for (uint8_t i = 0; i < SensorCount; i++)
  {
    Serial.print(sensorValues[i]);
    Serial.print('\t');
  }

  Serial.println(position);
  delay(100);

  while (contador1 < NumeroDePulsosEncoder && sensorValues[0] < 650 &&
         sensorValues[1] < 650 && sensorValues[2] < 650 &&
         sensorValues[3] < 650 && sensorValues[4] < 650 && sensorValues[5] < 650)
  {
  }
  Stop();
}

void Automatico()
{
  Serial1.write('j');
  contador1 = 0;
  NumeroDePulsosEncoder = 0;
  Adelante();
  servo1.write(0);
  analogWrite(PWM1, velocidad1);
  analogWrite(PWM2, velocidad2);
  uint16_t position = qtr.readLineBlack(sensorValues);

  for (uint8_t i = 0; i < SensorCount; i++)
  {
    Serial.print(sensorValues[i]);
    Serial.print('\t');
  }

  Serial.println(position);
  delay(100); // TENER EN CUENTA EL TEMPO PARA EL DISPARO DE LOS MOTORES

  if (sensorValues[0] < 650 && sensorValues[1] < 650 && sensorValues[2] < 650 &&
      sensorValues[3] < 650 && sensorValues[4] < 650 && sensorValues[5] < 650)
  {
    contador1 = 0;
    Adelante();
  }
  else if (sensorValues[0] < 650 && sensorValues[1] < 650 && sensorValues[2] < 650 &&
           sensorValues[3] < 650 && sensorValues[4] < 650 && sensorValues[5] > 650)
  {
    Stop();
    contador1 = 0;
    Grado20();
    Atras();
    while (contador1 < NumeroDePulsosEncoder)
    {
    }
    Stop();
    contador1 = 0;
    Grado90();
    Derecha();
    while (contador1 < NumeroDePulsosEncoder)
    {
    }
    Stop();
  }
  else if (sensorValues[0] < 650 && sensorValues[1] < 650 && sensorValues[2] < 650 &&
           sensorValues[3] < 650 && sensorValues[4] > 650 && sensorValues[5] > 650)
  {
    Stop();
    contador1 = 0;
    Grado20();
    Atras();
    while (contador1 < NumeroDePulsosEncoder)
    {
    }
    Stop();
    contador1 = 0;
    Grado90();
    Derecha();
    while (contador1 < NumeroDePulsosEncoder)
    {
    }
    Stop();
  }
  else if (sensorValues[0] < 650 && sensorValues[1] < 650 && sensorValues[2] < 650 &&
           sensorValues[3] > 650 && sensorValues[4] > 650 && sensorValues[5] > 650)
  {
    Stop();
    contador1 = 0;
    Grado20();
    Atras();
    while (contador1 < NumeroDePulsosEncoder)
    {
    }
    Stop();
    contador1 = 0;
    Grado90();
    Derecha();
    while (contador1 < NumeroDePulsosEncoder)
    {
    }
    Stop();
  }
  else if (sensorValues[0] < 650 && sensorValues[1] < 650 && sensorValues[2] > 650 &&
           sensorValues[3] > 650 && sensorValues[4] > 650 && sensorValues[5] > 650)
  {
    Stop();
    contador1 = 0;
    Grado20();
    Atras();
    while (contador1 < NumeroDePulsosEncoder)
    {
    }
    Stop();
    contador1 = 0;
    Grado90();
    Derecha();
    while (contador1 < NumeroDePulsosEncoder)
    {
    }
    Stop();
  }
  else if (sensorValues[0] < 650 && sensorValues[1] > 650 && sensorValues[2] > 650 &&
           sensorValues[3] > 650 && sensorValues[4] > 650 && sensorValues[5] > 650)
  {
    Stop();
    contador1 = 0;
    Grado20();
    Atras();
    while (contador1 < NumeroDePulsosEncoder)
    {
    }
    Stop();
    contador1 = 0;
    Grado90();
    Derecha();
    while (contador1 < NumeroDePulsosEncoder)
    {
    }
    Stop();
  }
  else if (sensorValues[0] > 650 && sensorValues[1] > 650 && sensorValues[2] > 650 &&
           sensorValues[3] > 650 && sensorValues[4] > 650 && sensorValues[5] > 650)
  {
    Stop();
    contador1 = 0;
    Grado20();
    Atras();
    while (contador1 < NumeroDePulsosEncoder)
    {
    }
    Stop();
    contador1 = 0;
    Grado135();
    Derecha();
    while (contador1 < NumeroDePulsosEncoder)
    {
    }
    Stop();
  }
  else if (sensorValues[0] > 650 && sensorValues[1] < 650 && sensorValues[2] < 650 &&
           sensorValues[3] < 650 && sensorValues[4] < 650 && sensorValues[5] < 650)
  {
    Stop();
    contador1 = 0;
    Grado20();
    Atras();
    while (contador1 < NumeroDePulsosEncoder)
    {
    }
    Stop();
    contador1 = 0;
    Grado90();
    Izquierda();
    while (contador1 < NumeroDePulsosEncoder)
    {
    }
    Stop();
  }
  else if (sensorValues[0] > 650 && sensorValues[1] > 650 && sensorValues[2] < 650 &&
           sensorValues[3] < 650 && sensorValues[4] < 650 && sensorValues[5] < 650)
  {
    Stop();
    contador1 = 0;
    Grado20();
    Atras();
    while (contador1 < NumeroDePulsosEncoder)
    {
    }
    Stop();
    contador1 = 0;
    Grado90();
    Izquierda();
    while (contador1 < NumeroDePulsosEncoder)
    {
    }
    Stop();
  }
  else if (sensorValues[0] > 650 && sensorValues[1] > 650 && sensorValues[2] > 650 &&
           sensorValues[3] < 650 && sensorValues[4] < 650 && sensorValues[5] < 650)
  {
    Stop();
    contador1 = 0;
    Grado20();
    Atras();
    while (contador1 < NumeroDePulsosEncoder)
    {
    }
    Stop();
    contador1 = 0;
    Grado90();
    Izquierda();
    while (contador1 < NumeroDePulsosEncoder)
    {
    }
    Stop();
  }
  else if (sensorValues[0] > 650 && sensorValues[1] > 650 && sensorValues[2] > 650 &&
           sensorValues[3] > 650 && sensorValues[4] < 650 && sensorValues[5] < 650)
  {
    Stop();
    contador1 = 0;
    Grado20();
    Atras();
    while (contador1 < NumeroDePulsosEncoder)
    {
    }
    Stop();
    contador1 = 0;
    Grado90();
    Izquierda();
    while (contador1 < NumeroDePulsosEncoder)
    {
    }
    Stop();
  }
  else if (sensorValues[0] > 650 && sensorValues[1] > 650 && sensorValues[2] > 650 &&
           sensorValues[3] > 650 && sensorValues[4] > 650 && sensorValues[5] < 650)
  {
    Stop();
    contador1 = 0;
    Grado20();
    Atras();
    while (contador1 < NumeroDePulsosEncoder)
    {
    }
    Stop();
    contador1 = 0;
    Grado90();
    Izquierda();
    while (contador1 < NumeroDePulsosEncoder)
    {
    }
    Stop();
  }
  else if (sensorValues[0] > 650 && sensorValues[1] < 650 && sensorValues[2] < 650 &&
           sensorValues[3] < 650 && sensorValues[4] < 650 && sensorValues[5] > 650)
  {
    Stop();
    contador1 = 0;
    Grado90();
    Atras();
    while (contador1 < NumeroDePulsosEncoder)
    {
    }
    Stop();
    contador1 = 0;
    Grado315();
    Derecha();
    while (contador1 < NumeroDePulsosEncoder)
    {
    }
    Stop();
  }
  else if (sensorValues[0] > 650 && sensorValues[1] > 650 && sensorValues[2] < 650 &&
           sensorValues[3] < 650 && sensorValues[4] > 650 && sensorValues[5] > 650)
  {
    Stop();
    contador1 = 0;
    Grado90();
    Atras();
    while (contador1 < NumeroDePulsosEncoder)
    {
    }
    Stop();
    contador1 = 0;
    Grado315();
    Derecha();
    while (contador1 < NumeroDePulsosEncoder)
    {
    }
    Stop();
  }
  else
  {
    Stop();
    contador1 = 0;
    Grado20();
    Atras();
    while (contador1 < NumeroDePulsosEncoder)
    {
    }
    Stop();
    contador1 = 0;
    Grado90();
    Derecha();
    while (contador1 < NumeroDePulsosEncoder)
    {
    }
    Stop();
  }

  digitalWrite(Trig, LOW);
  delayMicroseconds(4);
  digitalWrite(Trig, HIGH);
  delayMicroseconds(10);
  digitalWrite(Trig, LOW);
  duracion = pulseIn(Echo, HIGH);
  duracion = duracion / 2;
  distancia = duracion / 29;

  if (distancia <= 3)
  {
    Stop();
    servo1.write(0);
    delay(1000);
    servo1.write(33);
    getColor();
    if (countRed < countBlue && countGreen > 100 && countRed < 80)
    {
      digitalWrite(LedRojo, LOW);
      digitalWrite(LedVerde, HIGH);
      digitalWrite(LedAzul, HIGH);
      Serial1.write('f');
    }
    else if (countBlue < countRed && countBlue < countGreen)
    {
      digitalWrite(LedRojo, HIGH);
      digitalWrite(LedVerde, HIGH);
      digitalWrite(LedAzul, LOW);
      Serial1.write('h');
    }
    else if (countGreen < countRed && countGreen < countBlue)
    {
      digitalWrite(LedRojo, HIGH);
      digitalWrite(LedVerde, LOW);
      digitalWrite(LedAzul, HIGH);
      Serial1.write('g');
    }
    else
    {
      digitalWrite(LedRojo, HIGH);
      digitalWrite(LedVerde, HIGH);
      digitalWrite(LedAzul, HIGH);
    }
    delay(3000);
    servo1.write(0);
    Serial1.write('j');
    digitalWrite(LedRojo, HIGH);
    digitalWrite(LedVerde, HIGH);
    digitalWrite(LedAzul, HIGH);
    Stop();
    contador1 = 0;
    Grado20();
    Atras();
    while (contador1 < NumeroDePulsosEncoder)
    {
    }
    Stop();
    contador1 = 0;
    Grado90();
    Izquierda();
    while (contador1 < NumeroDePulsosEncoder)
    {
    }
    Stop();
    contador1 = 0;
  }
  Serial1.write('j');
}

void loop()
{
  if (Serial1.available() > 0)
  {
    char dato = Serial1.read();
    if (dato == 'i')
    {
      Automatico();
    }

    else if (dato == 'u')
    {
      Stop();
    }

    else if (dato == 'z')
    {
      Grado45();
    }

    else if (dato == 'x')
    {
      Grado90();
    }

    else if (dato == 'c')
    {
      Grado135();
    }

    else if (dato == 'v')
    {
      Grado180();
    }

    else if (dato == 'b')
    {
      Grado225();
    }

    else if (dato == 'n')
    {
      Grado270();
    }

    else if (dato == 'm')
    {
      Grado315();
    }

    else if (dato == 'l')
    {
      Grado360();
    }

    else if (dato == 'o')
    {
      servo1.write(0);
    }

    else if (dato == 'p')
    {
      servo1.write(33);
      getColor();
      if (countRed < countBlue && countGreen > 100 && countRed < 80)
      {
        digitalWrite(LedRojo, LOW);
        digitalWrite(LedVerde, HIGH);
        digitalWrite(LedAzul, HIGH);
        Serial1.write('f');
      }
      else if (countBlue < countRed && countBlue < countGreen)
      {
        digitalWrite(LedRojo, HIGH);
        digitalWrite(LedVerde, HIGH);
        digitalWrite(LedAzul, LOW);
        Serial1.write('h');
      }
      else if (countGreen < countRed && countGreen < countBlue)
      {
        digitalWrite(LedRojo, HIGH);
        digitalWrite(LedVerde, LOW);
        digitalWrite(LedAzul, HIGH);
        Serial1.write('g');
      }
      else
      {
        digitalWrite(LedRojo, HIGH);
        digitalWrite(LedVerde, HIGH);
        digitalWrite(LedAzul, HIGH);
      }
      Serial1.write('j');
      digitalWrite(LedRojo, HIGH);
      digitalWrite(LedVerde, HIGH);
      digitalWrite(LedAzul, HIGH);
      contador1 = 0;
    }

    else if (dato == 'w')
    {
      Adelante();
      Manual();
    }

    else if (dato == 's')
    {
      Atras();
      Manual();
    }

    else if (dato == 'd')
    {
      Derecha();
      Manual();
    }

    else if (dato == 'a')
    {
      Izquierda();
      Manual();
    }
    
  }
}
*/
void loop()
{
  Serial1.write('j');
  contador1 = 0;
  NumeroDePulsosEncoder = 0;
  Adelante();
  servo1.write(0);
  analogWrite(PWM1, velocidad1);
  analogWrite(PWM2, velocidad2);
  uint16_t position = qtr.readLineBlack(sensorValues);

  for (uint8_t i = 0; i < SensorCount; i++)
  {
    Serial.print(sensorValues[i]);
    Serial.print('\t');
  }

  Serial.println(position);
  delay(100); // TENER EN CUENTA EL TEMPO PARA EL DISPARO DE LOS MOTORES

  if (sensorValues[0] < 650 && sensorValues[1] < 650 && sensorValues[2] < 650 &&
      sensorValues[3] < 650 && sensorValues[4] < 650 && sensorValues[5] < 650)
  {
    contador1 = 0;
    Adelante();
  }
  else if (sensorValues[0] < 650 && sensorValues[1] < 650 && sensorValues[2] < 650 &&
           sensorValues[3] < 650 && sensorValues[4] < 650 && sensorValues[5] > 650)
  {
    Stop();
    contador1 = 0;
    Grado20();
    Atras();
    while (contador1 < NumeroDePulsosEncoder)
    {
    }
    Stop();
    contador1 = 0;
    Grado90();
    Derecha();
    while (contador1 < NumeroDePulsosEncoder)
    {
    }
    Stop();
  }
  else if (sensorValues[0] < 650 && sensorValues[1] < 650 && sensorValues[2] < 650 &&
           sensorValues[3] < 650 && sensorValues[4] > 650 && sensorValues[5] > 650)
  {
    Stop();
    contador1 = 0;
    Grado20();
    Atras();
    while (contador1 < NumeroDePulsosEncoder)
    {
    }
    Stop();
    contador1 = 0;
    Grado90();
    Derecha();
    while (contador1 < NumeroDePulsosEncoder)
    {
    }
    Stop();
  }
  else if (sensorValues[0] < 650 && sensorValues[1] < 650 && sensorValues[2] < 650 &&
           sensorValues[3] > 650 && sensorValues[4] > 650 && sensorValues[5] > 650)
  {
    Stop();
    contador1 = 0;
    Grado20();
    Atras();
    while (contador1 < NumeroDePulsosEncoder)
    {
    }
    Stop();
    contador1 = 0;
    Grado90();
    Derecha();
    while (contador1 < NumeroDePulsosEncoder)
    {
    }
    Stop();
  }
  else if (sensorValues[0] < 650 && sensorValues[1] < 650 && sensorValues[2] > 650 &&
           sensorValues[3] > 650 && sensorValues[4] > 650 && sensorValues[5] > 650)
  {
    Stop();
    contador1 = 0;
    Grado20();
    Atras();
    while (contador1 < NumeroDePulsosEncoder)
    {
    }
    Stop();
    contador1 = 0;
    Grado90();
    Derecha();
    while (contador1 < NumeroDePulsosEncoder)
    {
    }
    Stop();
  }
  else if (sensorValues[0] < 650 && sensorValues[1] > 650 && sensorValues[2] > 650 &&
           sensorValues[3] > 650 && sensorValues[4] > 650 && sensorValues[5] > 650)
  {
    Stop();
    contador1 = 0;
    Grado20();
    Atras();
    while (contador1 < NumeroDePulsosEncoder)
    {
    }
    Stop();
    contador1 = 0;
    Grado90();
    Derecha();
    while (contador1 < NumeroDePulsosEncoder)
    {
    }
    Stop();
  }
  else if (sensorValues[0] > 650 && sensorValues[1] > 650 && sensorValues[2] > 650 &&
           sensorValues[3] > 650 && sensorValues[4] > 650 && sensorValues[5] > 650)
  {
    Stop();
    contador1 = 0;
    Grado20();
    Atras();
    while (contador1 < NumeroDePulsosEncoder)
    {
    }
    Stop();
    contador1 = 0;
    Grado135();
    Derecha();
    while (contador1 < NumeroDePulsosEncoder)
    {
    }
    Stop();
  }
  else if (sensorValues[0] > 650 && sensorValues[1] < 650 && sensorValues[2] < 650 &&
           sensorValues[3] < 650 && sensorValues[4] < 650 && sensorValues[5] < 650)
  {
    Stop();
    contador1 = 0;
    Grado20();
    Atras();
    while (contador1 < NumeroDePulsosEncoder)
    {
    }
    Stop();
    contador1 = 0;
    Grado90();
    Izquierda();
    while (contador1 < NumeroDePulsosEncoder)
    {
    }
    Stop();
  }
  else if (sensorValues[0] > 650 && sensorValues[1] > 650 && sensorValues[2] < 650 &&
           sensorValues[3] < 650 && sensorValues[4] < 650 && sensorValues[5] < 650)
  {
    Stop();
    contador1 = 0;
    Grado20();
    Atras();
    while (contador1 < NumeroDePulsosEncoder)
    {
    }
    Stop();
    contador1 = 0;
    Grado90();
    Izquierda();
    while (contador1 < NumeroDePulsosEncoder)
    {
    }
    Stop();
  }
  else if (sensorValues[0] > 650 && sensorValues[1] > 650 && sensorValues[2] > 650 &&
           sensorValues[3] < 650 && sensorValues[4] < 650 && sensorValues[5] < 650)
  {
    Stop();
    contador1 = 0;
    Grado20();
    Atras();
    while (contador1 < NumeroDePulsosEncoder)
    {
    }
    Stop();
    contador1 = 0;
    Grado90();
    Izquierda();
    while (contador1 < NumeroDePulsosEncoder)
    {
    }
    Stop();
  }
  else if (sensorValues[0] > 650 && sensorValues[1] > 650 && sensorValues[2] > 650 &&
           sensorValues[3] > 650 && sensorValues[4] < 650 && sensorValues[5] < 650)
  {
    Stop();
    contador1 = 0;
    Grado20();
    Atras();
    while (contador1 < NumeroDePulsosEncoder)
    {
    }
    Stop();
    contador1 = 0;
    Grado90();
    Izquierda();
    while (contador1 < NumeroDePulsosEncoder)
    {
    }
    Stop();
  }
  else if (sensorValues[0] > 650 && sensorValues[1] > 650 && sensorValues[2] > 650 &&
           sensorValues[3] > 650 && sensorValues[4] > 650 && sensorValues[5] < 650)
  {
    Stop();
    contador1 = 0;
    Grado20();
    Atras();
    while (contador1 < NumeroDePulsosEncoder)
    {
    }
    Stop();
    contador1 = 0;
    Grado90();
    Izquierda();
    while (contador1 < NumeroDePulsosEncoder)
    {
    }
    Stop();
  }
  else if (sensorValues[0] > 650 && sensorValues[1] < 650 && sensorValues[2] < 650 &&
           sensorValues[3] < 650 && sensorValues[4] < 650 && sensorValues[5] > 650)
  {
    Stop();
    contador1 = 0;
    Grado90();
    Atras();
    while (contador1 < NumeroDePulsosEncoder)
    {
    }
    Stop();
    contador1 = 0;
    Grado315();
    Derecha();
    while (contador1 < NumeroDePulsosEncoder)
    {
    }
    Stop();
  }
  else if (sensorValues[0] > 650 && sensorValues[1] > 650 && sensorValues[2] < 650 &&
           sensorValues[3] < 650 && sensorValues[4] > 650 && sensorValues[5] > 650)
  {
    Stop();
    contador1 = 0;
    Grado90();
    Atras();
    while (contador1 < NumeroDePulsosEncoder)
    {
    }
    Stop();
    contador1 = 0;
    Grado315();
    Derecha();
    while (contador1 < NumeroDePulsosEncoder)
    {
    }
    Stop();
  }
  else
  {
    Stop();
    contador1 = 0;
    Grado20();
    Atras();
    while (contador1 < NumeroDePulsosEncoder)
    {
    }
    Stop();
    contador1 = 0;
    Grado90();
    Derecha();
    while (contador1 < NumeroDePulsosEncoder)
    {
    }
    Stop();
  }

  digitalWrite(Trig, LOW);
  delayMicroseconds(4);
  digitalWrite(Trig, HIGH);
  delayMicroseconds(10);
  digitalWrite(Trig, LOW);
  duracion = pulseIn(Echo, HIGH);
  duracion = duracion / 2;
  distancia = duracion / 29;

  if (distancia <= 3)
  {
    Stop();
    servo1.write(0);
    delay(1000);
    servo1.write(33);
    getColor();
    if (countRed < countBlue && countGreen > 100 && countRed < 80)
    {
      digitalWrite(LedRojo, LOW);
      digitalWrite(LedVerde, HIGH);
      digitalWrite(LedAzul, HIGH);
      Serial1.write('f');
    }
    else if (countBlue < countRed && countBlue < countGreen)
    {
      digitalWrite(LedRojo, HIGH);
      digitalWrite(LedVerde, HIGH);
      digitalWrite(LedAzul, LOW);
      Serial1.write('h');
    }
    else if (countGreen < countRed && countGreen < countBlue)
    {
      digitalWrite(LedRojo, HIGH);
      digitalWrite(LedVerde, LOW);
      digitalWrite(LedAzul, HIGH);
      Serial1.write('g');
    }
    else
    {
      digitalWrite(LedRojo, HIGH);
      digitalWrite(LedVerde, HIGH);
      digitalWrite(LedAzul, HIGH);
    }
    delay(3000);
    servo1.write(0);
    Serial1.write('j');
    digitalWrite(LedRojo, HIGH);
    digitalWrite(LedVerde, HIGH);
    digitalWrite(LedAzul, HIGH);
    Stop();
    contador1 = 0;
    Grado20();
    Atras();
    while (contador1 < NumeroDePulsosEncoder)
    {
    }
    Stop();
    contador1 = 0;
    Grado90();
    Izquierda();
    while (contador1 < NumeroDePulsosEncoder)
    {
    }
    Stop();
    contador1 = 0;
  }
  Serial1.write('j');
}