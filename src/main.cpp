#include <Arduino.h>
#include "MeOrion.h"
#include <Wire.h>
#include <Adafruit_INA219.h>
#include <Adafruit_ADS1X15.h>

//Main paramters:

const uint8_t number_Stages = 4; // HAVE TO BE ALWAYS "PAIR"
const uint8_t Number_Steps = 100;
const uint8_t Number_Measures = 25;
uint8_t Angle_Step = 20;

const unsigned long Number_Seconds_Between_Scan = 60; // KEEP unsigned long!!
const uint16_t Delay_Chosen = 1000;

uint16_t Scan_Speed=100;//microsecond but much longer due to acquisition process
uint16_t Normal_Speed= 6000; //2000 looks a bit to fast for the motor.
uint16_t InBetween_Speed= 1000;

// magnetometer

// MeCompass Compass(PORT_4);

//ads1115

Adafruit_ADS1115 ads1115;

// int16_t adc0, adc1, adc2, adc3;

// float voltage_0, voltage_1, voltage_2, voltage_3;

// servo

MePort port(PORT_3);
Servo myservo1;                  // create servo object to control a servo
int16_t servo1pin = port.pin2(); //attaches the servo on PORT_3 SLOT 2 ATTENTION slot 2 Guillaume!!! to the servo object

uint8_t Angles_Servo[number_Stages];


// stepper parameter

uint8_t Direction = 1;

int dirPin = mePort[PORT_1].s1; //the direction pin connect to Base Board PORT1 SLOT1
int stpPin = mePort[PORT_1].s2; //the Step pin connect to Base Board PORT1 SLOT2

// ina219 parameter:

Adafruit_INA219 ina219;

unsigned long myWantedTime;
float voltage_V = 0;
float current_mA = 0;

// paramètres hardcore

float my_array_Power[Number_Measures];
uint16_t my_Photoresistor_1[Number_Measures];
uint16_t my_Photoresistor_2[Number_Measures];
uint16_t my_Photoresistor_3[Number_Measures];
uint16_t my_Photoresistor_4[Number_Measures];

byte maxIndex[number_Stages];
float maxValuePower[number_Stages];

float MaxMaxValuePower;
byte MaxMaxIndices;
byte Search_Best_Step;
byte Search_Best_Servo;

// outliers detection

// uint8_t Treshold = 10;

// uint8_t Number_Correction = 0;

// GEstion merdique du demarrage depuis raspi

uint8_t Integer_1 = 0;
uint8_t Integer_2 = 0;

uint16_t iteration = 0;

unsigned long Tic;
unsigned long Toc;
unsigned long Toc_Tic;

//////////////////////////FUNCTIONS//////////////////////////////////

void step_2()
{
  digitalWrite(dirPin, Direction);
  delay(50);
  for (int i = 0; i < Search_Best_Step; i++)
  {
    digitalWrite(stpPin, HIGH);
    delayMicroseconds(Normal_Speed);
    digitalWrite(stpPin, LOW);
    delayMicroseconds(Normal_Speed);
  }
}

void Go_to_Optimum()
{
  myservo1.write(Search_Best_Servo);
  delay(InBetween_Speed);
  step_2();
  delay(InBetween_Speed);
}

void Go_to_Home()
{
  Direction = !Direction; //
  myservo1.write(Angles_Servo[0]);
  delay(InBetween_Speed);
  step_2();
  delay(InBetween_Speed);
  Direction = !Direction;
}

void step(byte Stages)
{
  digitalWrite(dirPin, Direction);
  delay(50);

  byte Indice_Measure = 0; 

  for (int i = 0; i < Number_Steps; i++)
  {
    digitalWrite(stpPin, HIGH);
    delayMicroseconds(Scan_Speed); // this delay maybe adjusted for sound.
  
    if (i % (Number_Steps / Number_Measures) == 0)
    {
      
      Tic=millis();
      current_mA = fabs(ina219.getCurrent_mA());
      voltage_V = ina219.getBusVoltage_V();

      my_array_Power[Indice_Measure] = fabs(voltage_V * current_mA); // fabs maybe become useless

      my_Photoresistor_1[Indice_Measure] = ads1115.readADC_SingleEnded(0);
      my_Photoresistor_2[Indice_Measure] = ads1115.readADC_SingleEnded(1);
      my_Photoresistor_3[Indice_Measure] = ads1115.readADC_SingleEnded(2);
      my_Photoresistor_4[Indice_Measure] = ads1115.readADC_SingleEnded(3);

      Indice_Measure++;
      Toc=millis();
    
    }
    else
    {
      // Serial.println(i);
      // Serial.println(Toc-Tic);
      delay(Toc-Tic);
    }
    digitalWrite(stpPin, LOW);
    delayMicroseconds(Scan_Speed);
  }

  for (byte i = 0; i < Number_Measures; i++)
  {

    Serial.print("S");
    Serial.print(",");
    Serial.print(i);
    Serial.print(",");
    Serial.print(my_array_Power[i]);
    Serial.print(","); // I print 3 times to avoid to get char when parsing the table with matlab.
    Serial.print(my_Photoresistor_1[i]);
    Serial.print(",");
    Serial.print(my_Photoresistor_2[i]);
    Serial.print(",");
    Serial.print(my_Photoresistor_3[i]);
    Serial.print(","); // I print 3 times to avoid to get char when parsing the table with matlab.
    Serial.print(my_Photoresistor_4[i]);
    Serial.print(",");
    Serial.print(0);
    Serial.print(",");
    Serial.println(0);
    // Serial.print(",");
    // Serial.print(0);
    // Serial.print(",");
    // Serial.print(0);
    // Serial.print(",");
    // Serial.println(0);
  }

  maxValuePower[Stages] = my_array_Power[Number_Measures - 1]; // take the last as reference for algo below (steps means 100) but index begin at 0 thus 99

  // j'ai un doute ici, peut-être faut il inclure la correction plus bas avec le modulo

  maxIndex[Stages] = Number_Measures - 1;

  for (byte i = 0; i < Number_Measures; i++)
  {
    if (my_array_Power[i] > maxValuePower[Stages])
    {
      maxValuePower[Stages] = my_array_Power[i];
      if ((Stages % 2) == 0) // si pair, alors on est sur l'allée (indexation à zero)
      {
          // pay attention here, the equation may be false or not perfectly right
          // the goal is to recover back the real step by multypling by the value
          // used in the modulo (100/25=4)
        maxIndex[Stages] = i * Number_Steps / Number_Measures;
      }
      else
      {
        // je pense que c'est OK: 100-0=100
        // pay attention here, the equation may be false or not perfectly right
        maxIndex[Stages] = (Number_Measures - i) * Number_Steps / Number_Measures; // si impair, on est sur le retour et il faut soustraire pour avoir l'index du pas dans le bon sens
      }
    }
  }
}

void scan()
{

  for (byte i = 0; i < number_Stages; i++)
  {

    myservo1.write(Angles_Servo[i]);
    delay(InBetween_Speed);
    step(i); // "i" send the current number of stage to the step function
    delay(InBetween_Speed);
    Direction = !Direction;
  }

  MaxMaxValuePower = maxValuePower[number_Stages - 1]; // affecte la dernière valeur à MaxMaxValuePower pour pouvoir faire la comparaison dans la boucle qui suit.
  MaxMaxIndices = number_Stages - 1;
  Search_Best_Step = maxIndex[number_Stages - 1];
  Search_Best_Servo = Angles_Servo[number_Stages - 1];

  for (byte i = 0; i < number_Stages; i++)
  {

    if (maxValuePower[i] > MaxMaxValuePower)
    {
      MaxMaxValuePower = maxValuePower[i];
      MaxMaxIndices = i;
      Search_Best_Step = maxIndex[MaxMaxIndices];      //ouch (the correct number of step to go)
      Search_Best_Servo = Angles_Servo[MaxMaxIndices]; // the correct angle to go again
    }
  }

  Serial.print("R");
  Serial.print(",");
  Serial.print(MaxMaxValuePower, 2);
  Serial.print(",");
  Serial.print(Search_Best_Step);
  Serial.print(",");
  Serial.print(Search_Best_Servo);
  Serial.print(",");
  Serial.print(0);
  Serial.print(",");
  Serial.print(0);
  Serial.print(",");
  Serial.print(0);
  Serial.print(",");
  Serial.print(0);
  Serial.print(",");
  Serial.println(0);
  // Serial.print(",");
  // Serial.print(0);
  // Serial.print(",");
  // Serial.print(0);
  // Serial.print(",");
  // Serial.println(0);

  Go_to_Optimum(); // on rejoint la meilleur position: c'est la fin de la fonction scan.
}

void setup()
{
  myservo1.attach(servo1pin); // attaches the servo on servopin1

  pinMode(dirPin, OUTPUT);
  pinMode(stpPin, OUTPUT);

  Serial.begin(9600);

  if (!ina219.begin())
  {
    Serial.println("Failed to find INA219 chip");
    while (1)
    {
      delay(10);
    }
  }

  //magneto
  //Compass.begin();

  //ads1115

  ads1115.begin();

  // rempli un vecteur nommé Angles_Servo avec tous les angles qui seront utilisés

  Angles_Servo[0] = 170;
  for (byte i = 1; i < number_Stages; i++)
  {
    Angles_Servo[i] = Angles_Servo[i - 1] - Angle_Step;
  }

  myservo1.write(Angles_Servo[0]);
}

void loop()
{

  // python envoie 1 pour lancer le programme et 0 pour l'arrêter (si taille fichier trop grand ou si ctrl +C). Il y a une boucle dans la gestion de keyboardinterrupt au cas
  // où le ctrl + C ne se passe pendant le scan. Le but est d'être sûr que le 0 envoyé arrive à bon port.

  if (Serial.available() > 0)
  {
    // la gestion du passage d'un entier 1 de python à arduino est un cauchemar. La solution.
    // la solution: déja dans python je précise qu'il s'agit d'u byte avec un petit b: ser.write(b"1")

    // dans arduino, j'utilise Serial.readString et non pas Serial.read.  ensuite je convertis ce string en entier en faisant Launch.toInt()
    // on gardera en tête que j'utilise la library String avec un grand S de arduino sans doute un peu pourri.
    //puis avec le if, si mon Integer_1 vaut 1, Integer_2 vaut 1. Le passage par une deuxième variable n'est pas clair.
    // si j'utilise simplement Integer_1 dans le if d'aprés, il refuse de le faire passer à 1 et reste initialiser à zero.
    // bre un merdier sans nom.

    String Launch = Serial.readString(); // reçoit un byte de pyhon et considéré comme String

    uint8_t Integer_1 = Launch.toInt(); // transformé en entier

    if (Integer_1 == 1)
    {

      Integer_2 = 1; // utilisation d'une deuxième variable necessaire sans que je comprenne pourquoi
    }
    else if (Integer_1 == 0)
    {

      Integer_2 = 0; // utilisation d'une deuxième variable necessaire sans que je comprenne pourquoi

      Serial.println("Motors Stopped");
    }
  }

  if (Integer_2 == 1) // voir note ci dessous; necessaire de changer de nom de variable pour rentrer dans la boucle .
  {
    if (iteration == 0)
    {
      delay(10000);
    }

    myWantedTime = millis();

    if (myWantedTime >= (Number_Seconds_Between_Scan * 1000 * iteration))
    {

      if (iteration > 0) // not for the first iteration obviously
      {
        Go_to_Home();
      }

      scan();
      iteration++; // compte le nombre de scan;
    }

    Serial.print("K");
    Serial.print(",");
    Serial.print(myWantedTime);
    Serial.print(",");
    Serial.print(ina219.getBusVoltage_V());
    Serial.print(","); // I divide by 50 in place of 100 because there is a divider bridge to protect A3
    Serial.print(fabs(ina219.getCurrent_mA()));
    Serial.print(",");
    Serial.print(fabs((ina219.getBusVoltage_V() * ina219.getCurrent_mA())));
    Serial.print(","); // favs maybe useless

    Serial.print(ads1115.readADC_SingleEnded(0));
    Serial.print(",");
    Serial.print(ads1115.readADC_SingleEnded(1));
    Serial.print(",");
    Serial.print(ads1115.readADC_SingleEnded(2));
    Serial.print(",");
    Serial.println(ads1115.readADC_SingleEnded(3));

    //Serial.print(",");
    // Serial.print(Compass.getHeadingX());
    // Serial.print(",");
    // Serial.print(Compass.getHeadingY());
    // Serial.print(",");
    // Serial.println(Compass.getHeadingZ()); // Serial.print(",");

    delay(Delay_Chosen);
  }
}