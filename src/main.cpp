#include <Arduino.h>
#include "MeOrion.h"
#include <Wire.h>
#include <Adafruit_INA219.h>
#include <Adafruit_ADS1X15.h>


//Main paramters:

const uint8_t  number_Stages = 2; // HAVE TO BE ALWAYS "PAIR"

unsigned long Number_Seconds_Between_Scan = 15;

unsigned long Delay_Chosen = 1000;

// magnetometer

MeCompass Compass(PORT_4);

//ads1115

Adafruit_ADS1115 ads1115;

int16_t adc0, adc1, adc2, adc3;

float voltage_0, voltage_1, voltage_2, voltage_3;

// servo

MePort port(PORT_3);
Servo myservo1;  // create servo object to control a servo
int16_t servo1pin =  port.pin2();//attaches the servo on PORT_3 SLOT 2 ATTENTION slot 2 Guillaume!!! to the servo object

uint8_t Angles_Servo[number_Stages]; 
uint8_t Angle_Step = 10;

// stepper parameter

uint8_t Direction = 1;
const uint8_t  NumberStep = 100;

int dirPin = mePort[PORT_1].s1;//the direction pin connect to Base Board PORT1 SLOT1
int stpPin = mePort[PORT_1].s2;//the Step pin connect to Base Board PORT1 SLOT2

// ina219 parameter:

Adafruit_INA219 ina219;

unsigned long myWantedTime;
float voltage_V = 0;
float current_mA = 0;

// paramètres hardcore

float my_array_Power[NumberStep];

byte maxIndex[number_Stages];
float maxValuePower[number_Stages] ;


float MaxMaxValuePower;
byte MaxMaxIndices;
byte Search_Best_Step;
byte Search_Best_Servo;

// outliers detection

// uint8_t Treshold = 10;

// uint8_t Number_Correction = 0;

// GEstion merdique du demarrage depuis raspi

 int Integer_2 = 1;

int iteration = 0;


void setup()
{
  myservo1.attach(servo1pin);  // attaches the servo on servopin1

  pinMode(dirPin, OUTPUT);
  pinMode(stpPin, OUTPUT);

  Serial.begin(9600);


  if (! ina219.begin()) {
    Serial.println("Failed to find INA219 chip");
    while (1) {
      delay(10);
    }
  }

    //magneto
  Compass.begin();

  //ads1115

  ads1115.begin();

  // rempli un vecteur nommé Angles_Servo avec tous les angles qui seront utilisés

  Angles_Servo[0] = 170;
  for (byte i = 1; i < number_Stages; i++)
  {
    Angles_Servo[i] = Angles_Servo[i - 1] - Angle_Step;
  }

for (byte i = 0; i < number_Stages; i++)
  {
   Serial.println(Angles_Servo[i]);
  }
 
myservo1.write(Angles_Servo[0]);
}


void scan() {

  for (byte i = 0 ; i < number_Stages; i++)
  {
    myservo1.write(Angles_Servo[i]);
    delay(2000);
    step(Direction, NumberStep, i); // "i" send the current number of stage to the step function
    delay(2000);
    Direction = !Direction;
  }

  // L'objectif ést de récupérér dans un vecteur contenant tous les maximums: THE maximum.
  // on commence par mettre MaxMaxValuePower à la valeur final du vecteur et on part de zero, pour avec la boucle, lui attribué la vrai valeur max.
  // on récupére en même temps l'indice (MaxMaxIndices) qui permet de retrouver le meilleur "pas" pour le stepper stocké dans le vecteur maxIndex
  // de même l'incide MaxMaxIndices est la meilleur position dans le vecteur Angles_Servo (à vérifier).

  // Attention à l'utilisation de number_Stages et de Number_Steps ou steps comme indice maximum car ils existent car initialisé mais jamais rempli (index commence à zero).


  MaxMaxValuePower = maxValuePower[number_Stages - 1]; // affecte la dernière valeur à MaxMaxValuePower pour pouvoir faire la comparaison dans la boucle qui suit.
  MaxMaxIndices = number_Stages - 1;
  Search_Best_Step = maxIndex[number_Stages - 1];
  Search_Best_Servo = Angles_Servo[number_Stages - 1];

//  Serial.print("P"); Serial.print(",");

  for (byte i = 0 ; i < number_Stages; i++)
  {

// "P" means preliminart results of scan


//  Serial.print(maxValuePower[i]); Serial.print(",");

    if ( maxValuePower[i] > MaxMaxValuePower) {
      MaxMaxValuePower = maxValuePower[i];
      MaxMaxIndices = i;
      Search_Best_Step = maxIndex[MaxMaxIndices]; //ouch (the correct number of step to go)
      Search_Best_Servo = Angles_Servo[MaxMaxIndices]; // the correct angle to go again
    }

  }

//Serial.println();

  

  // il y a forcement une erreur dans les lignes qui suivent.

  //  Serial.println(maxValuePower );
  //  Serial.println(MaxIndex);

  // "R" means results of scan
  Serial.print("R"); Serial.print(",");
  Serial.print(MaxMaxValuePower, 2); Serial.print(",");
  Serial.print(Search_Best_Step); Serial.print(",");
  Serial.print(Search_Best_Servo); Serial.print(",");
  // Serial.println(Number_Correction);

  Go_to_Optimum(); // on rejoint la meilleur position: c'est la fin de la fonction scan.


}

// rappel: un byte c'est 8 bit donc, dans notre cas, un entier inférieur à 256. correspond, je crois, à uint8_t


void Go_to_Optimum()
{

  myservo1.write(Search_Best_Servo);
  delay(2000);
  step_2();
  delay(2000);

}

void Go_to_Home()
{
        Direction = !Direction; //
        myservo1.write(Angles_Servo[0]);
        delay(2000);
        step_2();
        delay(2000);
        Direction = !Direction;
}

// this function will keep in an array all the value of power during à single line scan.
// then, it process data to remove outliers and send the maximum of a line in the vector maxValuePower process further by the scan function.

void step(byte Stages)
{
  digitalWrite(dirPin, dir);
  delay(50);
  for (int i = 0; i < steps; i++)
  {
    digitalWrite(stpPin, HIGH);
    delayMicroseconds(2000);// this delay maybe adjusted for sound.

    current_mA = ina219.getCurrent_mA();
    voltage_V = ina219.getBusVoltage_V();

    my_array_Power[i] = fabs(voltage_V * current_mA); // fabs maybe become useless
    digitalWrite(stpPin, LOW);
    delayMicroseconds(2000);
  }


  // correction for outliers based on matlab test using mesh showing example of errors

  // this outliers detection has to be verified : maybe mistake, maybe useless

  // not sure about me but i will change i=1 to i=2 and steps-1 to steps-2

  // for (byte i = 2; i < steps - 2; i++)
  // {

  //   if ((my_array_Power[i] - my_array_Power[i - 1] > Treshold) || (my_array_Power[i] - my_array_Power[i - 1] < - Treshold) )
  //   {


  //     // "O" means outlier: on indique la step (i), la valeur aberrante avant correction et aprés correction.
  //     Serial.print("O"); Serial.print(",");
  //     Serial.print(i); Serial.print(",");
  //     Serial.print( my_array_Power[i] ); Serial.print(",");
  //     my_array_Power[i] = (my_array_Power[i - 2] + my_array_Power[i + 2]) / 2;
  //     Number_Correction ++;
  //     Serial.println( my_array_Power[i] );

  //   }

    //    if (Corrected_Array_Power[i - 1] - my_array_Power[i] > Treshold)
    //    {
    //
    //      Corrected_Array_Power[i] = (my_array_Power[i - 2] + my_array_Power[i - 2]) / 2;
    //
    //    }

  }


  // "S" means scan result

  for (byte i = 0; i < steps ; i++)
  {

    Serial.print("S"); Serial.print(",");
    Serial.print( i ); Serial.print(",");
    Serial.print(my_array_Power[i]); Serial.print(","); // I print 3 times to avoid to get char when parsing the table with matlab.
    Serial.print(my_array_Power[i]); Serial.print(",");
    Serial.println(my_array_Power[i]);

  }

  maxValuePower[Stages] = my_array_Power[steps - 1]; // take the last as reference for algo below (steps means 100) but index begin at 0 thus 99
  
  // j'ai un doute ici, peut-être faut il inclure la correction plus bas avec le modulo
  maxIndex[Stages]=steps - 1;


  for (byte i = 0; i < steps; i++)
  {
    if ( my_array_Power[i] > maxValuePower[Stages]) {
      maxValuePower[Stages] = my_array_Power[i];
      if ( (Stages % 2) == 0) // si pair, alors on est sur l'allée (indexation à zero)
      {

        maxIndex[Stages] = i;

      }
      else
      {
        // je pense que c'est OK: 100-0=100
        maxIndex[Stages] = steps - i; // si impair, on est sur le retour et il faut soustraire pour avoir l'index du pas dans le bon sens
      }

    }
  }

}

// faire tounrner le panneau sans acquition.

// j'ai remplacer steps par Search_Best_Step: à verifier.

void step_2()
{
  digitalWrite(dirPin, dir);
  delay(50);
  for (int i = 0; i <  Search_Best_Step; i++)
  {
    digitalWrite(stpPin, HIGH);
    delayMicroseconds(2000);
    digitalWrite(stpPin, LOW);
    delayMicroseconds(2000);
  }

}

void loop()
{


  // python envoie 1 pour lancer le programme et 0 pour l'arrêter (si taille fichier trop grand ou si ctrl +C). Il y a une boucle dans la gestion de keyboardinterrupt au cas
  // où le ctrl + C ne se passe pendant le scan. Le but est d'être sûr que le 0 envoyé arrive à bon port.

//  if (Serial.available() > 0) {
//    // la gestion du passage d'un entier 1 de python à arduino est un cauchemar. La solution.
//    // la solution: déja dans python je précise qu'il s'agit d'u byte avec un petit b: ser.write(b"1")
//
//    // dans arduino, j'utilise Serial.readString et non pas Serial.read.  ensuite je convertis ce string en entier en faisant Launch.toInt()
//    // on gardera en tête que j'utilise la library String avec un grand S de arduino sans doute un peu pourri.
//    //puis avec le if, si mon Integer_1 vaut 1, Integer_2 vaut 1. Le passage par une deuxième variable n'est pas clair.
//    // si j'utilise simplement Integer_1 dans le if d'aprés, il refuse de le faire passer à 1 et reste initialiser à zero.
//    // bre un merdier sans nom.
//
//    String Launch = Serial.readString(); // reçoit un byte de pyhon et considéré comme String
//
//    int Integer_1 = Launch.toInt();// transformé en entier
//
//    if (Integer_1 == 1) {
//
//      Integer_2 = 1;// utilisation d'une deuxième variable necessaire sans que je comprenne pourquoi
//
//    }
//    else if (Integer_1 == 0)
//    {
//
//      Integer_2 = 0;// utilisation d'une deuxième variable necessaire sans que je comprenne pourquoi
//
//      Serial.println("Motors Stopped");
//    }
//  }

Serial.print(Integer_2);Serial.print(",");
Serial.print(iteration);Serial.print(",");

  if (Integer_2 == 1) // voir note ci dessous; necessaire de changer de nom de variable pour rentrer dans la boucle .
  {


    myWantedTime = millis();
    Serial.print(myWantedTime);Serial.print(",");
    Serial.print(Number_Seconds_Between_Scan * 1000 * iteration);Serial.print(",");
    Serial.println("test");
    if (myWantedTime >= (Number_Seconds_Between_Scan * 1000 * iteration))
    {


      // explication de ce "if", aprés s'être arrêté à une position optimale (donc iteration>0), il faut revenir à la position initiale avant de relancer le scan.
      // la direction est forcement opposé à la direction qui a permit de rejoindre la position optimale. on rejoint le depart et on remet la direction dans le bon sens
      // pour le prochain scan.

      if (iteration > 0) // not for the first iteration obviously
      {
        Go_to_Home()
      }

      scan();
      iteration ++; // compte le nombre de scan;
    }

    // ina219
    current_mA = ina219.getCurrent_mA();
    voltage_V = ina219.getBusVoltage_V();

    //ads1115



//    adc0 = ads1115.readADC_SingleEnded(0);
//    adc1 = ads1115.readADC_SingleEnded(1);
//    adc2 = ads1115.readADC_SingleEnded(2);
//    adc3 = ads1115.readADC_SingleEnded(3);
//
//
//    voltage_0 = (adc0 * 0.1875) / 1000;
//    voltage_1 = (adc1 * 0.1875) / 1000;
//    voltage_2 = (adc2 * 0.1875) / 1000;
//    voltage_3 = (adc3 * 0.1875) / 1000;


    // K means "kinetics"

    Serial.print("K"); Serial.print(",");
    Serial.print(myWantedTime); Serial.print(",");
    Serial.print(voltage_V); Serial.print(","); // I divide by 50 in place of 100 because there is a divider bridge to protect A3
    Serial.print(current_mA); Serial.print(",");
    Serial.print(fabs(voltage_V * current_mA)); Serial.print(",");// favs maybe useless

    Serial.print(ads1115.readADC_SingleEnded(0)); Serial.print(",");
    Serial.print(ads1115.readADC_SingleEnded(1)); Serial.print(",");
    Serial.print(ads1115.readADC_SingleEnded(2)); Serial.print(",");
    Serial.print(ads1115.readADC_SingleEnded(3)); Serial.print(",");


    Serial.print(Compass.getHeadingX()); Serial.print(",");
    Serial.print(Compass.getHeadingY()); Serial.print(",");
    Serial.println(Compass.getHeadingZ());// Serial.print(",");

    delay(Delay_Chosen);

  }

}