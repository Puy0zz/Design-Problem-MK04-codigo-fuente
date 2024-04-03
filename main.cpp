/*######################################################################
  //#  G0B1T: uC EXAMPLES. 2024.
  //######################################################################
  //# Copyright (C) 2024. F.E.Segura-Quijano (FES) fsegura@uniandes.edu.co
  //#
  //# This program is free software: you can redistribute it and/or modify
  //# it under the terms of the GNU General Public License as published by
  //# the Free Software Foundation, version 3 of the License.
  //#
  //# This program is distributed in the hope that it will be useful,
  //# but WITHOUT ANY WARRANTY; without even the implied warranty of
  //# MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
  //# GNU General Public License for more details.
  //#
  //# You should have received a copy of the GNU General Public License
  //# along with this program.  If not, see <http://www.gnu.org/licenses/>
  //####################################################################*/

//=======================================================
//  LIBRARY Definition
//=======================================================
#include <Arduino.h>
#include <vector>
#include <cstdlib>
#include <ctime>

#define REALMATRIX // Variable to use real matrix. Comment to not use it.

#ifdef REALMATRIX
#include "LedControl.h"
/* Pin definition for MAX72XX.
 ARDUINO pin 12 is connected to the DataIn  - In ESP32 pin 23
 ARDUINO pin 11 is connected to the CLK     - In ESP32 pin 18
 ARDUINO pin 10 is connected to LOAD        - In ES32 pin 5
 We have only a single MAX72XX.
 */
LedControl lc=LedControl(23,18,5,1);
#endif

//=======================================================
//  IF PROBLEMS
//=======================================================
// In LedControl.h  change the following line:
// #include <avr/pgmspace.h>
// by 
/* #if (defined(__AVR__))
 #include <avr\pgmspace.h>
 #else
 #include <pgmspace.h>
 #endif
*/

//=======================================================
//  REGISTERS (VARIABLES) to control the game
//=======================================================
/* Registers to background cars.*/
byte RegBACKGTYPE_dataRANDOM;
byte RegBACKGTYPE_dataZEROS = B00000000;

/* Time delays. */
unsigned long delaytime = 200;
unsigned long dt = delaytime;

/* Global Variables */
int i = 0;
int count = 0;
int der = 14;
int izq = 12;
int alim = 13;
int derSTATE= 0;
int izqSTATE= 0;
int Level= 1;

/*Conjunto de bytes a usar para configurar el nivel 1*/
std::vector<unsigned char> byteSet = {
0x01, 0x02, 0x03 , 0x04, 0x06, 0x08, 0x0C,
0x10, 0x20, 0x30 , 0x40, 0x60, 0x80, 0xC0,
};


/*Conjunto de bytes a usar para configurar el nivel 2*/
std::vector<unsigned char> byteSet2 = {
0x03, 0x06, 0x0C, 0x30, 0x60, 0xC0,
0x07, 0x0E, 0x1C, 0x38, 0x70, 0xE0, 
0xF0, 0x78,0x3C,0x1E,0x0F,
0x1F,0x3E,0x7C,0xF8   
};

/*Conjunto de bytes a usar para configurar el nivel 3*/
std::vector<unsigned char> byteSet3 = {
0x07, 0x0E, 0x1C, 0x38, 0x70, 0xE0,
0xF0, 0x78,0x3C,0x1E,0x0F,
0x1F,0x3E,0x7C,0xF8,
0x3F,0x7E,0xFC,
0xFE,0x7F    
};

/* States ans signals to change state*/
enum State_enum {STATERESET, STATESTART, STATECLEAR, STATECHECK, STATECHECKMATRIX, STATELEFT, STATERIGTH, STATELOST,STATELCONFIG};
uint8_t state = STATERESET;

enum Keys_enum {RESET_KEY, START_KEY, LEFT_KEY, RIGHT_KEY, NO_KEY};
uint8_t keys = RESET_KEY;

enum Status_enum {LOST, CONTINUE};
uint8_t Status = CONTINUE;

/* Key to control game by serial input. */
  int incomingByte;

/* Pointer and Matrix to Control Driver. */
  byte RegMatrix[8];
  byte *pointerRegMatrix;

/* Pointer and Register to control bottom car. */
  byte RegCar[1];
  byte *pointerRegCar;

/* Pointer and Register (Variable) to move bottom car */
  byte ShiftDir[1];
  byte *pointerShiftDir;

//=======================================================
//  getRandomByte() function
//=======================================================
/*Función que elige un número binario de 8 bits de forma aleatoria de un conjunto
determinado, clave para aumentar la dificultad de los niveles*/
unsigned char getRandomByte(const std::vector<unsigned char>& byteSet) {
    int index = rand() % byteSet.size(); // Generate a random index within the range of the vector
    return byteSet[index]; // Return the randomly chosen byte
}


//=======================================================
//  SETUP Arduino function
//=======================================================
/* Setup function initialization */
void setup()
{
#ifdef REALMATRIX
  /* The MAX72XX is in power-saving mode on startup, we have to do a wakeup call. */
  lc.shutdown(0, false);
  /* Set the brightness to a medium values. */
  lc.setIntensity(0, 8);
  /* Clear the display. */
  lc.clearDisplay(0);
#endif
  /* Serial port initialization. */
  Serial.begin(38400);

  /* Pointer to use Matrix between functions. */
  pointerRegMatrix = &RegMatrix[0];

  /* Pointer to use VectorCar between functions. */
  pointerRegCar = &RegCar[0];

  /* Pointer to use shift dir between functions */
  pointerShiftDir = &ShiftDir[0];

  //Definir las entradas por botones

  pinMode(der,INPUT); // declarar boton de la derecha como input
  pinMode(izq,INPUT); // declarar boton de la izquierda como input
  pinMode(alim,OUTPUT);// se declara el pin que da alimentación a los botones
}

//=======================================================
//  FUNCTION: writeResetMatrix
//=======================================================
/* Data matrix when reset*/
void writeResetMatrix(byte *pointerRegMatrix, byte *pointerRegCar)
{
  /* Global variables. */

  /* Here is the data to reset matrix */
  pointerRegMatrix[7] = B11111111;
  pointerRegMatrix[6] = B11111111;
  pointerRegMatrix[5] = B11111111;
  pointerRegMatrix[4] = B11111111;
  pointerRegMatrix[3] = B11111111;
  pointerRegMatrix[2] = B11111111;
  pointerRegMatrix[1] = B11111111;
  pointerRegMatrix[0] = B11111111;
  /* Here is the data to reset bottomCar */
  pointerRegCar[0] = B00000000;
}
//=======================================================
//  FUNCTION: writeStartMatrix
//=======================================================
void writeStartMatrix(byte *pointerRegMatrix, byte *pointerRegCar)
{
  /* Global variables. */

  /* Here is the data to start matrix */
  pointerRegMatrix[7] = B01111110;
  pointerRegMatrix[6] = B10000001;
  pointerRegMatrix[5] = B10011001;
  pointerRegMatrix[4] = B10000001;
  pointerRegMatrix[3] = B10100101;
  pointerRegMatrix[2] = B10011001;
  pointerRegMatrix[1] = B10000001;
  pointerRegMatrix[0] = B01111110;
/* Here is the data to start bottomCar */
  pointerRegCar[0] = B00000000;
}
//=======================================================
//  FUNCTION: writeClearMatrix
//=======================================================
void writeClearMatrix(byte *pointerRegMatrix, byte *pointerRegCar)
{
  /* Global variables. */

  /* Here is the data to clear matrix */
  pointerRegMatrix[7] = B00000000;
  pointerRegMatrix[6] = B00000000;
  pointerRegMatrix[5] = B00000000;
  pointerRegMatrix[4] = B00000000;
  pointerRegMatrix[3] = B00000000;
  pointerRegMatrix[2] = B00000000;
  pointerRegMatrix[1] = B00000000;
  pointerRegMatrix[0] = B00000000;
  /* Here is the data to clear bottomCar */
  pointerRegCar[0] = B00010000;
}
//=======================================================
//  FUNCTION: writeL1Matrix
//=======================================================
void writeL1Matrix(byte *pointerRegMatrix, byte *pointerRegCar)
{
  /* Global variables. */

  /* Here is the data to clear matrix */
  pointerRegMatrix[7] = B00001100;
  pointerRegMatrix[6] = B00001010;
  pointerRegMatrix[5] = B00001001;
  pointerRegMatrix[4] = B00001000;
  pointerRegMatrix[3] = B00001000;
  pointerRegMatrix[2] = B00001000;
  pointerRegMatrix[1] = B00111110;
  pointerRegMatrix[0] = B00000000;
  /* Here is the data to clear bottomCar */
  pointerRegCar[0] = B00010000;
}
//=======================================================
//  FUNCTION: writeL2Matrix
//=======================================================
void writeL2Matrix(byte *pointerRegMatrix, byte *pointerRegCar)
{
  /* Global variables. */

  /* Here is the data to clear matrix */
  pointerRegMatrix[7] = B11111111;
  pointerRegMatrix[6] = B10000000;
  pointerRegMatrix[5] = B10000000;
  pointerRegMatrix[4] = B11111111;
  pointerRegMatrix[3] = B00000001;
  pointerRegMatrix[2] = B00000001;
  pointerRegMatrix[1] = B11111111;
  pointerRegMatrix[0] = B00000000;
  /* Here is the data to clear bottomCar */
  pointerRegCar[0] = B00010000;
}
//=======================================================
//  FUNCTION: writeL3Matrix
//=======================================================
void writeL3Matrix(byte *pointerRegMatrix, byte *pointerRegCar)
{
  /* Global variables. */

  /* Here is the data to clear matrix */
  pointerRegMatrix[7] = B11111111;
  pointerRegMatrix[6] = B10000000;
  pointerRegMatrix[5] = B10000000;
  pointerRegMatrix[4] = B11111111;
  pointerRegMatrix[3] = B10000000;
  pointerRegMatrix[2] = B10000000;
  pointerRegMatrix[1] = B11111111;
  pointerRegMatrix[0] = B00000000;
  /* Here is the data to clear bottomCar */
  pointerRegCar[0] = B00010000;
}
//=======================================================
//  FUNCTION: writeLostMatrix
//=======================================================
void writeLostMatrix(byte *pointerRegMatrix, byte *pointerRegCar)
{
  /* Global variables. */

  /* Here is the data to lost matrix */
  pointerRegMatrix[7] = B01111110;
  pointerRegMatrix[6] = B10000001;
  pointerRegMatrix[5] = B10011001;
  pointerRegMatrix[4] = B10000001;
  pointerRegMatrix[3] = B10011001;
  pointerRegMatrix[2] = B10100101;
  pointerRegMatrix[1] = B10000001;
  pointerRegMatrix[0] = B01111110;
  /* Here is the data to lost matrix */
  pointerRegCar[0] = B00000000;
}
//=======================================================
//  FUNCTION: writeWinMatrix
//=======================================================
void writeWinMatrix(byte *pointerRegMatrix, byte *pointerRegCar)
{
  /* Global variables. */

  /* Here is the data to lost matrix */
  pointerRegMatrix[7] = B11110000;
  pointerRegMatrix[6] = B00010000;
  pointerRegMatrix[5] = B11010000;
  pointerRegMatrix[4] = B10011111;
  pointerRegMatrix[3] = B11110001;
  pointerRegMatrix[2] = B00001101;
  pointerRegMatrix[1] = B00001001;
  pointerRegMatrix[0] = B00001111;
  /* Here is the data to lost matrix */
  pointerRegCar[0] = B00000000;
}
//=======================================================
//  FUNCTION: writeGoCarsMatrix
//=======================================================
void writeGoCarsMatrix(byte *pointerRegMatrix)
{
  /* Global variables. */
  int m;

  i = i + 1;
  if (Level == 1){
    RegBACKGTYPE_dataRANDOM = getRandomByte(byteSet1);
  }else if (Level == 2){    
    RegBACKGTYPE_dataRANDOM = getRandomByte(byteSet2);
  }else if (Level == 3){    
    RegBACKGTYPE_dataRANDOM = getRandomByte(byteSet3);
  }
  /* Here is the data to start matrix */
  //random(1, 255);

  for (m = 0; m < 7; m++)
  {
    pointerRegMatrix[m] = pointerRegMatrix[m + 1];
  }
  if (i % 2 == 0)
    pointerRegMatrix[7] = RegBACKGTYPE_dataRANDOM;
  else
    pointerRegMatrix[7] = RegBACKGTYPE_dataZEROS;
}
//=======================================================
//  FUNCTION: writeCarBase
//=======================================================
void writeCarBase(byte *pointerRegCar, byte *pointerShiftDir)
{
  /* Global variables. */
  int m;
  
  /* Here is the data to start matrix */
  if (pointerShiftDir[0] == B00000001)
  {
    if (pointerRegCar[0] == B00000001)
      pointerRegCar[0] = pointerRegCar[0];
    else
      pointerRegCar[0] = pointerRegCar[0] >> 1;
  }
  else if (pointerShiftDir[0] == B00000010)
  {
    if (pointerRegCar[0] == B10000000)
      pointerRegCar[0] = pointerRegCar[0];
    else
      pointerRegCar[0] = pointerRegCar[0] << 1;
  }
  else
    pointerRegCar[0] = pointerRegCar[0];
}
//=======================================================
//  FUNCTION: checkLostMatrix (leds and console)
//=======================================================
void checkLostMatrix(byte *pointerRegMatrix, byte *pointerRegCar)
{
  /* Global variables. */
  byte check1, check2;
  check1 = pointerRegCar[0] & pointerRegMatrix[0];
  if (check1 == pointerRegCar[0]){
    Status = LOST;
  }else
    Status= CONTINUE;
}
//=======================================================
//  FUNCTION: printBits (by console all bits)
//=======================================================
void printBits(byte myByte)
{
  for (byte mask = 0x80; mask; mask >>= 1) {
    if (mask  & myByte)
      Serial.print('1');
    else
      Serial.print('0');
  }
}
//=======================================================
//  FUNCTION: PrintMatrix (Console)
//=======================================================
void PrintMatrix(byte *pointerRegMatrix, byte *pointerRegCar)
{
  /* Global variables. */
  int m;

  for (m = 7; m >= 1; m--)
  {
    printBits(pointerRegMatrix[m]);
    Serial.println();
  }
  printBits(pointerRegMatrix[0] | pointerRegCar[0]);
  Serial.println();
}
//=======================================================
//  FUNCTION: PrintALLMatrix (leds and console)
//=======================================================
void PrintALLMatrix(byte *pointerRegMatrix, byte *pointerRegCar)
{
  /* Global variables. */
  int m;

#ifdef REALMATRIX
  /* Display data one by one in matrix. */
  for (m = 7; m >= 1; m--)
  {
    lc.setRow(0, m, pointerRegMatrix[m]);
  }
  lc.setRow(0, m, (pointerRegMatrix[0] | pointerRegCar[0]));
#endif
  /* Display data one by one in console. */
  Serial.println("########");
  Serial.println("########");
  Serial.println("########");
  Serial.println("########");
  PrintMatrix(pointerRegMatrix, pointerRegCar);
  Serial.println("########");
}


//=======================================================
//  FUNCTION: read_KEY
//=======================================================
byte read_KEY(void)
{
  if (Serial.available() > 0)
  {
    incomingByte = Serial.read();
    delay(10);

  }
  switch (incomingByte)
  {
    case 'R':
      keys = RESET_KEY;
      break;
    case 'S':
      keys = START_KEY;
      break;
    case 'A':
      keys = LEFT_KEY;
      break;
    case 'D':
      keys = RIGHT_KEY;
      break;
    default:
      keys = NO_KEY;
      break;
  }

  //lectura botón físico
  derSTATE= digitalRead(der);
  izqSTATE = digitalRead(izq);
  delay(10);
  if (derSTATE == HIGH){
    keys= RIGHT_KEY;

  }
  else if (izqSTATE == HIGH){
    keys = LEFT_KEY;
 
  }

return keys;
}
//=======================================================
//  FUNCTION: state_machine_run
//=======================================================
void state_machine_run(byte *pointerRegMatrix, byte *pointerRegCar, byte *pointerShiftDir)
{
  /* Global variables. */

  PrintALLMatrix(pointerRegMatrix, pointerRegCar);
  

  switch (state)
  {
    case STATERESET:
      count = 0;
      Level = 1;
      writeResetMatrix(pointerRegMatrix,pointerRegCar);
      delay(delaytime);
      read_KEY();
      if (keys == RESET_KEY)
        state = STATERESET;
      else if (keys == START_KEY)
        state = STATESTART;
      break;

    case STATELCONFIG:
      if (count == 0){
        writeL1Matrix(pointerRegMatrix,pointerRegCar);
        delay(10*dt);
        state = STATECLEAR;
      }else if (count== 28){
        writeL2Matrix(pointerRegMatrix,pointerRegCar);
        Level= 2;
        delay(10*dt);
        state = STATECLEAR;
      }else if (count== 66){
        writeL3Matrix(pointerRegMatrix,pointerRegCar);
        delay(10*dt);  
        Level = 3;
        state = STATECLEAR;
      }else if (count ==113){
        incomingByte=Serial.read();//Para que no vuelva a emmpezar el juego automaticamente
        state= STATELCONFIG;
        writeWinMatrix(pointerRegMatrix,pointerRegCar);
        delaytime= dt;
        delay(10*delaytime);
        read_KEY();
        if (keys == START_KEY){
          state = STATESTART;
        }else if (keys == RESET_KEY){
          state = STATERESET;
        }     
        
        }
      break;

    case STATESTART:
      count = 0;
      Level= 1;
      writeStartMatrix(pointerRegMatrix,pointerRegCar);
      delay(10*delaytime);
      state = STATELCONFIG;
      break;

    case STATECLEAR:
      writeClearMatrix(pointerRegMatrix,pointerRegCar);
      delay(dt);
      state = STATECHECK;
      break;

    case STATECHECK:
      pointerShiftDir[0] = B00000000;
      state = STATECHECKMATRIX;
      writeCarBase(pointerRegCar, pointerShiftDir);
      read_KEY();
      //
      for (int i = 0; i < 8; ++i) {
        read_KEY();
        if (keys == RIGHT_KEY) {
          pointerShiftDir[0] = B00000010;
          writeCarBase(pointerRegCar, pointerShiftDir);
          checkLostMatrix(pointerRegMatrix, pointerRegCar);
          if (Status== LOST){
            state=STATELOST;
          }
        }else if (keys == LEFT_KEY) {
          pointerShiftDir[0] = B00000001;
          writeCarBase(pointerRegCar, pointerShiftDir);
          checkLostMatrix(pointerRegMatrix, pointerRegCar);
          if (Status== LOST){
            state=STATELOST;
            
          }
        }else if (keys == RESET_KEY){
          state= STATERESET;
        }
        PrintALLMatrix(pointerRegMatrix, pointerRegCar);
        delay(delaytime / 8);
        
      }
      //
      break;

    case STATECHECKMATRIX:
      writeGoCarsMatrix(pointerRegMatrix);
      state=STATECHECK;
      ++count;
      checkLostMatrix(pointerRegMatrix, pointerRegCar);
      if (Status == LOST)
        state = STATELOST;
      else if (count == 28 || count == 66 || count == 113){
        state= STATELCONFIG;
      }  
      else if (keys == RESET_KEY)
        state = STATERESET;
      else if (keys == LEFT_KEY)
        state = STATELEFT;
      else if (keys == RIGHT_KEY)
        state = STATERIGTH;
      
      break;

    case STATELEFT:
      pointerShiftDir[0] = B00000001;
      writeCarBase(pointerRegCar, pointerShiftDir);
      state = STATECHECK;
      break;

    case STATERIGTH:
      pointerShiftDir[0] = B00000010;
      writeCarBase(pointerRegCar, pointerShiftDir);
      state = STATECHECK;
      break;

    case STATELOST:
      writeLostMatrix(pointerRegMatrix,pointerRegCar);
      delaytime= dt;
      incomingByte=Serial.read();// Para que no vuelva a emmpezar el juego automaticamente
      delay(4*delaytime);
      read_KEY();
      if (keys == START_KEY)
        state = STATESTART;
      else if (keys == RESET_KEY){
        state= STATERESET;
      }
      else
        state = STATELOST;
      break;

    default:
      state = STATERESET;
      break;
  }
}
//=======================================================
//  FUNCTION: Arduino loop
//=======================================================
void loop()
{
  
  read_KEY();
  state_machine_run(pointerRegMatrix,pointerRegCar,pointerShiftDir);
  delay(1);
}
