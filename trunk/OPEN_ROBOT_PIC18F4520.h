#include <18f4520.h>
#device *=16 ADC=8
//#device CCSICD=TRUE

#include <bootloader_open_robot.h>

#fuses HS,NOLVP,WDT2048,PUT,NOBROWNOUT,NOPROTECT
#use delay(clock=20000000)
#use rs232(baud=115200, xmit=PIN_C6, rcv=PIN_C7,ERRORS,DISABLE_INTS,RESTART_WDT)

#define RAND_MAX 20
#include <stdlib.h>
#include <string.h>
#include <input.c>

#define SERVO_L      PIN_C1      //Output
#define SERVO_R      PIN_C2      //Output

#define Lservo_1A    PIN_D0      //Green, Output
#define Lservo_2A    PIN_D1      //Orange, Output
#define Rservo_1A    PIN_D2      //Green, Output
#define Rservo_2A    PIN_D3      //Orange, Output

#define ENC_R_CLK    PIN_A4      //Violet
#define ENC_R_DIR    PIN_D7      //Orange, Input

#define ENC_L_CLK    PIN_B0      //Violet
#define ENC_L_DIR    PIN_D6      //Orange, Input

#define RFID_RESET   PIN_C5      //RFID RESET PIN

#define TRIS_D_VAL   0x30        //D0,D1,D2,D3,D6,D7=Outputs, D4,D5=Inputs.
#define TRIS_B_VAL   0x0F        //B0,B1,B2,B3=Inputs, B4,B5,B6,B7=Outputs.

#define LEFT_MOTOR  TRUE
#define RIGHT_MOTOR FALSE
