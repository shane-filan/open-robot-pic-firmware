////////////////////////////////////////////////////////////////////////////////
// This program utilizes a PIC18F4520 to read/write data to RFID tags, poll/////
// connected sensors and control the motion of the two GM-8 Gearhead       /////
// motors using PID velocity & position control & low-cost WW-02 encoders. /////
// Commands are recieved by the PIC's UART via ZigBee, XBee wireless       /////
// Transcievers, or a direct connection with a laptop/desktop/pda.         /////
// The PI motor control code was originally written by Pete Skeggs of      /////
// Nubotics, but has been modified and added to by Abe Howell. More        /////
// specifically, Abe has modified the PI control and has added full PID    /////
// velocity and position control along with numerous other functionalities,/////
// like the ability to read and write to RFID tags using the SonMicro RFID /////
// Chip Module and the entire serial command set that allows a high-level  /////
// to control/communicate with the OPEN-ROBOT Board.                       /////
// Please refer to www.abotics.com for more information                    /////
// or visit www.nubotics.com for additional information related to the     /////
// WW-02 Wheel Encoders. Send comments or questions to abe@abotics.com     /////
// Hope you find this program useful in your robotics journey.             /////
// I make no gurantee or warrenty with regard to this program.             /////
// Sincerely Abraham L. Howell                                             /////
////////////////////////////////////////////////////////////////////////////////

#include <OPEN_ROBOT_PIC18F4520.h>

// timer2 variables
long timer2_overflow;                 // incremented when timer2_ticks overflows
int32 timer2_ticks;                   // incremented once per 204us; wraps every 10 days
long timer2_seconds_counter;          // counts up to 2441 to increment seconds_elapsed
int servo_counter;                    // determine when to execute the servo control loop
int rfid_counter;
int speed_counter;

// encoder variables
short enc_fwdir_R;                    // most recent value read from the right WW-01's DIR line
short enc_fwdir_R_prev;               // previous value
short enc_fwdir_L;
short enc_fwdir_L_prev;

// position variables
signed int16 enc_pos_R;               // in encoder ticks, about 0.0675" per tick; max 2212 inches = 184 feet
signed int16 enc_pos_L;

// velocity measurement variables
int32 prev_t2_R;                      // the value of timer2_ticks the last time we saw a clock pulse from the right WW-01
int32 prev_t2_L;
int index_R;                          // current position in the array of periods
int index_L;
int32 enc_period_R_sum;               // running sum of the last four clock periods
int32 enc_period_L_sum;
int32 enc_period_R;                   // the same as the sum divided by 4
int32 enc_period_L;
int32 enc_period_R_array[4];          // time between pulses of the right WW-01's clock line, in multiples of timer 2 overflows (every 204us)
int32 enc_period_L_array[4];

signed int16 enc_speed_R;             // current signed rotation rate, in units of 0.1 inches per second
signed int16 enc_speed_L;

// velocity control PID variables
signed int16 i_err_R, i_err_L;               // integral of error
signed int16 prev_vel_err_R, prev_vel_err_L; // previous velocity errors
signed int16 prev_pos_err_R, prev_pos_err_L; // previous position errors
signed int16 out_R, out_L;                   // value to output to the motor
short dir_R, dir_L;                          // direction to drive the motor

signed int16 req_speed_R;              // desired or commanded velocity, in tenths of an inch per second
signed int16 req_speed_L;

// position control PID variables
signed int16 req_pos_R;                // desired or commanded position, in encoder ticks
signed int16 req_pos_L;
signed int16 req_pos_max_vel;          // desired maximum velocity while moving to the new position
signed int16 req_pos_cur_max;          // current max value
signed int16 req_pos_vel_incs;         // amount to add to velocity each increment (used for initial acceleration)
signed int16 req_pos_inc_down_count;   // when we are done accelerating to the desired maximum speed
signed int16 pos_i_err_R, pos_i_err_L; // integrator of position error (used to overcome frictional errors)

// program state flags
short pos_reached;                     // we have reached the desired positions on both wheels
short r_enc_read;                      // we have read something from the encoder
short l_enc_read;
short update_servo;                    // it is time to calculate the PID control loop
short update_pwm;                      // it is time to update the motor PWMs

// constants
#define POS_INTEG_LIMIT 55             // maximum velocity value that we allow the position integral term to grow to to prevent windup
#define INTEG_LIMIT 50                 // maximum error value that we allow the velocity integral term to grow to to prevent windup
#define MAX_SPEED 55                   // maximum velocity in tenths of an inch per second (depends on the servo brand and other factors)
#define ACCEL_INCREMENTS 1             // how fast to advance the maximum allowed velocity when doing position control
#define MAX_POSITION 250               // Maximum allowed position due to 16-bit calculations.

// calculate the speed conversion factor
#define Dwh 2.621                     // wheel diameter
#define PI 3.14159                    // PI
#define Cwh (Dwh * PI)                // wheel circumference
#define TSCALE 10                     // convert to 10ths of an inch
#define INVTICK 4883                  // this is 1 / 204.8 us, to avoid using floating point math
#define NCLKS 128                     // number of encoder clocks per wheel rotation
#define Ktps ((Cwh * TSCALE * INVTICK) / NCLKS)
#define vel_prop 1

// EEPROM Parameters. These parameters can be written to the Data EEPROM and
// then retrieved upon next boot.
// eeprom[0]=vKp,eeprom[1]=vKi,eeprom[2]=vKd,eeprom[3]=pKp,eeprom[4]=pKi,eeprom[5]=pKd
// eeprom[6]=ir sensor reflex level, eeprom[7]=light reflex level
// eeprom[8]= left pwm motor offset, eeprom[9]= right pwm motor offset.
int eeprom[10];

// RFID & RS232 VARIABLES
char RFID_Buff[18];
char rs232_buff[16];
int rs232_buff_index=0;
int motor_data[16];

// Various control variables.
short agent_blocked=FALSE;
short velocity_control=FALSE;
short position_control=FALSE;
short speed_control=FALSE;
short left_motor_dir,right_motor_dir;
short halt_agent=TRUE;
short find_rfid=FALSE;
short tag_found=FALSE;
short update_rfid=FALSE;
short wander=FALSE;
short update_speed=TRUE;
short parse_motion_data = FALSE;

int wander_block_count=0;

int left_motor_speed,right_motor_speed;

int AD_Reflex=0;

//A/D Sensor Variables.
int ad_chan_2,ad_chan_3,ad_chan_1,sensor_id,battery_level;
int ad_chan_4,ad_chan_5,right_light,left_light;

// Used in the PID Position Loop and adjusts how abruptly the robot decelerates.
int stop_constant = 180;

/******************************************************************************/
/* All function declarations.                                                 */
/******************************************************************************/
void set_velocity(signed long speed_R, signed long speed_L);
void control_velocity(void);
void control_position(void);
void parse_motor_commands();
void stop_agent(short clear);
void init_motors(void);
void set_motor_speed_dir(short left, int speed, short fwdd);
void setup_hardware(void);
short GetRFID(void);
void Write_Tag_Data(void);
void calc_enc_speeds(void);
short Set_RFID_Auto_Mode(void);
void Stop_RFID_READ(void);

/******************************************************************************/
/* Get 8-Bit Value from RS232_BUFF                                            */
/*                                                                            */
/* Function to parse out integer from incoming buffer.                        */
/******************************************************************************/
int get_rs232_int(int start_pos)
{
   char tmpBuff[3];
   int i;

   tmpBuff[0]=tmpBuff[1]=tmpBuff[2]=' ';
   for (i=0;i<3;i++)
   {
      if (rs232_buff[i+start_pos]!=13) tmpBuff[i]=rs232_buff[i+start_pos];
      else break;
   }
   return (atoi(tmpBuff));
}

/******************************************************************************/
/* Parse PID Gains                                                            */
/*                                                                            */
/* Function to parse the PID Gains from incoming buffer.                      */
/******************************************************************************/
void parse_pid_gains(short position_gains)
{
   int tmp_val;

   tmp_val=get_rs232_int(2);

   if (!position_gains)
   {
      if (rs232_buff[1]=='P') eeprom[0]=tmp_val;
      else if (rs232_buff[1]=='I') eeprom[1]=tmp_val;
      else if (rs232_buff[1]=='D') eeprom[2]=tmp_val;
   }
   else
   {
      if (rs232_buff[1]=='P') eeprom[3]=tmp_val;
      else if (rs232_buff[1]=='I') eeprom[4]=tmp_val;
      else if (rs232_buff[1]=='D') eeprom[5]=tmp_val;
   }
}

/******************************************************************************/
/* This function will all parameters to Data EEPROM.                          */
/******************************************************************************/
void save_eeprom_parameters()
{
   int i;
   for (i=0;i<10;i++) write_eeprom(i,eeprom[i]);
}

/******************************************************************************/
/* This function will set the a/d reflex level.                               */
/******************************************************************************/
void set_ad_reflex_level()
{
   eeprom[6]=get_rs232_int(1);
}

/******************************************************************************/
/* This function will set the light reflex level.                             */
/******************************************************************************/
void set_light_reflex_level()
{
   eeprom[7]=get_rs232_int(1);
}

/******************************************************************************/
/* This function will set the pwm motor offset.                               */
/******************************************************************************/
void set_pwm_motor_offset(short left)
{
   if (left) eeprom[8]=get_rs232_int(2);
   else eeprom[9]=get_rs232_int(2);
}

/******************************************************************************/
/* This function reset the rfid chipmodule.                                   */
/******************************************************************************/
void reset_rfid_module()
{
   //Reset the RFID Chip Module.
   output_high(RFID_RESET);
   delay_us(10);
   output_low(RFID_RESET);
}

/******************************************************************************/
/* This function will set the specified I/O pin low.                          */
/******************************************************************************/
void set_io_low()
{
   switch (rs232_buff[1])
   {
      case '0':
         output_low(PIN_B1);
         break;

      case '1':
         output_low(PIN_B2);
         break;

      case '2':
         output_low(PIN_B3);
         break;

      case '3':
         output_low(PIN_B4);
         break;

      case '4':
         output_low(PIN_B5);
         break;

      case '5':
         output_low(PIN_B6);
         break;

      case '6':
         output_low(PIN_B7);
         break;

      case '7':
         output_low(PIN_C5);
         break;
   }
}

/******************************************************************************/
/* This function will set the specified I/O pin high.                         */
/******************************************************************************/
void set_io_high()
{
   switch (rs232_buff[1])
   {
      case '0':
         output_high(PIN_B1);
         break;

      case '1':
         output_high(PIN_B2);
         break;

      case '2':
         output_high(PIN_B3);
         break;

      case '3':
         output_high(PIN_B4);
         break;

      case '4':
         output_high(PIN_B5);
         break;

      case '5':
         output_high(PIN_B6);
         break;

      case '6':
         output_high(PIN_B7);
         break;

      case '7':
         output_high(PIN_C5);
         break;
   }
}

/******************************************************************************/
/* This function will get the appropriate I/O pin reading.                    */
/******************************************************************************/
short get_io()
{
   switch (rs232_buff[1])
   {
      case '0':
         return (input(PIN_B1));
         break;
      case '1':
         return (input(PIN_B2));
         break;
      case '2':
         return (input(PIN_B3));
         break;
      case '3':
         return (input(PIN_B4));
         break;
      case '4':
         return (input(PIN_B5));
         break;
      case '5':
         return (input(PIN_B6));
         break;
      case '6':
         return (input(PIN_B7));
         break;
      case '7':
         return (input(PIN_C5));
         break;
      default: return (0);
   }
}

/******************************************************************************/
/* This function will get the current IR reading of a specific sensor.        */
/******************************************************************************/
int get_ir_reading()
{
   switch (rs232_buff[1])
   {
      case '0':
         return (battery_level);
         break;

      case '1':
         return (ad_chan_1);
         break;

      case '2':
         return (ad_chan_2);
         break;

      case '3':
         return (ad_chan_3);
         break;

      case '4':
         return (ad_chan_4);
         break;

      case '5':
         return (ad_chan_5);
         break;

      case '6':
         return (right_light);
         break;

      case '7':
         return (left_light);
         break;

      default:
         return (0);
         break;
   }
}

/******************************************************************************/
/* RS232 RX. Receives and parses all incoming serial data.                    */
/******************************************************************************/
#int_RDA
RDA_isr(){
   int i;

   if (kbhit()){

      rs232_buff[rs232_buff_index]=getc();
      if (rs232_buff[rs232_buff_index]==13){

         rs232_buff_index=0;

         switch (rs232_buff[0]){

            // Get all A/D readings.
            case 'A':
               printf("a%U,%U,%U,%U,%U,%U,%U,%U\r",battery_level,ad_chan_1,ad_chan_2,ad_chan_3,ad_chan_4,ad_chan_5,right_light,left_light);
               break;

            // Is Agent Blocked?
            case 'B':
               if (!agent_blocked) printf("bc\r");
               else printf("bb\r");
               break;

            //Set A/D reflex Level.
            case 'C':
               set_ad_reflex_level();
               printf("c\r");
               break;

            // Is the move complete?
            case 'D':
               if (pos_reached) printf("da\r");
               else printf("dn\r");
               break;

            // Get Encoder readings.
            case 'E':
               printf("e%ld,%ld\r",enc_pos_L,enc_pos_R);
               break;

            // Get A/D Light reflex Levels.
            case 'F':
               printf("f%U,%U\r", eeprom[6], eeprom[7]);
               break;

            // Get PID Gains for velocity or position controller.
            case 'G':
               if (rs232_buff[1]=='P') printf("gp%U,%U,%U\r",eeprom[3],eeprom[4],eeprom[5]);
               else if (rs232_buff[1]=='V') printf("gv%U,%U,%U\r",eeprom[0],eeprom[1],eeprom[2]);
               else break;
               break;

            // Halt the Agent immediately!
            case 'H':
               stop_agent(1);
               printf("h\r");
               break;

            // Set AD Sensor Reflexes On/Off.
            case 'I':
               printf("i\r");
               AD_Reflex=get_rs232_int(1);
               agent_blocked=FALSE;
               break;

            //Set Specific I/O pin low.
            case 'J':
               set_io_low();
               printf("j\r");
               break;

            //Set Specific I/O pin low.
            case 'K':
               set_io_high();
               printf("k\r");
               break;

            //Get Specific I/O pin reading.
            case 'L':
               printf("l%U\r",get_io());
               break;

            // Motor Commands (Position, Velocity & Open Loop).
            case 'M':
               if (!parse_motion_data)
               {
                  parse_motion_data = TRUE;
                  for (i=0;i<16;i++) motor_data[i]=rs232_buff[i];
                  printf("ma\r");
               }
               else
               {
                  printf("mn\r");
               }
               break;

            // Set Light Reflex Level.
            case 'N':
               set_light_reflex_level();
               printf("n\r");
               break;

            // Set RFID Auto-Mode.
            case 'O':
               if (Set_RFID_Auto_Mode()) printf("oa\r");
               else printf("on\r");
               break;

            // Set Position PID Control Gains.
            // Default Kp=5, Ki=2, Kd=1
            case 'P':
               printf("p\r");
               parse_pid_gains(1);
               break;

            // Set Velocity PID Control Gains.
            // Default Kp=5, Ki=1, Kd=1
            case 'Q':
               printf("q\r");
               parse_pid_gains(0);
               break;

            // Reset IR Blockage.
            case 'R':
               agent_blocked=FALSE;
               printf("r\r");
               break;

            // Get Specific A/D Reading by channel.
            case 'S':
               printf("s%C%U\r",rs232_buff[1],get_ir_reading());
               break;

            // Read RFID Tag Data.
            case 'T':
               if (GetRFID())
               {
                  printf("t");
                  for (i=13;i<17;i++)
                  {
                     putc(RFID_Buff[i]);
                  }
                     printf("\r");
               }
               else
               {
                  printf("tn\r");
               }
               break;

            // Wander Mode.
            case 'U':
               halt_agent=FALSE;
               wander=TRUE;
               printf("u\r");
               break;

            // Get the current wheel velocities.
            case 'V':
               if (speed_control) calc_enc_speeds();
               printf("v%ld,%ld\r", enc_speed_L,enc_speed_R);
               break;

            // Write RFID Tag Data.
            case 'W':
               Write_Tag_Data();
               printf("wa\r");
               break;

            // Reset RFID Chip Module.
            case 'X':
               printf("x\r");
               reset_rfid_module();
               break;

            // Save all EEPROM Parameters.
            case 'Y':
               printf("y\r");
               save_eeprom_parameters();
               break;

            // Find RFID Tag.
            case 'Z':
               find_rfid=TRUE;
               tag_found=FALSE;
               printf("z\r");
               break;

            // RFID tag found?
            case 'z':
               if (tag_found) printf("za\r");
               else printf("zn\r");
               break;

            // OPEN-ROBOT Firmware Version Information.
            case '!':
               printf("!Firmware Version: V4 (Build Date = 06-06-09)\r");
               break;

            // Get or Set pwm motor offset.
            case '@':
               // Set pwm offset.
               if (rs232_buff[1] == 'L')
               {
                  set_pwm_motor_offset(1);
                  printf("@\r");
               }
               else if (rs232_buff[1] == 'R')
               {
                  set_pwm_motor_offset(0);
                  printf("@\r");
               }
               // Get pwm offset.
               else if (rs232_buff[1] == 'G') printf("@%U,%U\r",eeprom[8],eeprom[9]);
               break;
         }
      }
      else{
         rs232_buff_index++;
         if (rs232_buff_index>15)
         {
            rs232_buff_index=0;
            for (i=0;i<16;i++) rs232_buff[i]=' ';
         }
      }
   }
}

/******************************************************************************/
/* Timer0 ISR (Right Encoder Clk)                                             */
/* This ISR triggers on each right encoder tick & keeps a running average of  */
/* the period, which will later be used to calculate wheel velocity.          */
/******************************************************************************/
#int_timer0
timer0_isr()
{
   // Set to 255 so that we interrupt on next encoder tick.
   set_timer0(255);
   enc_fwdir_R = !input(ENC_R_DIR);
   if (enc_fwdir_R)
      enc_pos_R++;
   else
      enc_pos_R--;
   enc_period_R_sum -= enc_period_R_array[index_R];// remove the oldest value from the sum
   enc_period_R_sum += enc_period_R_array[index_R] = timer2_ticks - prev_t2_R;// store the newest value, and add it to the sum
   enc_period_R = enc_period_R_sum >> 2;// calculate a new average period
   index_R++;                   // move to the next array position
   if (index_R == 4)
      index_R = 0;
   prev_t2_R = timer2_ticks;    // store the current timer2_ticks for next time
   r_enc_read = 1;
}

/******************************************************************************/
/* Left Encoder Clk ISR                                                       */
/*                                                                            */
/******************************************************************************/
#int_ext
ext_isr()
{
   enc_fwdir_L = input(ENC_L_DIR);
   if (enc_fwdir_L)
      enc_pos_L++;
   else
      enc_pos_L--;
   enc_period_L_sum -= enc_period_L_array[index_L];
   enc_period_L_sum += enc_period_L_array[index_L] = timer2_ticks - prev_t2_L;
   enc_period_L = enc_period_L_sum >> 2;
   index_L++;
   if (index_L == 4)
      index_L = 0;

   prev_t2_L = timer2_ticks;
   l_enc_read = 1;
}

/******************************************************************************/
/* Timer 2 Interrupt Handler                                                  */
/*                                                                            */
/* this keeps track of time and updates the servos                            */
/* on a fixed interval                                                        */
/******************************************************************************/
#int_timer2
timer2_isr()
{
   int current_ir_dist;

   timer2_ticks++;
   timer2_seconds_counter++;
   if (timer2_seconds_counter == 100) // 25 = 20.479ms period (48.83 Hz)
   {
      servo_counter--;
      if (!servo_counter)
      {
         update_servo = 1;
         servo_counter = 10;
      }
      timer2_seconds_counter = 0;
      if (!update_rfid)
      {
         rfid_counter--;
         if (!rfid_counter)
         {
            update_rfid = 1;
            rfid_counter = 50; // 50 = 1023.95ms period (0.976 Hz)
         }
      }
      if (!update_speed)
      {
         speed_counter--;
         if (!speed_counter)
         {
            update_speed = 1;
            speed_counter = 10;
         }
      }
   }

   if (update_servo && position_control && !pos_reached) control_position();
   else if (update_servo && velocity_control) control_velocity();

   // Time to update the PWM value.
   if (update_pwm)
   {
      update_pwm = 0;
      set_motor_speed_dir(FALSE, (int)out_R, dir_R);
      set_motor_speed_dir(TRUE, (int)out_L, dir_L);
   }

   // Update the IR Sensor Data according to sensor_id.
   // This ensures that the most up to date readings are available.
   switch (sensor_id){

         case 0:
            battery_level=read_adc();
            sensor_id=1;
            break;
         case 1:
            ad_chan_1=read_adc();
            sensor_id=2;
            break;
         case 2:
            ad_chan_2=read_adc();
            sensor_id=3;
            break;
         case 3:
            ad_chan_3=read_adc();
            sensor_id=4;
            break;
         case 4:
            ad_chan_4=read_adc();
            sensor_id=5;
            break;
         case 5:
            ad_chan_5=read_adc();
            sensor_id=6;
            break;
         case 6:
            right_light=read_adc();
            sensor_id=7;
            break;
         case 7:
            left_light=read_adc();
            sensor_id=0;
            break;
         default:
            battery_level=read_adc();
            sensor_id=1;
            break;
   }
   //Switch to the new sensorid so that it will be ready with a valid reading next pass.
   set_adc_channel(sensor_id);
}

/******************************************************************************/
/* Calc Encoder Speeds                                                        */
/*                                                                            */
/* Speed in 0.1"/sec = (Cwh / NCLKS) * TSCALE / (TICK * PER) = Ktps / PER     */
/* where Cwh = 8.12" wheel circumference, NCLKS = 128 (32 stripe disk),       */
/* TSCALE = 10 (to convert inches to 0.1"), TICK = 205us per timer2 tick,     */
/* and PER is the measured period in timer 2 ticks                            */
/* Ktps = 3098                                                                */
/******************************************************************************/
void calc_enc_speeds()
{
   if (!r_enc_read)  // if the wheel is spinning so slow that we don't have current reading,
      enc_period_R = timer2_ticks - prev_t2_R;  // calculate period since last update
   else
      r_enc_read = FALSE;  // otherwise use it.

   enc_speed_R = (signed int16)(Ktps / enc_period_R); // converts from 205us ticks per edge to multiples of 0.1 inches per second

   if (!enc_fwdir_R)
      enc_speed_R = -enc_speed_R;

   if (!l_enc_read)
      enc_period_L = timer2_ticks - prev_t2_L;
   else
      l_enc_read = FALSE;

   enc_speed_L = (signed int16)(Ktps / enc_period_L);

   if (!enc_fwdir_L)
      enc_speed_L = -enc_speed_L;
}

/******************************************************************************/
/* Sat16                                                                      */
/* Limit value to be between positive and negative limits.                    */
/******************************************************************************/
#separate
signed long sat16(signed long value, signed long limit)
{
   if (!bit_test(value, 15))
   {
      if (value > limit)
         value = limit;
   }
   else
   {
      if (value < -limit)
         value = -limit;
   }
   return value;
}

/******************************************************************************/
/* Sat32                                                                      */
/* Limit value to be between positive and negative limits.                    */
/******************************************************************************/
#separate
signed long sat32(signed int32 value, signed int32 limit)
{
   if (!bit_test(value, 31))
   {
      if (value > limit)
         value = limit;
   }
   else
   {
      if (value < -limit)
         value = -limit;
   }
   return value;
}

/******************************************************************************/
/* Set Velocity                                                               */
/*                                                                            */
/* Set velocity in terms of tenths of an inch per second.                     */
/******************************************************************************/
void set_velocity(signed long speed_R, signed long speed_L)
{
   i_err_L = 0;
   i_err_R = 0;

   speed_R = sat16(speed_R, MAX_SPEED);
   if (bit_test(req_speed_R, 15) ^ bit_test(speed_R, 15))
      i_err_R = 0;                        // reset integral on direction change.
   req_speed_R = speed_R;

   speed_L = sat16(speed_L, MAX_SPEED);
   if (bit_test(req_speed_L, 15) ^ bit_test(speed_L, 15))
      i_err_L = 0;                        // reset integral on direction change.
   req_speed_L = speed_L;

}

/******************************************************************************/
/* Calculate Velocity Integral Term                                           */
/*                                                                            */
/******************************************************************************/
#separate
signed int16 calc_vel_integ(signed int16 integ, signed int16 err)
{
   return sat16(integ + (eeprom[1]*err), INTEG_LIMIT);
}

/******************************************************************************/
/* Calculate Motor Output                                                     */
/*                                                                            */
/* This converts the total control error to PWM or servo control value.       */
/*                                                                            */
/* GM8 Gearmotors rotate at 65-70 RPM when powered at 5volts, which is:       */
/* 70 (rev/min)*(1min/60sec)*8.23inches = 9.61 = 96.1 tenths per sec.         */
/* 65 (rev/min)*(1min/60sec)*8.23inches = 8.92 = 89.2 tenths per sec.         */
/* 255/8.92 = 2.86
/* 8.92 inches / sec = 89.2 tenths per sec. 255/89.2 =~ 2.86 =~ 45/17.        */
/******************************************************************************/
#separate
signed int16 calc_vel_out(signed int16 value)
{
   return ((value * (signed int16)90) / (signed long)23);
}

/******************************************************************************/
/* Control Velocity                                                           */
/*                                                                            */
/* This performs the PID (Proportional / Integral / Derivative)               */
/* control loop calculations.                                                 */
/******************************************************************************/
void control_velocity()
{
   signed int16 d_err_L, d_err_R, p_err_L, p_err_R, err_R, err_L; // derivative & proportional errors.

   update_servo = 0;    // wait until next update interval.

   calc_enc_speeds();   // how fast are we going?
   err_R = (req_speed_R - enc_speed_R); // calculate error term
   err_L = (req_speed_L - enc_speed_L);

   i_err_R = calc_vel_integ(i_err_R, err_R);// Calculate Integral Terms, with saturation to prevent windup
   i_err_L = calc_vel_integ(i_err_L, err_L);

   p_err_L = eeprom[0]*err_L;  //Calculate Proportional Error Terms.
   p_err_R = eeprom[0]*err_R;

   d_err_L = eeprom[2]*(err_L - prev_vel_err_L);  //Calculate Derivative Error Terms.
   d_err_R = eeprom[2]*(err_R - prev_vel_err_R);

   prev_vel_err_L = err_L;   //Previous error terms now update to become the current values.
   prev_vel_err_R = err_R;

   out_R = (p_err_R + i_err_R - d_err_R + (req_speed_R)*12)/16;
   out_L = (p_err_L + i_err_L - d_err_L + (req_speed_L)*12)/16;

   if (!bit_test(out_R,15))
   {
      out_R += eeprom[9];
      dir_R = TRUE;
   }
   else
   {
      out_R = -out_R;
      out_R += eeprom[9];
      dir_R = FALSE;
   }
   if (out_R > 255) out_R = 255;

   if (!bit_test(out_L, 15))
   {
      out_L += eeprom[8];
      dir_L = TRUE;
   }
   else
   {
      out_L = -out_L;
      out_L += eeprom[8];
      dir_L = FALSE;
   }
   if (out_L > 255) out_L = 255;

   // Print out the current velocity terms.
   /*printf("err_L=%ld , err_R=%ld\r", err_L, err_R);
   printf("p_err_L=%ld , p_err_R=%ld\r", p_err_L, p_err_R);
   printf("i_err_L=%ld , i_err_R=%ld\r", i_err_L, i_err_R);
   printf("d_err_L=%ld , d_err_R=%ld\r", d_err_L, d_err_R);
   printf("out_L=%ld , out_R=%ld\r", out_L, out_R);*/

   //Try to correct minor biases when driving with equal velocities.
   if ((abs(req_speed_R)==abs(req_speed_L))&&!position_control)
   {

      if (abs(enc_speed_R)>abs(enc_speed_L))
      {
         if (!bit_test(out_L, 15))
         {
            out_L += vel_prop;
            out_R -= vel_prop;
         }
         else
         {
            out_L-= vel_prop;
            out_R += vel_prop;
         }
      }
      if (abs(enc_speed_L)>abs(enc_speed_R))
      {
         if (!bit_test(out_R, 15))
         {
            out_R += vel_prop;
            out_L -= vel_prop;
         }
         else
         {
            out_R -= vel_prop;
            out_L += vel_prop;
         }
      }
   }

   if (out_R > 255) out_R = 255;
   if (out_L > 255) out_L = 255;

   update_pwm = 1;// tell interrupt handler to update PWM; this ensures it happens at a fixed rate
}

/******************************************************************************/
/* Reset PID Control Variables                                                */
/*                                                                            */
/* Function to reset variables before attempting position control.            */
/******************************************************************************/
#separate
void reset_pid_variables(void){

   int i;

   // Zero out encoder variables.
   enc_pos_L=0;
   enc_pos_R=0;
   enc_speed_R=0;
   enc_speed_L=0;
   index_R=0;
   index_L=0;
   enc_period_R_sum=0;
   enc_period_L_sum=0;
   enc_period_R=0;
   enc_period_L=0;
   prev_t2_L=0;
   prev_t2_R=0;
   l_enc_read=0;
   r_enc_read=0;

   for (i=0;i<4;i++){
      enc_period_R_array[i]=0;
      enc_period_L_array[i]=0;
   }

   // Zero out Velocity PID variables.
   i_err_R=0;
   i_err_L=0;
   prev_vel_err_R=0;
   prev_vel_err_L=0;
   // Zero out Position PID variables.
   pos_i_err_R=0;
   pos_i_err_L=0;
   prev_pos_err_L=0;
   prev_pos_err_R=0;
}

/******************************************************************************/
/* Set Position                                                               */
/*                                                                            */
/* This sets the left and right wheel positions.  If delta is 1, then the     */
/* position values are assumed to be relative to the current position, other- */
/* wise they are assumed to be absolute.  max_velocity is the upper limit     */
/* at which the caller wishes the robot to go while reaching that position.   */
/*                                                                            */
/* If the wheels are 8.20" in circumference, this is 0.064" per              */
/* position increment, or 15.6 ticks per inch.                                */
/* Position is controlled in terms of encoder ticks.                          */
/******************************************************************************/
void set_position(signed int16 posR, signed int16 posL, signed long max_velocity)
{
   int i;

   pos_reached = FALSE;

   //Reset Control Variables.
   reset_pid_variables();

   // Check to be sure we have not exceeded MAX_POSITION
   if (!bit_test(posL, 15))
   {
      if (posL > MAX_POSITION) posL = MAX_POSITION;
   }
   else
   {
      if (posL < -MAX_POSITION) posL = -MAX_POSITION;
   }
   if (!bit_test(posR, 15))
   {
      if (posR > MAX_POSITION) posR = MAX_POSITION;
   }
   else
   {
      if (posR < -MAX_POSITION) posR = -MAX_POSITION;
   }

   req_pos_R = posR;
   req_pos_L = posL;

   set_velocity(max_velocity, max_velocity); // do this to calculate goal max vel
   req_pos_max_vel = req_speed_R;

   req_pos_inc_down_count = req_speed_R / ACCEL_INCREMENTS; // calculate the number of steps to accelerate in
   if (bit_test(req_pos_inc_down_count, 15))
      req_pos_inc_down_count = -req_pos_inc_down_count; // make sure it is a positive value

   if (!req_pos_inc_down_count)      // not enough speed, so just go in one jump
   {
      req_pos_inc_down_count = 1;
      req_pos_vel_incs = req_speed_R;               // otherwise, go in one step
   }
   else
   {
      if (bit_test(req_speed_R, 15))// go in multiple steps, of the correct sign
         req_pos_vel_incs = -ACCEL_INCREMENTS;
      else
         req_pos_vel_incs = ACCEL_INCREMENTS;
   }
   req_pos_cur_max = req_pos_vel_incs;
   pos_i_err_R = pos_i_err_L = 0;

}

/******************************************************************************/
/* Calculate Velocity Based on Position Error                                 */
/*                                                                            */
/* The stop_constant is empirically found for a given system.  In effect,     */
/* it sets how quickly the robot decelerates as it gets close to the desired  */
/* position.                                                                  */
/* Since req_pos_cur_max (the current allowed maximum velocity) is in tenths  */
/* per second, and value is the error term in units of position in encoder    */
/* ticks, the stop_constant is in units of encoder ticks too.  Thus, the      */
/* value returned is velocity in tenths of an inch per second.                */
/******************************************************************************/
#separate
signed int16 calc_pos_speed(signed int16 value)
{
   return (signed int16)(((signed int32)req_pos_cur_max * value) / stop_constant);
}

/******************************************************************************/
/* Control Position                                                           */
/*                                                                            */
/* This routine calculates the position PID control loop.  It then uses       */
/* the resulting velocity values to set the inner velocity control loop.      */
/******************************************************************************/
void control_position()
{
   signed int16 spdR;
   signed int16 spdL;
   int spd_factor;

   signed int16 d_err_L, d_err_R, p_err_L, p_err_R, req_pos_err_R, req_pos_err_L;

   req_pos_err_R = req_pos_R - enc_pos_R; // calculate current error in position
   req_pos_err_L = req_pos_L - enc_pos_L;

   p_err_L = eeprom[3]*req_pos_err_L*16;  //Calculate the proportional errors.
   p_err_R = eeprom[3]*req_pos_err_R*16;

   pos_i_err_R += eeprom[4]*req_pos_err_R*16; // integrate the error to ensure we reach the destination despite various frictional loads
   pos_i_err_L += eeprom[4]*req_pos_err_L*16;

   d_err_L = eeprom[5]*(req_pos_err_L-prev_pos_err_L)*16;  //Calculate the derivative errors.
   d_err_R = eeprom[5]*(req_pos_err_R-prev_pos_err_R)*16;

   prev_pos_err_L=req_pos_err_L; //Update the previous errors with the current error values.
   prev_pos_err_R=req_pos_err_R;

   pos_i_err_R = sat16(pos_i_err_R, POS_INTEG_LIMIT); // saturate the integrator to prevent windup / overshoot / instability
   pos_i_err_L = sat16(pos_i_err_L, POS_INTEG_LIMIT);

   spdR = calc_pos_speed(p_err_R + pos_i_err_R - d_err_R)/16; // convert the total position error term to desired velocity
   spdL = calc_pos_speed(p_err_L + pos_i_err_L - d_err_L)/16;

   // Print out position control values.
   /*printf("req_pos_err_L=%ld , req_pos_err_R=%ld\r", req_pos_err_L, req_pos_err_R);
   printf("p_err_L=%ld , p_err_R=%ld\r", p_err_L, p_err_R);
   printf("pos_i_err_L=%ld , pos_i_err_R=%ld\r", pos_i_err_L, pos_i_err_R);
   printf("d_err_L=%ld , d_err_R=%ld\r", d_err_L, d_err_R);
   printf("spdL=%ld , spdR=%ld\r", spdL, spdR);
   printf("req_pos_cur_max=%ld\r", req_pos_cur_max);*/

   // Only perform if driving straight and destination for each wheel is same!
   // This code attempts to correct a leading/lagging wheel situation.
   spd_factor=1;
   if (((abs(enc_pos_L)>10) && (abs(enc_pos_R)>10)) && (abs(enc_pos_L)<abs(req_pos_L)-5) && (abs(enc_pos_R)<abs(req_pos_R)-5))
   {
     if (abs(req_pos_R) == abs(req_pos_L))
      {
         // Left Wheel is leading in encoder counts.
         if (abs(enc_pos_L)>abs(enc_pos_R))
         {
            if (!bit_test(enc_pos_L, 15)) spdL-=spd_factor;
            else spdL+=spd_factor;
         }
         // Right Wheel is leading in encoder counts.
         else
         {
            if (!bit_test(enc_pos_R, 15)) spdR-=spd_factor;
            else spdR+=spd_factor;
         }
      }
   }

   // Check to see if we have reached the destination.
   if (!pos_reached)
   {
      if ((abs(req_pos_err_R) < 6) || (abs(req_pos_err_L) < 6))   // allow up to 1 stripe in error (too small, and the system is unstable because we can't stop that accurately)
      {
         if ((abs(req_pos_err_R) < 6)) spdR=0;
         if ((abs(req_pos_err_L) < 6)) spdL=0;
         if ((abs(req_pos_err_R) < 6) && (abs(req_pos_err_L) < 6)) pos_reached = TRUE;
      }
      if (req_pos_inc_down_count)              // have we finished accelerating?
      {
         req_pos_inc_down_count--;
         req_pos_cur_max += req_pos_vel_incs;  // not yet -- bump up the speed
      }
      else
         req_pos_cur_max = req_pos_max_vel;    // we've reached the speed limit
   }

   if (!pos_reached){
      set_velocity(spdR, spdL);                   // output our calculated speeds
      control_velocity();                         // then use them to drive the motors.
   }
   else{
      stop_agent(1);
   }
}

#separate
/******************************************************************************/
/* Function to convert open loop speeds accordingly.                          */
/******************************************************************************/
int adjust_motor_speed(long motor_speed, short left)
{
   if (motor_speed>55) motor_speed=55;
   if (motor_speed != 0)
   {
      if (left) motor_speed += eeprom[8];
      else motor_speed += eeprom[9];

   }
   return (int)motor_speed;
}

/******************************************************************************/
/* Parse Motor Commands                                                       */
/*                                                                            */
/* Function to parse the motor commands  from incoming buffer.                */
/******************************************************************************/
void parse_motor_commands(){

   signed int16 left_ticks,right_ticks;
   char tick_buff[5];
   int i,j,comma_pos,end_pos;
   short left_motor_dir_prev, right_motor_dir_prev;

   left_motor_dir_prev = left_motor_dir;
   right_motor_dir_prev = right_motor_dir;

   // ','=44 '-'=45

   //Now determine the comma_pos and end_pos.
   for (i=2;i<16;i++){
      if (motor_data[i]==44) comma_pos=i;
      if (motor_data[i]==13){
          end_pos=i;
          break;
      }
   }

   //Parse out the left ticks.
   j=0;
   for (i=0;i<6;i++){
         tick_buff[i]=' ';
   }
   for (i=2;i<comma_pos;i++){
      if (motor_data[i]!=45){
         tick_buff[j]=motor_data[i];
         j++;
      }
   }
   left_ticks=atol(tick_buff);

   //Parse out the right ticks.
   j=0;
   for (i=0;i<6;i++){
         tick_buff[i]=' ';
   }
   for (i=comma_pos+1;i<end_pos;i++){
      if (motor_data[i]!=45){
         tick_buff[j]=motor_data[i];
         j++;
      }
   }
   right_ticks=atol(tick_buff);

   pos_reached=FALSE;
   update_pwm = 0;
   velocity_control=FALSE;
   position_control=FALSE;
   speed_control=FALSE;
   halt_agent=FALSE;
   wander=FALSE;

   if (motor_data[1]=='P'){
      if (motor_data[2]==45) left_ticks=-left_ticks;
      if (motor_data[comma_pos+1]==45) right_ticks=-right_ticks;
      set_position(right_ticks,left_ticks,MAX_SPEED);
      position_control=TRUE;
   }

   else if (motor_data[1]=='O'){
      left_ticks = adjust_motor_speed(left_ticks,1);
      left_motor_speed = left_ticks;
      right_ticks = adjust_motor_speed(right_ticks,0);
      right_motor_speed = right_ticks;
      if (motor_data[2]==45) {
         left_motor_dir=0;
      }
      else {
         left_motor_dir=1;
      }
      if (motor_data[comma_pos+1]==45) {
         right_motor_dir=0;
      }
      else {
         right_motor_dir=1;
      }
      speed_control=TRUE;
      update_speed=TRUE;
   }

   else if (motor_data[1]=='V'){
      if (motor_data[2]==45) left_ticks=-left_ticks;
      if (motor_data[comma_pos+1]==45) right_ticks=-right_ticks;
      set_velocity(right_ticks,left_ticks);
      velocity_control=TRUE;
   }

}

/******************************************************************************/
/*  Function to stop the agent immediately!                                   */
/******************************************************************************/
void stop_agent(short clear)
{
   // Update all state variables.
   pos_reached=TRUE;
   velocity_control=FALSE;
   position_control=FALSE;
   speed_control=FALSE;
   find_rfid=FALSE;
   wander=FALSE;

   // Zero out the velocities of each wheel.
   enc_speed_L=0;
   enc_speed_R=0;

   // If processing a Sensor reflex trigger.
   if (!clear)
   {
      agent_blocked=TRUE;
   }

   // Actually stop the motors.
   set_velocity(0, 0);
   set_motor_speed_dir(FALSE, 0, 1);
   set_motor_speed_dir(TRUE, 0, 1);
   out_L = out_R = 0;
   update_pwm = 0;
   halt_agent=TRUE;
}

/******************************************************************************/
/*  Function to check AD Sensor Reflexes and stop the agent immediately!      */
/******************************************************************************/
void check_sensor_reflex()
{
   //If using AD sensor reflexes check the appropriate AD sensors.
   if (!agent_blocked && AD_Reflex!=0)
   {
      if (battery_level>eeprom[6]&&bit_test(AD_Reflex,0)) stop_agent(0);
      else if (ad_chan_1>eeprom[6]&&bit_test(AD_Reflex,1)) stop_agent(0);
      else if (ad_chan_2>eeprom[6]&&bit_test(AD_Reflex,2)) stop_agent(0);
      else if (ad_chan_3>eeprom[6]&&bit_test(AD_Reflex,3)) stop_agent(0);
      else if (ad_chan_4>eeprom[6]&&bit_test(AD_Reflex,4)) stop_agent(0);
      else if (ad_chan_5>eeprom[6]&&bit_test(AD_Reflex,5)) stop_agent(0);
      else if (right_light>eeprom[7]&&bit_test(AD_Reflex,6)) stop_agent(0);
      else if (left_light>eeprom[7]&&bit_test(AD_Reflex,7)) stop_agent(0);
   }
}

/******************************************************************************/
/*  Main Function. Zero out the ram.                                          */
/******************************************************************************/
#zero_ram
main() {

   int i,indx;
   short bNeg = FALSE;

   // Delay to let MatchPort boot. During setup we'll clear the RS232 buffer.
   delay_ms(500);

   //Setup the PIC hardware.
   setup_hardware();

   //Reset RFID Module.
   reset_rfid_module();

   while (TRUE) {

      //Restart the watchdog timer while looping forever!
      restart_wdt();

      // Check the IR Sensory reflex.
      check_sensor_reflex();

      if (parse_motion_data)
      {
         parse_motor_commands();
         parse_motion_data = FALSE;
      }

      if (halt_agent)
      {
         set_motor_speed_dir(FALSE, 0, 1);
         set_motor_speed_dir(TRUE, 0, 1);
      }
      else
      {
         if (speed_control && update_speed){
            set_motor_speed_dir(1,left_motor_speed,left_motor_dir);
            set_motor_speed_dir(0,right_motor_speed,right_motor_dir);
            update_speed=FALSE;
         }

         //Wander Mode.
         if (wander)
         {

            short LB,RB;
            LB=RB=0;

            left_motor_speed=adjust_motor_speed(25,1);
            right_motor_speed=adjust_motor_speed(25,0);

            if (ad_chan_2>eeprom[6]) LB=1;
            if (ad_chan_3>eeprom[6]) RB=1;

            wander_block_count+=1;

            // Case#1.
            if (!LB&&RB)
            {
               left_motor_dir=0;
               right_motor_dir=1;
            }
            // Case#2.
            else if (LB&&!RB)
            {
               left_motor_dir=1;
               right_motor_dir=0;
            }
            // Case#3.
            else if (LB&&RB)
            {
               if (LB>RB)
               {
                  left_motor_dir=0;
                  right_motor_dir=1;
               } else
               {
                  left_motor_dir=1;
                  right_motor_dir=0;
               }
            } else
            {
               left_motor_dir=1;
               right_motor_dir=1;
               wander_block_count=0;
            }

            if (wander_block_count>3) delay_ms(rand()*50);

            set_motor_speed_dir(1,left_motor_speed,left_motor_dir);
            set_motor_speed_dir(0,right_motor_speed,right_motor_dir);
         }
      }

      // Find RFID Tag as requested.
      if (find_rfid && update_rfid)
      {
         update_rfid=0;
         if (GetRFID())
         {
            stop_agent(1);
            find_rfid=FALSE;
            tag_found=TRUE;
         }
      }
   }
}

/******************************************************************************/
/*  Function to initialize motor control                                      */
/******************************************************************************/
void init_motors()
{
   //Setup Capture/Compare units for PWM mode, so that they can be used to pulse the motors.
   setup_ccp1(CCP_PWM);
   setup_ccp2(CCP_PWM);

   //Set PWM Duties to zero.
   set_pwm1_duty(0);
   set_pwm2_duty(0);

   //Set both motors to off.
   set_motor_speed_dir(LEFT_MOTOR, 0, 1);
   set_motor_speed_dir(RIGHT_MOTOR, 0, 1);

   //Set all the H-Bridge control pins to low.
   output_low(Lservo_1A);
   output_low(Lservo_2A);
   output_low(Rservo_1A);
   output_low(Rservo_2A);
}

/******************************************************************************/
/* Set Motor Speed Dir                                                        */
/*                                                                            */
/******************************************************************************/
void set_motor_speed_dir(short left, int speed, short fwdd)
{
   if (left)
   {
      if (fwdd){
         output_high(Lservo_2A);
         output_low(Lservo_1A);}
      else{
         output_high(Lservo_1A);
         output_low(Lservo_2A);}

      set_pwm2_duty(speed);
   }
   else
   {
      if (fwdd){
         output_high(Rservo_2A);
         output_low(Rservo_1A);}
      else{
         output_high(Rservo_1A);
         output_low(Rservo_2A);}

      set_pwm1_duty(speed);
   }
}

/******************************************************************************/
/*  Setup the PIC18F4520 Hardware                                             */
/******************************************************************************/
void setup_hardware()
{
   int i;

   // EEPROM Parameters. These parameters can be written to the Data EEPROM and
   // then retrieved upon next boot.
   // eeprom[0]=vKp,eeprom[1]=vKi,eeprom[2]=vKd,eeprom[3]=pKp,eeprom[4]=pKi,eeprom[5]=pKd
   // eeprom[6]=ir sensor reflex level, eeprom[7]=light reflex level
   // eeprom[8]= left pwm motor offset, eeprom[9]= right pwm motor offset.

   // Read in all EEPROM parameters.
   for (i=0;i<10;i++) eeprom[i]=read_eeprom(i);

   // Set PID gains as needed.
   if (eeprom[0]==255) eeprom[0]=5;
   if (eeprom[1]==255) eeprom[1]=1;
   if (eeprom[2]==255) eeprom[2]=1;
   if (eeprom[3]==255) eeprom[3]=5;
   if (eeprom[4]==255) eeprom[4]=2;
   if (eeprom[5]==255) eeprom[5]=1;
   
   // Set IR Sensor Reflex Level as needed.
   if (eeprom[6]==255) eeprom[6]=60;
   
   // Set PWM Motor Offsets if eeprom[8]==255 or eeprom[9]==255.
   if (eeprom[8]==255) eeprom[8]=196;
   if (eeprom[9]==255) eeprom[9]=196;

   AD_Reflex=0;
   for (i=0;i<16;i++) rs232_buff[i]=' ';
   pos_reached=TRUE;
   reset_pid_variables();
   servo_counter = 5;
   sensor_id=1;                                 //Set a/d channel#1 to read first.
   battery_level=0;
   ad_chan_1=0;
   ad_chan_2=0;
   ad_chan_3=0;
   ad_chan_4=0;
   ad_chan_5=0;
   right_light=0;
   left_light=0;
   set_tris_b(TRIS_B_VAL);                      //Setup PortB.
   set_tris_d(TRIS_D_VAL);                      //Setup PortD.
   setup_wdt(WDT_ON);                           //Set up watchdog timer in case the program locks it will be reset.
   setup_adc_ports(AN0_TO_AN7);
   setup_adc(ADC_CLOCK_INTERNAL);
   port_b_pullups(TRUE);
   setup_counters(RTCC_EXT_L_TO_H,RTCC_DIV_1 | RTCC_8_BIT); // set up timer 0 input T0CKI for Right CLK input
   set_timer0(255);                             // max count so interrupts on first edge
   setup_timer_1(T1_INTERNAL|T1_DIV_BY_2);      // set up timer 1 to generate proper timing for COMPARE unit RC servo timing
   set_timer1(0);
   setup_timer_2(T2_DIV_BY_1,255,4);            // 4883 Hz interrupt rate (204.8us period); 19531 Hz PWM frequency
   ext_int_edge(0,H_TO_L);                        // set up RB0 to interrupt on falling edge of Left CLK
   init_motors();                               // Init. Motors
   enable_interrupts(INT_EXT);                  // Enable RB0 Interrupt.
   enable_interrupts(INT_RTCC);                 // Right CLK
   enable_interrupts(INT_RDA);                  //RS232 Interrupt
   enable_interrupts(INT_TIMER2);               // time keeping
   enable_interrupts(GLOBAL);                   // Enable Global Interrupts.
   #priority rda                                //Set RX as top priority so that we receive all incoming serial data.

   // Clear out RS232 Buffer and set buffer index to zero.
   rs232_buff_index=0;
   for (i=0;i<16;i++) rs232_buff[i]=' ';

}

/******************************************************************************/
/*  Function to interrogate RFID board and get a current reading              */
/******************************************************************************/
short GetRFID(void)
{
   int i, start;
   long timeout;

   //#use rs232(baud=19200, xmit=PIN_D4, rcv=PIN_D5, DISABLE_INTS,SAMPLE_EARLY)
   #use rs232(baud=19200, xmit=PIN_D4, rcv=PIN_D5,SAMPLE_EARLY)

   for (i=0;i<18;i++) RFID_Buff[i]=0;

   //Send read command to RFID Board.
   putc(255);  // Header.
   putc(1);    // Reserved, always 1.
   putc(3);    // Length of payload (Data + Command).
   putc(16);   // Read tag command.
   putc(1);    // Mode type. Byte Track Mode - Manchester RF/64
   putc(2);    // Block# to read.
   putc(23);   // CheckSum.

   // Read in response from RFID module.
   for (i=0;i<5;i++)
   {
      timeout=0;
      while(!kbhit()&&(++timeout<60000))
      delay_us(5);
      if (kbhit())
         RFID_Buff[i]=getc();
      else
      {
         return (FALSE);
      }
   }
   for (i=5;i<18;i++)
   {
      timeout=0;
      while(!kbhit()&&(++timeout<60000))
      delay_us(5);
      if (kbhit())
         RFID_Buff[i]=getc();
      else
      {
         // Stop current read because no tag was found.
         Stop_RFID_READ();
         return (FALSE);
      }
   }

   return (TRUE);
}

/******************************************************************************/
/*  Function to write data to available RFID Tag                              */
/******************************************************************************/
void Write_Tag_Data(){

   int chksum, i;
   long timeout;

   chksum = 41 + rs232_buff[1] + rs232_buff[2] + rs232_buff[3] + rs232_buff[4];

   //Setup serial port for RFID Reader.
   //#use rs232(baud=19200, xmit=PIN_D4, rcv=PIN_D5, DISABLE_INTS, SAMPLE_EARLY)
   #use rs232(baud=19200, xmit=PIN_D4, rcv=PIN_D5, SAMPLE_EARLY)

   //Send write tag command to RFID Board.
   putc(255);           // Header.
   putc(1);             // Reserved, always 1.
   putc(6);             // Length of payload (Data + Command).
   putc(32);            // Write tag command.
   putc(2);             // Block# to be written.
   putc(rs232_buff[1]); // RFID data byte#1.
   putc(rs232_buff[2]); // RFID data byte#2.
   putc(rs232_buff[3]); // RFID data byte#3.
   putc(rs232_buff[4]); // RFID data byte#4.
   putc(chksum);        // CheckSum.

   // Only way to verify that correct data on tag is to read the tag.

 }

 /******************************************************************************/
 /*  Function to Set RFID Auto-Mode                                            */
 /******************************************************************************/
 short Set_RFID_Auto_Mode(void)
 {
   int i;
   long timeout;

   //Setup serial port for RFID Reader.
   //#use rs232(baud=19200, xmit=PIN_D4, rcv=PIN_D5, DISABLE_INTS, SAMPLE_EARLY)
   #use rs232(baud=19200, xmit=PIN_D4, rcv=PIN_D5, SAMPLE_EARLY)

   for (i=0;i<18;i++) RFID_Buff[i]=0;

   putc(255);  // Header.
   putc(1);    // Reserved, always 1.
   putc(9);    // Length of payload (Data + Command).
   putc(135);   // Auto-Mode Command.
   putc(0);    // Disable Auto-Mode.
   putc(1);    // Use Byte Track Mode. Manchester RF/64
   putc(2);    // # of Blocks to read.
   putc(0);    // No Password.
   putc(0);    // Password byte#1 even though not used.
   putc(0);    // Password byte#2 even though not used.
   putc(0);    // Password byte#3 even though not used.
   putc(0);    // Password byte#4 even though not used.
   putc(148);  // CheckSum.

   //Read in response from RFID module.
   for (i=0;i<5;i++)
   {
      timeout=0;
      while(!kbhit()&&(++timeout<60000))
      delay_us(5);
      if (kbhit())
         RFID_Buff[i]=getc();
      else
      {
         return (FALSE);
      }
   }

   return TRUE;
 }

 /******************************************************************************/
 /*  Function to Stop RFID Read.                                               */
 /******************************************************************************/
 void Stop_RFID_READ(void)
 {
   int i;

   //Setup serial port for RFID Reader.
   //#use rs232(baud=19200, xmit=PIN_D4, rcv=PIN_D5, DISABLE_INTS, SAMPLE_EARLY)
   #use rs232(baud=19200, xmit=PIN_D4, rcv=PIN_D5, SAMPLE_EARLY)

   for (i=0;i<18;i++) RFID_Buff[i]=0;

   putc(255);  // Header.
   putc(1);    // Reserved, always 1.
   putc(1);    // Length of payload (Data + Command).
   putc(12);   // Auto-Mode Command.
   putc(14);  // CheckSum.

 }

/******************************************************************************/
/*  End of the Program                                                        */
/******************************************************************************/
