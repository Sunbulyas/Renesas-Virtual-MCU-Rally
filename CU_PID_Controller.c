#include "renesas_api.h"

#define UPRAMP 100.0
#define MEDIUM 30.0
#define SLOW 17.0
#define BRAKE -30.0
#define U_BRAKE -50.0

#define STRICT 2000.0
#define LOOSE 25.0

//3.2----1.5
#define Kp 3.2
#define Kd 1.5

#define K_p 0.05999999 
#define K_d 1.5
//0.05999----1.5

#define BaseSpeed 50
#define MaxSpeed 100
//85-100
int lastError = 0;
int lastError_1 = 0 ;

int error;
float angle;
int error1;
float adding;
float rightMotorSpeed;
float leftMotorSpeed;

enum states_t
{
  PID,
  CORNER_IN,
  CORNER_OUT,
  RIGHT_CHANGE_DETECTED,
  RIGHT_TURN,
  LEFT_CHANGE_DETECTED,
  LEFT_TURN,
  RAMP_UP,
  RAMP_DOWN
} state = PID;

float s =1.0;
double last_time = 0.0;
int left_change_pending = 0;
int right_change_pending = 0;
int detected_state = PID;

int main(int argc, char **argv)
{
  wb_robot_init(); // this call is required for WeBots initialisation
  init();          // initialises the renesas MCU controller

  while (wb_robot_step(TIME_STEP) != -1)
  {
    update();
    double *angles = imu();
    unsigned short *sensor = line_sensor();    
    float line = 0, sum = 0, weighted_sum = 0;
    bool double_line = 0;
    bool left_change = 0;
    bool right_change = 0;
    for (int i = 0; i < 8; i++)
    { 
      weighted_sum += sensor[i] * i;
      sum += sensor[i];
      if (sensor[i] < 400)
      {
        double_line++;
        if (i < 4)
        {left_change++;}
        else
        {right_change++;}
      }
    }
    line = weighted_sum / sum - 3.5;

    switch (state)
    {
      case PID:
        error = line*2000;
        angle = Kp * error + Kd * (error - lastError);
        lastError = error; 
          
        error1 = line;                                                           
        adding = K_p * error + K_d * (error1 - lastError_1);                                   
        lastError_1 = error1;
        
        if(angle>2000 || angle<-2000)s=0.5;
        if((angle<=2000 && angle>0 )   || (-2000<=angle && angle<0))s=1.0;
        if(angle==0)s=1.5;   
        
        float MotorSpeed = s*(BaseSpeed + adding);                                               
               
        if (MotorSpeed > MaxSpeed ) MotorSpeed = MaxSpeed;         
        if (MotorSpeed < 0)MotorSpeed = 0;

        motor(MotorSpeed, MotorSpeed, MotorSpeed, MotorSpeed);
        handle(angle);
             
        if (angles[1] > 0.0001) /////
      {
        last_time = time();
        state = RAMP_UP;
      }
      else if (angles[1] < -0.1)
      {
        last_time = time();
        state = RAMP_DOWN;
      }  
      else if (double_line > 7)
      {
        if (detected_state == CORNER_IN)
        {
          last_time = time();
          state = CORNER_IN;
        }
        detected_state = CORNER_IN;
      }
      else if (time() - last_time > 0.2 && right_change > 2 )
      {
        if (detected_state == RIGHT_CHANGE_DETECTED)
        {
          last_time = time();
          state = RIGHT_CHANGE_DETECTED;
        }
        detected_state = RIGHT_CHANGE_DETECTED;
      }
      else if (time() - last_time > 0.2 && left_change > 2 )
      {
        if (detected_state == LEFT_CHANGE_DETECTED)
        {
          last_time = time();
          state = LEFT_CHANGE_DETECTED;
        }
        detected_state = LEFT_CHANGE_DETECTED;
      }      
    break;
         
    case CORNER_IN:      

          if (time() - last_time < 0.05)
          {
            motor(U_BRAKE, U_BRAKE, U_BRAKE, U_BRAKE);
          }
          else if  ((time() - last_time >= 0.05) && (time() - last_time < 0.40))
          {
            motor(BRAKE, BRAKE, BRAKE, BRAKE);
          }
          else
          {
            motor(SLOW, SLOW, SLOW, SLOW);
          }   
      
      handle(LOOSE * line);
      if (time() - last_time > 0.2 && (left_change > 3 || right_change > 3))
      {
        last_time = time();
        state = CORNER_OUT;
      }
      break;      
    case CORNER_OUT:
      motor(MEDIUM, MEDIUM, MEDIUM, MEDIUM);
      handle(STRICT * line);
      if (time() - last_time > 0.70)
      {
        last_time = time();
        state = PID;
      }
      break;
    case RIGHT_CHANGE_DETECTED:
      motor(MEDIUM, MEDIUM, MEDIUM, MEDIUM);
      handle(LOOSE * line);
      if (double_line == 0)
      {
        last_time = time();
        state = RIGHT_TURN;
      }
      if (time() - last_time > 1.5)
      {
        last_time = time();
        state = PID;
      }
      break;
    case RIGHT_TURN:
      motor(MEDIUM, MEDIUM, MEDIUM, MEDIUM);
      if (time() - last_time < 0.29)
        handle(-30);
      else if (time() - last_time < 0.8)
        handle(15);
      else if (double_line > 1)
      {
        last_time = time();
        state = PID;
      }
      break;
    case LEFT_CHANGE_DETECTED:
      motor(MEDIUM, MEDIUM, MEDIUM, MEDIUM);
      handle(LOOSE * line);
      if (double_line == 0)
      {
        last_time = time();
        state = LEFT_TURN;
      }
      if (time() - last_time > 1.5)
      {
        last_time = time();
        state = PID;
      }
      break;
    case LEFT_TURN:
      motor(MEDIUM, MEDIUM, MEDIUM, MEDIUM);
      if (time() - last_time < 0.29)
        handle(30);
      else if (time() - last_time < 0.8)
        handle(-15);
      else if (double_line > 1)
      {
        last_time = time();
        state = PID;
      }
      break;
      
    case RAMP_UP:
      motor(100, 100, 100, 100);
      handle(STRICT * line);
      if (angles[1] < 0.05)
      {
        last_time = time();
        state = PID;
      }
      break;
    case RAMP_DOWN:
      motor(SLOW, SLOW, SLOW, SLOW);
      handle(STRICT * line);
      if (angles[1] > -0.05)
      {
        last_time = time();
        state = PID;
      }     
      break;
    }
  };
  wb_robot_cleanup(); // this call is required for WeBots cleanup
  return 0;
}