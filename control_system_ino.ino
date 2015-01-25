#include "definitions.h"
#include <stdlib.h>

/* Dynamixel Initialization */
Dynamixel Dxl(DXL_BUS_SERIAL1);

unsigned long time;

void setup() {

  SerialUSB.attachInterrupt(usbInterrupt);
  pinMode(BOARD_LED_PIN, OUTPUT);  //toggleLED_Pin_Out
  // Dynamixel 2.0 Baudrate -> 0: 9600, 1: 57600, 2: 115200, 3: 1Mbps 
  Dxl.begin(3);
  /* jointMode() is to use position mode */
  Dxl.jointMode(ID_TOP); 
  Dxl.jointMode(ID_BOTTOM);
  
  Dxl.writeWord(ID_TOP, GOAL_POSITION, 2048); 
  delay(1000);
  Dxl.writeWord(ID_BOTTOM, GOAL_POSITION, 2048); 
  delay(3000);
  
}
/* convert computed angle to motor position */
int convert(float angle) {
  int pos = 2048 + round((angle / 360.0) * 1024);    
  return pos;  
}

/* Coordinates of the center of a face */
float x = 0, y = 0;
float last_x = 0, last_y = 0; // the previous position

/* Locker for motors */
int is_move = 1;

// randomly generated time between [MIN_TIME, MAX_TIME] to change position      
void moveHead(float t = MIN_TIME + rand() % int(MAX_TIME - MIN_TIME) + 0.0) {
   if (is_move == 1) {
      
      /* minimum jerk computation */
      float t_2 = t * t;
      float t_3 = t_2 * t;
      float t_4 = t_3 * t;
      float t_5 = t_4 * t;
      float x_a[4], y_a[4];
      x_a[0] = last_x;
      y_a[0] = last_y;
      float ax[3][4] = {
        {t_3, t_4, t_5, x - x_a[0]},
        {3*t_2, 4*t_3, 5*t_4, 0.0},
        {6*t, 8*t_2, 20*t_3, 0.0}
      };
      float ay[3][4] = {
        {t_3, t_4, t_5, y - y_a[0]},
        {3*t_2, 4*t_3, 5*t_4, 0.0},
        {6*t, 8*t_2, 20*t_3, 0.0}
      };
      gauss(ax, x_a);  // x_a(t) is 5th degree polynomial position profile
      gauss(ay, y_a);  // x_a(t) is 5th degree polynomial position profile
      
      
      /* modeling motion using x_a(t) and y_a(t) position profiles */
      for (float tm = 0.0; tm <= t; tm += SAMPLE_TIME) {
        float x_t = x_a[0], y_t = y_a[0];
        float tt = tm*tm*tm;
        for (int i = 3; i <= 5; i++)
          x_t += x_a[i - 2] * tt, y_t += y_a[i - 2] * tt, tt *= tm;
        float angle_TOP = y_t * FV_Y / HEIGHT;
        float angle_BOTTOM = x_t * FV_X / WIDTH;
    
        int pos_BOTTOM = convert(angle_BOTTOM);
        int pos_TOP = convert(angle_TOP);
    
        Dxl.writeWord(ID_TOP, GOAL_POSITION, pos_TOP);    
        Dxl.writeWord(ID_BOTTOM, GOAL_POSITION, pos_BOTTOM);
        
       // SerialUSB.println(pos_TOP);
       // SerialUSB.println(pos_BOTTOM);
        
        Dxl.flushPacket();
        if(!Dxl.getResult()){
          SerialUSB.println("Comm Fail");
        }  
    }
      last_x = x;
      last_y = y;
   }
}

#define MIN_FREE_TIME 2500
#define MAX_FREE_TIME 6000
#define RANGE_X_INTERVAL 7000
#define RANGE_Y_INTERVAL 1000
#define RANGE_DUR_TIME 2000

float getUniformRand() {
  return (rand() % RAND_MAX + .0) / RAND_MAX;
}

int lock_random_movement = 0;
int is_following = 1;
unsigned long last_time = 0, delta_time = 0;

void randomMovements() {
  if (is_following) {
    lock_random_movement = 0;
    return;
  }
  SerialUSB.println(millis());
  if (!lock_random_movement) {
    last_time = millis();
    delta_time = (unsigned long) (getUniformRand() * (MAX_FREE_TIME - MIN_FREE_TIME) + MIN_FREE_TIME);
    SerialUSB.println(last_time);
    SerialUSB.println(delta_time);
  } else {
    if (millis() - last_time > delta_time) {
      x += (getUniformRand() - getUniformRand()) * RANGE_X_INTERVAL;
      y += (getUniformRand() - getUniformRand()) * RANGE_Y_INTERVAL;
      SerialUSB.println("RANDOM MOVE!");
    } else { 
      randomMovements();
      return;
    }
  }
  lock_random_movement ^= 1;
  randomMovements();
}

void usbInterrupt(byte* buffer, byte nCount){
  SerialUSB.print("nCount =");
  SerialUSB.println(nCount);
  for(unsigned int i=0; i < nCount;i++)  
    SerialUSB.print((char)buffer[i]);
  SerialUSB.println("");
  if ((char)buffer[0] == '1') {
    x = -(rand()%6700);
    y = +(rand()%3300);
    is_following = 1;
  } else
  if ((char)buffer[0] == '2') {
    x = +(rand()%6700);
    y = +(rand()%3300);
    is_following = 1;
  } else
  if ((char)buffer[0] == '3') {
    x = +(rand()%6700);
    y = -(rand()%3300);
    is_following = 1;
  } else
  if ((char)buffer[0] == '4') {
    x = -(rand()%6700);
    y = -(rand()%3300);
    is_following = 1;
  } else
  if ((char)buffer[0] == 'f') {
    is_following = 0;
    randomMovements();
  } else {
    is_following = 1;
    x = 0;
    y = 0;
  }
}

/* Solution of system of equations for minimum jerk */
#define EPS 1e-4
void gauss(float a[][4], float res[]) {
  int n = 3;
  int m = 3;
  int where[3] = {-1, -1, -1};
  for (int col = 0, row = 0; col < m && row < n; col++) {
    int sel = row;
    for (int i = row; i < n; i++)
      if (abs(a[i][col]) > abs(a[sel][col]))
        sel = i;
      if (abs(a[sel][col]) < EPS)
        continue;
     for (int i = col; i <= m; i++) {
       float temp = a[sel][i];
       a[sel][i] = a[row][i];
       a[row][i] = temp;
     }
     where[col] = row;
    for (int i = 0; i < n; i++)
      if (i != row) {
        float c = a[i][col] / a[row][col];
        for (int j = col; j <= m; j++)
          a[i][j] -= a[row][j] * c;
      }
  }
  for (int i = 0; i < m; i++)
    if (where[i] != -1)
      res[i + 1] = a[where[i]][m] / a[where[i]][i];
}



void loop() {  
    
    
    toggleLED();
  
    if (abs(x * 2) >= WIDTH || abs(y * 2) >= HEIGHT) {
     // is_move = 0;
    }
  
    if (is_following)
      moveHead();
    else {
      if (lock_random_movement) {
        
      } else {
      
      }
    }
}



