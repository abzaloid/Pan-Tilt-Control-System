#include "definitions.h"
#include <stdlib.h>
#include <math.h>
/* Dynamixel Initialization */
Dynamixel Dxl(DXL_BUS_SERIAL1);

unsigned long time;

int is_sleeping = 0;

void goSleep() {
  is_sleeping = 1;
  Dxl.writeWord(ID_BOTTOM, GOAL_POSITION, 50); 
  delay(1000);
  Dxl.writeWord(ID_TOP, GOAL_POSITION, 2000 + 2048); 
  delay(1000);
}

void getUp() {
  is_sleeping = 0;
  Dxl.writeWord(ID_TOP, GOAL_POSITION, 2048); 
  delay(1000);
  Dxl.writeWord(ID_BOTTOM, GOAL_POSITION, 2048); 
  delay(1000);
}

void setup() {

  Serial2.begin(57600);
  pinMode(BOARD_LED_PIN, OUTPUT);  //toggleLED_Pin_Out
  Dxl.begin(3);
  
  Dxl.jointMode(ID_TOP); 
  Dxl.jointMode(ID_BOTTOM);
  Dxl.maxTorque(ID_TOP, 900);
  Dxl.maxTorque(ID_BOTTOM, 900);
  // speed
  Dxl.writeWord(ID_TOP, 32, 80);
  Dxl.writeWord(ID_BOTTOM, 32, 80);
  getUp();
  
}
/* convert computed angle to motor position */
int convert(float angle) {
  int pos = 2048 + round((angle / 180.0) * 1024);    
  return pos;  
}

int is_following = 1;
/* Coordinates of the center of a face */
float x = 0, y = 0;
float last_x = 0, last_y = 0; // the previous position

/* Locker for motors */
int is_move = 1;

int random_state = ORIGIN;

int moving_finished = 1;

int cnt_detection = 0;
    
      int x_prev=2048, y_prev=2048;
void moveHead(float t = MIN_TIME + rand() % int(MAX_TIME - MIN_TIME) + 0.0) {
  
//    SerialUSB.println("----from:---");
//  
//    SerialUSB.println(last_x);
//    SerialUSB.println(last_y);
//  
//    SerialUSB.println("----to:-----");
//    SerialUSB.println(x);
//    SerialUSB.println(y);
  
    float real_dist = sqrt((last_x - x) * (last_x - x) + (last_y - y) * (last_y - y));
  
    cnt_detection++;
    if (cnt_detection > 45)
      cnt_detection = 45;
  
//    if (cnt_detection > 40 && real_dist > THRESHOLD_LENGTH) {
//      moving_finished = 1;
//      return;
//    }
  
    x = 0.9 * last_x + 0.1 * x;
    y = 0.9 * last_y + 0.1 * y;
  
    moving_finished = 0;
    float dist = sqrt((last_x - x) * (last_x - x) + (last_y - y) * (last_y - y));
      if (!(last_x==0&&last_y==0) && dist <= MIN_THRESHOLD_LENGTH) {
       moving_finished=1;
       return;
      }


//      t /= abs(200-dist);

      //SerialUSB.println("moving");
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
        {6*t, 12*t_2, 20*t_3, 0.0}
      };
      float ay[3][4] = {
        {t_3, t_4, t_5, y - y_a[0]},
        {3*t_2, 4*t_3, 5*t_4, 0.0},
        {6*t, 12*t_2, 20*t_3, 0.0}
      };
      gauss(ax, x_a);  // x_a(t) is 5th degree polynomial position profile
      gauss(ay, y_a);  // x_a(t) is 5th degree polynomial position profile
      
      
      /* modeling motion using x_a(t) and y_a(t) position profiles */
      for (float tm = 0.0, cnt=0; tm <= t; tm += SAMPLE_TIME, cnt++) {
      
        float x_t = x_a[0], y_t = y_a[0];
        float tt = tm * tm * tm;
        for (int i = 3; i <= 5; i++)
          x_t += x_a[i - 2] * tt, y_t += y_a[i - 2] * tt, tt *= tm;
   
    //SerialUSB.println(x_t);
    //SerialUSB.println(y_t);
    
     
    
        float angle_TOP = y_t * FV_Y / HEIGHT;
        float angle_BOTTOM = x_t * FV_X / WIDTH;
        
        int pos_BOTTOM = convert(angle_BOTTOM);
        int pos_TOP = convert(angle_TOP);
        
        if (pos_TOP > 2150)
          pos_TOP = 2150;
    
//       if ((pos_TOP - y_prev) * (y-last_y) < 0)
//        continue;
      if ((pos_BOTTOM - x_prev) * (last_x - x) > 0)
        continue;
    
      y_prev=pos_TOP;
      x_prev=pos_BOTTOM;
        Dxl.writeWord(ID_TOP, GOAL_POSITION, pos_TOP);    
        Dxl.writeWord(ID_BOTTOM, GOAL_POSITION, pos_BOTTOM);
       
        Dxl.flushPacket();
        if(!Dxl.getResult()){
          SerialUSB.println("Comm Fail");
        }  
    }
//      float angle_TOP = y * FV_Y / HEIGHT;
//        float angle_BOTTOM = x * FV_X / WIDTH;
//        
//        
//        int pos_BOTTOM = convert(angle_BOTTOM);
//        int pos_TOP = convert(angle_TOP);
//        
//        SerialUSB.println(pos_BOTTOM);
//        
//        Dxl.writeWord(ID_TOP, GOAL_POSITION, pos_TOP);    
//        Dxl.writeWord(ID_BOTTOM, GOAL_POSITION, pos_BOTTOM);
    moving_finished = 1;
    last_x = x;
    last_y = y;
    
}

int is_random_movement = 0;
unsigned long last_time = 0, delta_time = 0;

int writing_state = 0;

void loop() { 
   if(Serial2.available() && moving_finished == 1){
      toggleLED();
      char ch = (char)Serial2.read();       
      if (ch == 'x') {
        writing_state = WRITES_X;
        x = 0;
      } else
      if (ch == 'y') {
        writing_state = WRITES_Y;
        y = 0;
      } else
      if (ch == '$') {
        writing_state = GOT_XY;
      } else
      if (ch-'0'>=0&&ch-'0'<=9){
        if (writing_state == WRITES_X) {
           x = x * 10 + (ch - '0');           
        }
        else
        if (writing_state == WRITES_Y) {
           y = y * 10 + (ch - '0');
        }
      } else
        writing_state = 0;
    }
    if (writing_state == GOT_XY){
      x -= (WIDTH / 2);
      y -= (HEIGHT / 2);
      
//  float temp = x;
//  x = y;
//  y = temp;
      
      x *= -1;
      //y *= -1;
     
      moveHead();
      writing_state = 0;
    } 
}



#define EPS 0
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
