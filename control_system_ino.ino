/* Serial device defines for dxl bus */
#define DXL_BUS_SERIAL1 1  //Dynamixel on Serial1(USART1)  <-OpenCM9.04

/* Dynamixel ID defines */
#define ID_BOTTOM 1
#define ID_TOP 2

/* Control table defines */
#define GOAL_POSITION 30



/* Field of View of Camera's X and Y axes in degrees */
#define FV_X 45.0
#define FV_Y 45.0

/* Parameters of camera */
#define WIDTH  640.0
#define HEIGHT 480.0

/* ID's for semantic & easy use */
#define TOP 0
#define BOTTOM 1

/* Used for detecting high-frequent noises */
#define THRESHOLD_SQR_LENGTH 100*100

/* Used for smooth motion (FIR) */
#define PREVIOUS 0.9

Dynamixel Dxl(DXL_BUS_SERIAL1);

/* Trapezoidal pattern */
word my_time = 0;
#define RAMP_TIME 5
#define CONSTANT_TIME 5
#define STEP_SPEED 10

/* Used for motor lock */
byte motor_lock = 0;

void setup() {


  // Dynamixel 2.0 Baudrate -> 0: 9600, 1: 57600, 2: 115200, 3: 1Mbps 
  Dxl.begin(3);
  /* jointMode() is to use position mode */
  Dxl.jointMode(ID_TOP); 
  Dxl.jointMode(ID_BOTTOM);
  
  
}
/* convert computed angle to motor position */
word convert(float angle, byte id);

void loop() {  
  
  /* Test DYNAMIXELS */
  
//  //Turn dynamixel ID 1 to position 0  
//  Dxl.writeWord(ID_BOTTOM, GOAL_POSITION, 0); 
//  // Wait for 1 second (1000 milliseconds)
//  delay(1000);         
//  //Turn dynamixel ID 1 to position 300
//  Dxl.writeWord(ID_BOTTOM, GOAL_POSITION, 300);
//  // Wait for 1 second (1000 milliseconds)
//  delay(1000);              
  
  /* spiral movement */  
    
  float theta;  
  float last_x = 0, last_y = 0;
  
  for (theta = 0.0; ; theta += 0.1) {
    float r = 0.5 * theta;
    float x = r * cos(theta), y = r * sin(theta);


    /* used for smooth motion */    
    x = PREVIOUS * last_x + (1 - PREVIOUS) * x;
    y = PREVIOUS * last_y + (1 - PREVIOUS) * y;

    /* check if high frequency found */
    if ((x - last_x) * (x - last_x) + (y - last_y) * (y - last_y) >= THRESHOLD_SQR_LENGTH) {
      // TODO
      continue;
    }
    
    /* check for not exceeding boundaries */
    if (abs(x * 2) >= WIDTH || abs(y * 2) >= HEIGHT) {
      // TODO
      break;
    }
        
    float angle_TOP = y * FV_y / HEIGHT;
    float angle_BOTTOM = x * FV_x / WIDTH;
    
    Dxl.writeWord(ID_TOP, GOAL_POSITION, convert(angle_TOP, TOP));    
    delay(300);
    Dxl.writeWord(ID_BOTTOM, GOAL_POSITION, convert(angle_BOTTOM, BOTTOM));
    delay(300);
    
    last_x = x, last_y = y;
    
  }  
  
}

word convert(float angle, byte id) {
  word pos = round(angle * 1024.0 / 360.0);    
  if (id == TOP) {
    if (pos < 0) {
      // TODO      
    }
  } else 
  if (id == BOTTOM) {
    if (pos < 0) {
      // TODO
    }
  }  
}

/* S-curve speed control */
void scurve () {
  /* trapezoidal pattern */
  
  word speed;
  if (my_time <= RAMP_TIME)
    speed += STEP_SPEED;
  else if (my_time > RAMP_TIME + CONSTANT_TIME) 
    speed -= STEP_SPEED;
  
  /* Increase time which has period of the trapezoid length */
  my_time = (my_time + 1) % (2 * RAMP_TIME + CONSTANT_TIME);
}



/* Check if motors positions are well */
byte check_position() {
  // TODO
}
