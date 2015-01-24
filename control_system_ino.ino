/* Serial device defines for dxl bus */
#define DXL_BUS_SERIAL1 1  //Dynamixel on Serial1(USART1)  <-OpenCM9.04

/* Dynamixel ID defines */
#define ID_BOTTOM 1
#define ID_TOP 2

/* Control table defines */
#define GOAL_POSITION 30
#define MOVING 46

/* Field of View of Camera's X and Y axes in degrees */
#define FV_X 45.0
#define FV_Y 45.0

/* Parameters of camera */
#define WIDTH  1280.
#define HEIGHT 800.0

/* ID's for semantic & easy use */
#define TOP 0
#define BOTTOM 1

/* Used for detecting high-frequent noises */
#define THRESHOLD_SQR_LENGTH 100*100

/* Used for smooth motion (FIR) */
#define PREVIOUS 0.9

Dynamixel Dxl(DXL_BUS_SERIAL1);

/* Trapezoidal pattern */
int my_time = 0;
#define RAMP_TIME 5
#define CONSTANT_TIME 5
#define STEP_SPEED 10

/* Used for motor lock */
int motor_lock = 0;

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
int convert(float angle, int id);

float x = 0, y = 0;
float step_x = 0, step_y = 0;

void usbInterrupt(byte* buffer, byte nCount){
  SerialUSB.print("nCount =");
  SerialUSB.println(nCount);
  for(unsigned int i=0; i < nCount;i++)  //printf_SerialUSB_Buffer[N]_receive_Data
    SerialUSB.print((char)buffer[i]);
    
//  if ((char)buffer[0] == '8')
//    step_y += 1;
//  else
//  if ((char)buffer[0] == '2')
//    step_y -= 1;
//  else
//  if ((char)buffer[0] == '6')
//    step_x -= 1;
//  else
//  if ((char)buffer[0] == '4')
//    step_x += 1;
  
  if ((char)buffer[0] == '1') {
    x = -(rand()%6700);
    y = +(rand()%3300);
  } else
  if ((char)buffer[0] == '2') {
    x = +(rand()%6700);
    y = +(rand()%3300);
  } else
  if ((char)buffer[0] == '3') {
    x = +(rand()%6700);
    y = -(rand()%3300);
  } else
  if ((char)buffer[0] == '4') {
    x = -(rand()%6700);
    y = -(rand()%3300);
  } else {
    x = 0;
    y = 0;
  }
  
  SerialUSB.println("");
}

void loop() {  
    
  float theta;  
  
  toggleLED();
  //  x += step_x * 20;
  //  y += step_y * 20;
  
  
  
  //  SerialUSB.println(step_x);
 //   SerialUSB.println(step_y);
    SerialUSB.println("-----------------------------------");
    delay(2000);

    /* check if high frequency found */
    //if ((x - last_x) * (x - last_x) + (y - last_y) * (y - last_y) >= THRESHOLD_SQR_LENGTH) {
      // TODO
      //continue;
    //}
    
    /* check for not exceeding boundaries */
    if (abs(x * 2) >= WIDTH || abs(y * 2) >= HEIGHT) {
      // TODO
      //continue;
    }
        
    float angle_TOP = y * FV_Y / HEIGHT;
    float angle_BOTTOM = x * FV_X / WIDTH;
    
    int pos_BOTTOM = convert(angle_BOTTOM, BOTTOM);
    int pos_TOP = convert(angle_TOP, TOP);
    
    SerialUSB.print("pos_TOP = ");
    SerialUSB.print(pos_TOP);
    SerialUSB.print(" pos_BOTTOM = ");
    SerialUSB.println(pos_BOTTOM);
    
    SerialUSB.print("x = ");
    SerialUSB.print(x);
    SerialUSB.print(" y = ");
    SerialUSB.println(y);
    
    Dxl.writeWord(ID_TOP, GOAL_POSITION, pos_TOP);    
    Dxl.writeWord(ID_BOTTOM, GOAL_POSITION, pos_BOTTOM);
    
    Dxl.flushPacket();
  
    if(!Dxl.getResult()){
      SerialUSB.println("Comm Fail");
    }
}

int convert(float angle, int id) {
  int pos = 2048 + round((angle / 360.0) * 1024);    
  return pos;  
}

/* S-curve speed control */
void scurve () {
  /* trapezoidal pattern */
  
  int speed;
  if (my_time <= RAMP_TIME)
    speed += STEP_SPEED;
  else if (my_time > RAMP_TIME + CONSTANT_TIME) 
    speed -= STEP_SPEED;
  
  /* Increase time which has period of the trapezoid length */
  my_time = (my_time + 1) % (2 * RAMP_TIME + CONSTANT_TIME);
}



/* Check if motors positions are well */
int check_position() {
  // TODO
}
