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
#define WIDTH  1280.0
#define HEIGHT 800.0

/* ID's for semantic & easy use */
#define TOP 0
#define BOTTOM 1

/* minimum jerk time in ms */
#define MAX_TIME 1000  // maximum time to change position  
#define MIN_TIME 50    // minimum time to change position
#define SAMPLE_TIME 10 // sample time to quantize the distance 
