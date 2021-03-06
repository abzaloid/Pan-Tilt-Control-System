#ifndef __DEFINITIONS__
#define __DEFINITIONS__

#define WRITES_X 1
#define WRITES_Y 2
#define GOT_XY 3

/* Serial device defines for dxl bus */
#define DXL_BUS_SERIAL1 1  //Dynamixel on Serial1(USART1)  <-OpenCM9.04

/* Dynamixel ID defines */
#define ID_BOTTOM 1
#define ID_TOP 2

/* Control table defines */
#define GOAL_POSITION 30
#define MOVING 46

/* Field of View of Camera's X and Y axes in degrees */
#define FV_X 150.0
#define FV_Y 120.0

/* Parameters of camera */
#define WIDTH  640.0
#define HEIGHT 480.0

/* ID's for semantic & easy use */
#define TOP 0
#define BOTTOM 1

/* minimum jerk time in ms */
#define MAX_TIME 500  // maximum time to change position  
#define MIN_TIME 300    // minimum time to change position
#define SAMPLE_TIME 5 // sample time to quantize the distance 

/* Random movement */
// Coordinates of the head
#define X_DEVIATION 6000
#define X_MEAN 0
#define Y_DEVIATION 10
#define Y_MEAN -1200
// Time to look at and wait for the next movement
#define MIN_FREE_TIME 2500
#define MAX_FREE_TIME 20000
// Time to look at and wait for the next movement
#define MIN_FREE_AROUND_TIME 800
#define MAX_FREE_AROUND_TIME 2000
// The state of random position
#define ORIGIN 0
#define SIDE 1


/* Limitations of motor positions */
#define ORIGIN_POS 2048
#define MAX_POS 512

#define THRESHOLD_LENGTH 200
#define MIN_THRESHOLD_LENGTH 5




#endif
