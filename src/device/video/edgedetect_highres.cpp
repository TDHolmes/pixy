//
// begin license header
//
// This file is part of Pixy CMUcam5 or "Pixy" for short
//
// All Pixy source code is provided under the terms of the
// GNU General Public License v2 (http://www.gnu.org/licenses/gpl-2.0.html).
// Those wishing to use Pixy source code, software and/or
// technologies under different licensing terms should contact us at
// cmucam@cs.cmu.edu. Such licensing terms are available for
// all portions of the Pixy codebase presented here.
//
// end license header
//

// 
// This is an additional file added by Tyler Holmes and Broderick Carlin
// of the University of Saint Thomas, adding the functionality of edge detection
// for the purposes of object avoidance on an autonomous vehicle. The function
// gets a frame, processes it using a simple edge detection formula, and returns
// a processed array.
//
// This specific implementation is designed for a maze navigating robot
// 


#include <stdio.h>
#include "progvideo.h"
#include "pixy_init.h"
#include "camera.h"
#include "conncomp.h"
#include <string.h>
#include "edgedetect.h"	//Tyler Edit
#include "rcservo.h"
#include "edgedetect_highres.h"
#include "led.h"
#include <math.h>


// tangent calculations used in triangulation calculations
#define TAN_FOVH_DIV_2 0.435
#define TAN_FOVW_DIV_2 0.767

// NOT DEFINED TO THE RIGHT VALUES YET
// units in cm
#define H_C 3.6
#define H_B 4.1

extern uint8_t UART_DATA_AVAILABLE;			// global interrupt that gets the byte from the PIC


void edgeDetect_highres_run()
{
	
	uint8_t *frame = (uint8_t *)SRAM1_LOC;
	uint8_t *frameloc = (uint8_t *)(SRAM1_LOC + 0);
	uint8_t *sendPositions = (uint8_t*)(SRAM1_LOC);
	float theta;
	
	// recieve the command to get a frame
	while(1) {
		
		// red LED: Stopped waiting for data
		led_setRGB(255, 0, 0);
		
		while(1) {
			if(UART_DATA_AVAILABLE) {		// Data has come!
				theta = (float)UART_DATA_AVAILABLE;
				UART_DATA_AVAILABLE = 0;
				break;
			}
		}
		
		if(theta > 45) {
			// make sure that theta is casted as a float
			theta = (float)(theta*(3.14159/180.0));
			
			// green LED, lets go!
			led_setRGB(0, 255, 0);
			
		// grab frame
			cam_getFrame(frameloc, SRAM1_SIZE, CAM_GRAB_M1R2, 0, 0, RES_WIDTH, RES_HEIGHT);
		
			frameloc = frame;			// 
		
		// double for loop for calculating edges
			for(uint16_t y = 1; y < (RES_HEIGHT); y += 1) {
				uint16_t ypo = y + 1;
				uint16_t ymo = y - 1;
				for(uint16_t x = 1; x < (RES_WIDTH); x += 1) {
					uint16_t xpo = x + 1;
					uint16_t xmo = x - 1;
					
				// Gradient calculation
					
					// intensity calculation for the pixel groups around each pixel
					uint16_t intense_XPO_Y;
					uint16_t intense_XMO_Y;
					uint16_t intense_X_YPO;
					uint16_t intense_XPO_YPO;
					uint16_t intense_XMO_YPO;
					uint16_t intense_X_YMO;
					uint16_t intense_XPO_YMO;
					uint16_t intense_XMO_YMO;
					
					if( ( (x % 2 == 0) && (y % 2 == 0) ) || ( (x % 2 == 1) && (y % 2 == 1) ) )  {			// We are on a blue or red pixel
						
						intense_XPO_Y = intensityCalc_GreenPixel(frameloc, xpo, y);
							
						intense_XMO_Y = intensityCalc_GreenPixel(frameloc, xmo, y);
						
						intense_X_YPO = intensityCalc_GreenPixel(frameloc, x, ypo);
						
						intense_XPO_YPO = intensityCalc_BlueRedPixel(frameloc, xpo, ypo);
						
					  intense_XMO_YPO = intensityCalc_BlueRedPixel(frameloc, xmo, ypo);
						
						intense_X_YMO = intensityCalc_GreenPixel(frameloc, x, ymo);
								
						intense_XPO_YMO = intensityCalc_BlueRedPixel(frameloc, xpo, ymo);
								
						intense_XMO_YMO = intensityCalc_BlueRedPixel(frameloc, xmo, ymo);
					} 
					else {			// We are on a green pixel
 
						intense_XPO_Y = intensityCalc_BlueRedPixel(frameloc, xpo, y);
							
						intense_XMO_Y = intensityCalc_BlueRedPixel(frameloc, xmo, y);
						
						intense_X_YPO = intensityCalc_BlueRedPixel(frameloc, x, ypo);
						
						intense_XPO_YPO = intensityCalc_GreenPixel(frameloc, xpo, ypo);
						
					  intense_XMO_YPO = intensityCalc_GreenPixel(frameloc, xmo, ypo);
						
						intense_X_YMO = intensityCalc_BlueRedPixel(frameloc, x, ymo);
								
						intense_XPO_YMO = intensityCalc_GreenPixel(frameloc, xpo, ymo);
								
						intense_XMO_YMO = intensityCalc_GreenPixel(frameloc, xmo, ymo);
						
					}
					
					float grad1 = abs(intense_XPO_Y - intense_XMO_Y
						+ intense_XPO_YPO - intense_XMO_YPO
						+ intense_XPO_YMO - intense_XMO_YMO);
						
					float grad2 = abs(intense_X_YPO -	intense_X_YMO
						+ intense_XPO_YPO -	intense_XPO_YMO
						+ intense_XMO_YPO - intense_XMO_YMO);
				
								// Threashold detection
					if( (grad1 + grad2) > THREASHOLD_LOW ) {
						// EDGE
						frameloc[y*RES_WIDTH + x] = 255;
					}
					else {
						// NO EDGE
						frameloc[y*RES_WIDTH + x] = 0;
					}
				}
			} // end nested for loop
				
			led_setRGB(255, 0, 255);	// Purple LED
			
				// floor detection & distance extrapolation
			uint16_t count = 0;
			for(float x = (POS_OFFSET); x < (RES_WIDTH - POS_OFFSET); x += 1.0) {	// start on the left
				
				float xPos;
				for(float y = (RES_HEIGHT - POS_OFFSET); y > POS_OFFSET; y -= 1.0) {	// start from the bottom
					
					if(frameloc[((uint16_t)y)*RES_WIDTH + (uint16_t)x] != 0) {
						float yPos;
						
						double theta_ph = atan(((2.0*y-200.0)/200.0)*TAN_FOVH_DIV_2);		// angle of the pixel
						double cos_theta_ph = cos(theta_ph);											// used in the computations
						double cos_theta_minus_ph = cos(theta - theta_ph);				// used in the computations
						
						yPos = ((double)((3.9)*((cos_theta_ph))))/(cos_theta_minus_ph) + 
											(2.1)*tan(theta - theta_ph);		// y distance from the bot
						xPos = (yPos*(2.0*x - 320.0))/417.0;			// x distance from the bot

						sendPositions[2*count] = xPos;				// these two lines send x,y pairs
						sendPositions[2*count + 1] = yPos;
						
						count++;		// count of the number of edges of obsticles detected
						break;			// stop looking for the edge, break to the next x co-ordinate
					}
					else {
						// color the floor a different color. Not used in this scenario
					}
				}
			}
			
			UART_Send(LPC_USART0, sendPositions, 2*count, BLOCKING);	// sends x,y pairs
			count = 0;
			
			// clear array
			for(uint16_t x = 0; x < 640; x++) {
				sendPositions[x] = 255;
			}
			
		} // end edge detecting
		
		else if(theta > 1 ) {	// Servo move routine
			uint16_t position;
			int8_t retVal;
			// Move the servo based on the input from the PIC
			// theta == 2 corrisponds to a 45 degree angle,
			// theta == 42 corrisponds to a 135 degree angle.
			
			position = (position - 2)*(25);
			retVal = rcs_setPos(1, position);
			
			if(retVal == 0) {
				
				uint8_t retStr[] = "successful Servo Move\n\r";
				UART_Send(LPC_USART0, retStr, 24, BLOCKING);
			}
			else {
				uint8_t retStr[] = "Error: Servo Move\n\r";
				UART_Send(LPC_USART0, retStr, 20, BLOCKING);
			}
		}
		else {		// theta == 1, they are asking for my ID
			
			// tell the processing script/pic/whatever that I am the pixy
			uint8_t ID[] = "I am the Pixy!\n\r";
			UART_Send(LPC_USART0, ID, 17, BLOCKING);
		}
	}
}



uint16_t intensityCalc_BlueRedPixel(uint8_t* frame, uint16_t x, uint16_t y) {
	
	return ((frame[y*RES_WIDTH + x] + 
		( frame[(y+1)*RES_WIDTH + x] + frame[(y-1)*RES_WIDTH + x] + frame[y*RES_WIDTH + x+1] +	frame[y*RES_WIDTH + x-1] ) / 4 + 
		(	frame[(y+1)*RES_WIDTH + x+1] + frame[(y+1)*RES_WIDTH + x-1] + frame[(y-1)*RES_WIDTH + x+1] + frame[(y-1)*RES_WIDTH + x-1] ) / 4) / 3);
}

uint16_t intensityCalc_GreenPixel(uint8_t* frame, uint16_t x, uint16_t y) {
	
	return ((( frame[(y+1)*RES_WIDTH + x] + frame[(y-1)*RES_WIDTH + x] )/2 + 
		( frame[y*RES_WIDTH + x+1] +	frame[y*RES_WIDTH + x-1] )/2 + 
		( frame[y*RES_WIDTH + x] + 	frame[(y+1)*RES_WIDTH + x+1] + frame[(y+1)*RES_WIDTH + x-1] + frame[(y-1)*RES_WIDTH + x+1] + frame[(y-1)*RES_WIDTH + x-1] ) / 5) / 3);
}

