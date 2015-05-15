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
#include "edgedetect.h"
#include "led.h"
#include "pixyvals.h"
#include <math.h>


// tangent calculations used in triangulation calculations
#define TAN_FOVH_DIV_2 0.435
#define TAN_FOVW_DIV_2 0.767

// NOT DEFINED TO THE RIGHT VALUES YET
// units in cm
#define H_C 3.6
#define H_B 4.1

extern uint8_t UART_DATA_AVAILABLE;			// global interrupt that gets the byte from the PIC


void edgeDetect_run()
{
	uint8_t brightness = 100;
	cam_setBrightness(brightness); 				// 0 to 255. Camera brightness setting
	
	int8_t retVal = 0;
	retVal += rcs_enable(1, 1);
	
	if(retVal != 0) {		// enabling error
		
		uint8_t retStr[] = "Error: Servo init";
		UART_Send(LPC_USART0, retStr, 18, BLOCKING);
	}
	
	uint8_t *frame = (uint8_t *)SRAM1_LOC;
	uint8_t *frameloc = (uint8_t *)(SRAM1_LOC + 0);
	uint8_t *sendPositions = (uint8_t *)SRAM1_LOC;
	
	uint8_t floorArray[80][13];
	
	float theta;
	uint16_t count = 0;
	
	// recieve the command to get a frame
	while(1) {
		
		// clear array (front edge)
		for(uint16_t x = 0; x < 320; x++) {
			sendPositions[x] = 255;
		}
		
		// clear array (floor array)
		
		for(uint16_t y = 0; y < 13; y++) {
			for(uint16_t x = 0; x < 72; x++) {
				floorArray[x][y] = 0;
			}
		}
		
		
		
		// red LED: Stopped waiting for data
		led_setRGB(255, 0, 0);
		
		while(1) {
			if(UART_DATA_AVAILABLE) {		// Data has come!
				theta = (float)UART_DATA_AVAILABLE;
				UART_DATA_AVAILABLE = 0;
				break;
			}
		}
		
		if(theta > 42) {
			count = 0;
			// make sure that theta is casted as a float
			theta = (float)(theta*(3.14159/180.0));
			
			// green LED, lets go!
			led_setRGB(255, 255, 255);
			
		// grab frame
			cam_getFrame(frameloc, SRAM1_SIZE, CAM_GRAB_M1R2, 0, 0, RES_WIDTH, RES_HEIGHT);
		
			frameloc = frame;			
		
		// double for loop for calculating edges
			for(uint16_t y = 1 + OFFSET; y < (RES_HEIGHT - OFFSET); y += 2) {
				uint16_t ypo = y + 1;
				uint16_t ymo = y - 1;
//				uint16_t ymt = y - 2;
//				uint16_t ypt = y + 2;
				
				for(uint16_t x = 1 + OFFSET; x < (RES_WIDTH - OFFSET); x += 2) {
					uint16_t xpo = x + 1;
					uint16_t xmo = x - 1;
//					uint16_t xpt = x + 2; 
//					uint16_t xmt = x - 2;
					
				// Gradient/intensity calculation
					
					// intensity calculation for the pixel groups. each "pixel" we use is actually the intensity
					// calculated based off of a group of four pixels. This is for speed, accuracy, and clean
					// edges.

					uint16_t intense_XPO_Y = frameloc[y*RES_WIDTH + xpo] + frameloc[ypo*RES_WIDTH + xpo+1] + 
							(frameloc[ypo*RES_WIDTH + xpo] + frameloc[y*RES_WIDTH + xpo+1])/2;
					
					uint16_t intense_XMO_Y = frameloc[y*RES_WIDTH + xmo] + frameloc[ypo*RES_WIDTH + x] + 
							(frameloc[ypo*RES_WIDTH + xmo] + frameloc[y*RES_WIDTH + x])/2;
					
					uint16_t intense_X_YPO = frameloc[ypo*RES_WIDTH + x] + frameloc[(ypo+1)*RES_WIDTH + xpo] + 
							(frameloc[(ypo+1)*RES_WIDTH + x] + frameloc[ypo*RES_WIDTH + xpo])/2;
					
					uint16_t intense_XPO_YPO = frameloc[ypo*RES_WIDTH + xpo] + frameloc[(ypo+1)*RES_WIDTH + xpo+1] + 
							(frameloc[(ypo+1)*RES_WIDTH + xpo] + frameloc[ypo*RES_WIDTH + xpo+1])/2;
					
					uint16_t intense_XMO_YPO = frameloc[(ypo)*RES_WIDTH + xmo] + frameloc[(ypo+1)*RES_WIDTH + x] + 
							(frameloc[(ypo+1)*RES_WIDTH + xmo] + frameloc[ypo*RES_WIDTH + x])/2;
					
					uint16_t intense_X_YMO = frameloc[ymo*RES_WIDTH + x] + frameloc[y*RES_WIDTH + xpo] + 
							(frameloc[y*RES_WIDTH + x] + frameloc[ymo*RES_WIDTH + xpo])/2;
							
					uint16_t intense_XPO_YMO = frameloc[ymo*RES_WIDTH + xpo] + frameloc[y*RES_WIDTH + xpo+1] + 
							(frameloc[y*RES_WIDTH + xpo] + frameloc[ymo*RES_WIDTH + xpo+1])/2;
							
					uint16_t intense_XMO_YMO = frameloc[ymo*RES_WIDTH + xmo] + frameloc[y*RES_WIDTH + x] + 
							(frameloc[y*RES_WIDTH + xmo] + frameloc[ymo*RES_WIDTH + x])/2;

							
					/*   ORIGINAL
					float gradx = abs(intense_XPO_Y - intense_XMO_Y
						+ intense_XPO_YPO - intense_XMO_YPO
						+ intense_XPO_YMO - intense_XMO_YMO);
						
					float grady = abs(intense_X_YPO -	intense_X_YMO
						+ intense_XPO_YPO -	intense_XPO_YMO
						+ intense_XMO_YPO - intense_XMO_YMO);
				*/
#ifdef THREASHOLD_NORMAL

				float grady = (1*(intense_XPO_YPO + GRAD_CO*intense_XPO_Y 
					+ intense_XPO_YMO - intense_XMO_YPO 
					- GRAD_CO*intense_XMO_Y - intense_XMO_YMO));
						
				float gradx = (3*(intense_XMO_YMO + GRAD_CO*intense_X_YMO
					+ intense_XPO_YMO - intense_XMO_YPO 
					- GRAD_CO*intense_X_YPO - intense_XPO_YPO));
				
#else
				
				float grady = (3*(-intense_XPO_YPO - GRAD_CO*intense_XPO_Y 
					- intense_XPO_YMO + intense_XMO_YPO 
					+ GRAD_CO*intense_XMO_Y + intense_XMO_YMO))/GRAD_THREASHOLD;
						
				float gradx = (3*(intense_XMO_YMO + GRAD_CO*intense_X_YMO
					+ intense_XPO_YMO - intense_XMO_YPO 
					- GRAD_CO*intense_X_YPO - intense_XPO_YPO))/GRAD_THREASHOLD;
#endif
					
								// Threashold detection
				float grad = gradx+grady;
								// Threashold detection
					
#ifdef THREASHOLD_NORMAL
					if( (grad > THREASHOLD_LOW) && (grad < THREASHOLD_HIGH) ) {
#else
					if(gradx*gradx + grady*grady > 1) {
#endif
						// EDGE
						// frameloc[ymo*RES_WIDTH + xmo] = 255;
						// frameloc[y*RES_WIDTH + xmo] = 255;
						// frameloc[ymo*RES_WIDTH + x] = 255;
						
						frameloc[y*RES_WIDTH + x] = 255;
					}
					else {
						// NO EDGE
						// frameloc[ymo*RES_WIDTH + xmo] = 0;
						// frameloc[y*RES_WIDTH + xmo] = 0;
						// frameloc[ymo*RES_WIDTH + x] = 0;
						
						frameloc[y*RES_WIDTH + x] = 0;
					}
				}
			} // end nested for loop
			
			
			// begin filtering. Looks at each pixel, looks around that pixel (if it is on), and if it is 
			// surounded by mostly off pixels, it also turns that pixel off. Filters out non-lines from our
			// detection.

			for(uint16_t y = 1 + OFFSET; y < (RES_HEIGHT - OFFSET); y += 2) {
				uint16_t ypt = y + 2;
				uint16_t ymt = y - 2;
			
				for(uint16_t x = 1 + OFFSET; x < (RES_WIDTH - OFFSET); x += 2) {
					
					if(frameloc[y*RES_WIDTH + x] != 0) {		// if current pix. == on, check if it should be off
						uint16_t xpt = x + 2;
						uint16_t xmt = x - 2;
						
						uint8_t numOfPxOff = 0;
						
						if(frameloc[y*RES_WIDTH + xpt] == 0) 
							numOfPxOff++;
						
						if(frameloc[y*RES_WIDTH + xmt] == 0) 
							numOfPxOff++;
						
						if(frameloc[(ypt)*RES_WIDTH + x] == 0) 
							numOfPxOff++;
						
						if(frameloc[(ymt)*RES_WIDTH + x] == 0) 
							numOfPxOff++;
						
						if(numOfPxOff > 2) {
							// frameloc[(y-1)*RES_WIDTH + x-1] = 0;
							// frameloc[y*RES_WIDTH + x-1] = 0;
							// frameloc[(y-1)*RES_WIDTH + x] = 0;
							frameloc[y*RES_WIDTH + x] = 0; 				// we only ever look at this pixel
						}
						
					} //end if(edge detected)
				} // end x for
			} // end y for
			
			
			// floor detection & distance extrapolation.
			
			for(float x = (POS_OFFSET); x < (RES_WIDTH - POS_OFFSET); x += 2.0) {	// start on the left
				
				float xPos;
				
				for(float y = (RES_HEIGHT - POS_OFFSET); y > POS_OFFSET; y -= 2.0) {	// start from the bottom
					
					if(frameloc[((uint8_t)y)*RES_WIDTH + (uint16_t)x] != 0) {
						float yPos;
						
						double theta_ph = atan(((2.0*y-200.0)/200.0)*TAN_FOVH_DIV_2);		// angle of the pixel
						double cos_theta_ph = cos(theta_ph);											// used in the computations
						double cos_theta_minus_ph = cos(theta - theta_ph);				// used in the computations
						
						yPos = ((double)((3.9)*((cos_theta_ph))))/(cos_theta_minus_ph) + 
											(2.1)*tan(theta - theta_ph);		// y distance from the bot
						xPos = ((yPos*(2.0*x - 320.0))/320.0)*TAN_FOVW_DIV_2;			// x distance from the bot
						
						sendPositions[2*count] = (uint8_t)(xPos + 128);						// these two lines put x,y pairs
						sendPositions[2*count + 1] = (uint8_t)yPos;								// into the send array
						
						
						// floor array is a 76 cm wide by 104 cm long grid of where the robot can and cannot drive. 
						// It is byte packed in the y direction
						floorArray[(int8_t)(xPos + 36)] [((int8_t)yPos)/8] |= 1 << (((int8_t)yPos) % 8);
						
						count += 1;		// count of the number of edges of obsticles detected
						break;			// stop looking for the edge, break to the next x co-ordinate
					} // end if
					
				} //end yfor 
			} // end xfor
			
			if(count == 0) {					// we have not detected any edges. Oh no!
				uint8_t noEdges = 42;
				UART_Send(LPC_USART0, &noEdges, 1, BLOCKING); 
			}
			
			UART_Send(LPC_USART0, sendPositions, 2*count, BLOCKING);	// sends x,y pairs!!!!
			
			led_setRGB(255, 0, 255);	// Purple LED
			
			// Byte packing for processing script
/*
			for(uint16_t y = 0; y < RES_HEIGHT/2; y += 1) {
				for (uint16_t x = 0; x < RES_WIDTH/2; x += 8) {

						frameloc[y*(RES_WIDTH/16) + x/8] = (frameloc[(y*2+1)*RES_WIDTH + (2*(x+0)+1)] & 0x80) | 
																						(frameloc[(y*2+1)*RES_WIDTH + (2*(x+1)+1)] & 0x40) |
																						(frameloc[(y*2+1)*RES_WIDTH + (2*(x+2)+1)] & 0x20) |
																						(frameloc[(y*2+1)*RES_WIDTH + (2*(x+3)+1)] & 0x10) |
																						(frameloc[(y*2+1)*RES_WIDTH + (2*(x+4)+1)] & 0x08) |
																						(frameloc[(y*2+1)*RES_WIDTH + (2*(x+5)+1)] & 0x04) |
																						(frameloc[(y*2+1)*RES_WIDTH + (2*(x+6)+1)] & 0x02) |
																						(frameloc[(y*2+1)*RES_WIDTH + (2*(x+7)+1)] & 0x01); 

				}
			}
			// frame[0] = 'A';																		 // key byte
			// UART_Send(LPC_USART0, frameloc, 2001, BLOCKING);		 // Send the frame byte packed to see it in processing
*/
			
		} // end edge detecting
		
		
		
		else if(theta > 1 ) {	// Servo move routine
			uint16_t position;
			// Move the servo based on the input from the PIC
			// theta == 2 corrisponds to a 45 degree angle,
			// theta == 42 corrisponds to a 135 degree angle.
			position = theta;
			position = (position - 2)*(25);
			rcs_setPos(1, position);
		}
		
		else {		// theta == 1, they are asking for my ID
			
			// tell the processing script/pic/whatever that I am the pixy
			uint8_t ID[] = "I am the Pixy!\n\r";
			UART_Send(LPC_USART0, ID, 17, BLOCKING);
		}
	}
}


void toggleLED() {
	
			// Toggle LED to see when we're sending frames

	static uint8_t toggle = 0;
	toggle ^= 1;
	if(toggle) {
		led_setRGB(255, 255, 255);
	}
	else {
		led_setRGB(0, 0, 0);
	}
}
