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
	
	int8_t retVal = 0;
	retVal += rcs_enable(1, 1);
	
	if(retVal != 0) {		// enabling error
		
		uint8_t retStr[] = "Error: Servo init";
		UART_Send(LPC_USART0, retStr, 18, BLOCKING);
	}
	
	uint8_t *frame = (uint8_t *)SRAM1_LOC;
	uint8_t *frameloc = (uint8_t *)(SRAM1_LOC + 0);
	uint8_t sendPositions[RES_WIDTH];
	
//	uint8_t floorArray[80][13];
	
	float theta;
	
	// recieve the command to get a frame
	while(1) {
		
		// clear array (front edge)
		for(uint16_t x = 0; x < 320; x++) {
			sendPositions[x] = 255;
		}
		
		// clear array (floor array)
		
//		for(uint16_t y = 0; y < 13; y++) {
//			for(uint16_t x = 0; x < 72; x++) {
//				floorArray[x][y] = 0;
//			}
//		}
		
		
		
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
			
				uint16_t blobLen = 0;
				uint8_t inBlob = 0;
				uint16_t blobXPos = 0;
				uint8_t count = 0;
			
			// for a set angle of 12, we go to an angle of 62 degrees
			// make sure that theta is casted as a float
			theta = (float)(theta*(3.14159/180.0));
			
			// green LED, lets go!
			led_setRGB(255, 255, 255);
			
		// grab frame
					led_setRGB(0, 0, 0);
			cam_getFrame(frameloc, SRAM1_SIZE, CAM_GRAB_M1R2, 0, 0, RES_WIDTH, RES_HEIGHT);
				led_setRGB(255, 255, 255);
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

										float gradx = (3*(intense_XPO_YPO + GRAD_CO*intense_XPO_Y 
					+ intense_XPO_YMO - intense_XMO_YPO 
					- GRAD_CO*intense_XMO_Y - intense_XMO_YMO));
						
				float grady = (3*(intense_XMO_YMO + GRAD_CO*intense_X_YMO
					+ intense_XPO_YMO - intense_XMO_YPO 
					- GRAD_CO*intense_X_YPO - intense_XPO_YPO));
					
								// Threashold detection
				float grad = abs(gradx) + grady;
								// Threashold detection
				
				
			
				if( (grad > THREASHOLD_LOW) && (gradx < THREASHOLD_HIGH) ) {
					// EDGE
					frameloc[y*RES_WIDTH + x] = 255;		// sets the red pixel to max
				}
				//else if(-grady > THREASHOLD_LOW && -gradx < THREASHOLD_HIGH) {
				//	frameloc[(y+1)*RES_WIDTH + x] = 255;
				//}
				else {
					// NO EDGE
					frameloc[y*RES_WIDTH + x] = 0;			// turns off the red 
				} 
				
				// BLOB DETECTION
				if(gradx > THREASHOLD_LOW) {			// if we've entered a blob (black to white transistion, going left 2 right)
					inBlob = 1;
					blobLen++;
					blobXPos = x;
				}
				if(inBlob && (-gradx < THREASHOLD_LOW)) {	// if we're in a blob and haven't reached the outer edge
					blobLen++;
				}
				if(inBlob && (-gradx > THREASHOLD_LOW)) {	// in a blob, but exiting
					
					if(blobLen < MAX_BLOB_LEN) {
						// we've found a blob! DELETEEEEE
						//for(uint16_t blobX = blobXPos; blobX >= x; blobX += 2) {
					//		frameloc[y*RES_WIDTH + blobX] = 0;
					//	}
						frameloc[y*RES_WIDTH + blobXPos] = 0;
						frameloc[y*RES_WIDTH + x] = 0;
//						if( x < RES_WIDTH -OFFSET) {
//							frameloc[y*RES_WIDTH + x + 2] = 0;
//						}
//						if( x > OFFSET) {
//							frameloc[y*RES_WIDTH + blobXPos - 2] = 0;
//						}
						blobLen = inBlob = 0;
					}
					else {
						// clear vals, no blob here
						blobLen = inBlob = 0;
					}
				}
				
			}
		} // end nested for loop
		
			
	// noise pixel filtering
			

			for(uint16_t y = 1 + OFFSET; y < (RES_HEIGHT - OFFSET); y += 2) {
				uint16_t ypt = y + 2;
				uint16_t ymt = y - 2;
			
				for(uint16_t x = 1 + OFFSET; x < (RES_WIDTH - OFFSET); x += 2) {
					
					if(frameloc[y*RES_WIDTH + x] == 255) {		// if current pix. == on, check if it should be off
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
						
						if(frameloc[ymt*RES_WIDTH + xpt] == 0) 
							numOfPxOff++;
						
						if(frameloc[ymt*RES_WIDTH + xmt] == 0) 
							numOfPxOff++;
						
						if(frameloc[(ypt)*RES_WIDTH + xpt] == 0) 
							numOfPxOff++;
						
						if(frameloc[(ypt)*RES_WIDTH + xmt] == 0) 
							numOfPxOff++;
						
						if(numOfPxOff > 4) {
							
							frameloc[y*RES_WIDTH + x] = 0; 				// we only ever look at this pixel
						}
						
					} //end if(edge detected)
				} // end x for
			} // end y for
		
		// END NOISE FILTERING
			
	  // front edge detection
			
			for( float x = 1 + GND_OFFSET_X; x < (RES_WIDTH - GND_OFFSET_X); x += 2.0) 
			{																												// start on the left
				float xPos;
				
				for( float y = (RES_HEIGHT - GND_OFFSET_Y - 1); y > GND_OFFSET_Y; y -= 2.0) 
				{																											// start from the bottom
					
						float yPos;
					
					if(frameloc[((uint16_t)y)*RES_WIDTH + (uint16_t)x] != 0) {			// if the pixel is on

										// ACTUAL LOCATION CALCULATION
						double theta_ph = atan(((2.0*y-200.0)/200.0)*TAN_FOVH_DIV_2);		// angle of the pixel
						double cos_theta_ph = cos(theta_ph);											// used in the computations
						double cos_theta_minus_ph = cos(theta - theta_ph);				// used in the computations
						
						yPos = ((double)((3.9)*((cos_theta_ph))))/(cos_theta_minus_ph) + 
											(2.1)*tan(theta - theta_ph);		// y distance from the bot
						xPos = ((yPos*(2.0*x - 320.0))/320.0)*TAN_FOVW_DIV_2;			// x distance from the bot
						
						sendPositions[2*count] = (int8_t)(xPos+128);
						sendPositions[2*count + 1] = (int8_t)yPos;
						
						count++;
						
					//if(frameloc[((int16_t)y)*RES_WIDTH + (int16_t)x] != 0) {
						
						//frameloc[((int16_t)y-2)*RES_WIDTH + (int16_t)x - 1] = 0xFF;
										
						break;			// stop looking for the edge, break to the next x co-ordinate
					} // end if
					
				} // end yfor 
			} 	// end xfor
			
		// end front edge detection
			
			if(count == 0) {					// we have not detected any edges. Oh no!
				uint8_t noEdges = 42;
				UART_Send(LPC_USART0, &noEdges, 1, BLOCKING); 
			}
			
	//		UART_Send(LPC_USART0, sendPositions, 2*count, BLOCKING);	// sends x,y pairs!!!!
			
			
			/////// BEGIN FLOOR PACKING//////////////////
			
			// we want to send out a byte packed (in the y direction) map of where we can and cannot drive.
			// this array will have each cell (bit) represent a 2x2 cm square on the floor. We will loop through
			// the front edge (sendPositions) array and place it in the map accordingly. First pass, we'll just
			// add up how many fall into each block, then we will threashold that (weighted by how close the 
			// block is to the robot) and byte pack it and send it out!
			//
			// the floor will be 80cm (in the y direction) by 80 cm (x direction), giving us an array of 40x40 
			// when uncompressed and 5 by 40 when compressed (byte packed in the y direction).
			uint8_t uncompressedFloorArray[1600];
			
			for(uint32_t i = 0; i < 1600; i++) {
				uncompressedFloorArray[i] = 0;
			}
			
			// first, put the front edge in the floor array
			for(uint16_t i = 0; i < 2*count; i += 2) {
				//if( (sendPositions[i] - 128) + 40) < 40 && (sendPositions[i+1] < 80)) {
					
					uncompressedFloorArray[(((sendPositions[i] - 128) + 40) >> 1) + 40*(sendPositions[i+1] >> 1)] = 1;		// increment that floor location
					//uint8_t rxbuf = (sendPositions[i] - 128 + 40) >> 1;
					uint8_t rxbuf = sendPositions[i+1] >> 1;
					//UART_Send(LPC_USART0, &rxbuf, 1, BLOCKING);
			//	}
			}
			uint8_t aboveFrontEdge = 0;
			
			// a possible interum step: threasholding?
			
			// next, fill in above the front edge
			for(uint16_t x = 0; x < 40; x++) {
				for(uint16_t y = 0; y < 40; y++) {
					
					if(uncompressedFloorArray[x + y*40] >= 1 && !aboveFrontEdge) {
						aboveFrontEdge = 1;
					}
					else if(aboveFrontEdge) {
						uncompressedFloorArray[x + y*40] = 1;
						
					}
					
				} // x for
				
				aboveFrontEdge = 0;
			}	// y for
			
			// Now, byte pack into SendArray
			
			count = 0;
			
		for(uint16_t x = 0; x < 320; x++) {
			sendPositions[x] = 0;
		}
			
			for(uint16_t y = 0; y < 40; y++) {
				for(uint16_t x = 0; x < 40; x++) {
					
					if(uncompressedFloorArray[x + y*40] >= 1) {
						// can't drive here
						sendPositions[x + (y/8)*40] &= (!(1 << (y%8)));
					}
					else {	// can drive here															         --|----- 1 is bit shifted the mod of y
						sendPositions[x + (y/8)*40] |= (1 << (y%8));			//byte | 0b00100000
					}
				}
			}
			
			
			UART_Send(LPC_USART0, sendPositions, 200, BLOCKING);
			//UART_Send(LPC_USART0, uncompressedFloorArray, 1600, BLOCKING);
			
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
	
		led_setRGB(255, 0, 255);	// Purple LED
	}
			led_setRGB(0, 0, 0);	// Purple LED

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
