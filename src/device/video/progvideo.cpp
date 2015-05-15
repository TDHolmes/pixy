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

#include <stdio.h>
#include "progvideo.h"
#include "pixy_init.h"
#include "pixyvals.h"
#include "camera.h"
//#include "colorlut.h"
#include "blobs.h"
#include "conncomp.h"
#include "param.h"
#include <string.h>
#include "edgedetect_highres.h"
#include <math.h>
#include "rcservo.h"
#include "pixyvals.h"


 
Program g_progVideo =
{
	"video",
	"continuous stream of raw camera frames",
	videoSetup, 
	videoLoop
};

int videoSetup()
{
	return 0;
}

extern uint8_t UART_DATA_AVAILABLE;			// global interrupt that gets the byte from the PIC

void sendCustom(uint8_t renderFlags=RENDER_FLAG_FLUSH)
{
	static int8_t theta = 0;
	int32_t len;
	uint8_t *frame = (uint8_t *)SRAM1_LOC;
	
	if(UART_DATA_AVAILABLE) {		// Data has come!
			theta = (float)UART_DATA_AVAILABLE;
			UART_DATA_AVAILABLE = 0;
	}
	
	if(theta > 1 ) {	// Servo move routine
			uint16_t position;
			// Move the servo based on the input from UART
			// range is input = 2 -> 42
			position = theta;
			position = (position - 2)*(25);
			rcs_setPos(1, position);
		}

		// fill buffer contents manually for return data 
		len = Chirp::serialize(g_chirpUsb, frame, SRAM1_SIZE, HTYPE(FOURCC('C','M','V','2')), HINT8(renderFlags), UINT16(CAM_RES2_WIDTH), UINT16(CAM_RES2_HEIGHT), UINTS8_NO_COPY(CAM_RES2_WIDTH*CAM_RES2_HEIGHT), END);
		// write frame after chirp args
		cam_getFrame(frame+len, SRAM1_SIZE-len, CAM_GRAB_M1R2, 0, 0, CAM_RES2_WIDTH, CAM_RES2_HEIGHT);

		uint8_t *frameloc = (uint8_t *)(SRAM1_LOC + len);
		
		// double for loop for calculating edges
			for(uint16_t y = 1 + OFFSET; y < (RES_HEIGHT - OFFSET); y += 2) {
				uint16_t ypo = y + 1;
				uint16_t ymo = y - 1;
				uint16_t ymt = y - 2;
				uint16_t ypt = y + 2;
				
				for(uint16_t x = 1 + OFFSET; x < (RES_WIDTH - OFFSET); x += 2) {
					uint16_t xpo = x + 1;
					uint16_t xmo = x - 1;
					uint16_t xpt = x + 2; 
					uint16_t xmt = x - 2;
					
				// Gradient calculation
					
					// intensity calculation for the pixel groups around each pixel
		
					uint16_t intense_XPO_Y = frameloc[y*RES_WIDTH + xpo] + frameloc[ypo*RES_WIDTH + xpt] + 
							(frameloc[ypo*RES_WIDTH + xpo] + frameloc[y*RES_WIDTH + xpt])/2;
					
					uint16_t intense_XMO_Y = frameloc[y*RES_WIDTH + xmo] + frameloc[ypo*RES_WIDTH + x] + 
							(frameloc[ypo*RES_WIDTH + xmo] + frameloc[y*RES_WIDTH + x])/2;
					
					uint16_t intense_X_YPO = frameloc[ypo*RES_WIDTH + x] + frameloc[(ypt)*RES_WIDTH + xpo] + 
							(frameloc[(ypt)*RES_WIDTH + x] + frameloc[ypo*RES_WIDTH + xpo])/2;
					
					uint16_t intense_XPO_YPO = frameloc[ypo*RES_WIDTH + xpo] + frameloc[(ypt)*RES_WIDTH + xpt] + 
							(frameloc[(ypt)*RES_WIDTH + xpo] + frameloc[ypo*RES_WIDTH + xpt])/2;
					
					uint16_t intense_XMO_YPO = frameloc[(ypo)*RES_WIDTH + xmo] + frameloc[(ypt)*RES_WIDTH + x] + 
							(frameloc[(ypt)*RES_WIDTH + xmo] + frameloc[ypo*RES_WIDTH + x])/2;
					
					uint16_t intense_X_YMO = frameloc[ymo*RES_WIDTH + x] + frameloc[y*RES_WIDTH + xpo] + 
							(frameloc[y*RES_WIDTH + x] + frameloc[ymo*RES_WIDTH + xpo])/2;
							
					uint16_t intense_XPO_YMO = frameloc[ymo*RES_WIDTH + xpo] + frameloc[y*RES_WIDTH + xpt] + 
							(frameloc[y*RES_WIDTH + xpo] + frameloc[ymo*RES_WIDTH + xpt])/2;
							
					uint16_t intense_XMO_YMO = frameloc[ymo*RES_WIDTH + xmo] + frameloc[y*RES_WIDTH + x] + 
							(frameloc[y*RES_WIDTH + xmo] + frameloc[ymo*RES_WIDTH + x])/2;
				

				float grady = (3*(intense_XPO_YPO + GRAD_CO*intense_XPO_Y 
					+ intense_XPO_YMO - intense_XMO_YPO 
					- GRAD_CO*intense_XMO_Y - intense_XMO_YMO));
						
				float gradx = (3*(intense_XMO_YMO + GRAD_CO*intense_X_YMO
					+ intense_XPO_YMO - intense_XMO_YPO 
					- GRAD_CO*intense_X_YPO - intense_XPO_YPO));
					
								// Threashold detection
				float grad = gradx+grady;
								// Threashold detection
					
				
					if( (grad > THREASHOLD_LOW) && (grad < THREASHOLD_HIGH) ) {
						// EDGE
						frameloc[y*RES_WIDTH + x] = 255;		// sets the red pixel to max
					}
					else {
						// NO EDGE
						frameloc[y*RES_WIDTH + x] = 0;			// turns off the red 
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
						
						if(numOfPxOff > 5) {
							
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
					
					if(frameloc[((uint8_t)y)*RES_WIDTH + (uint16_t)x] != 0) {			// if the pixel is on

						/*				// ACTUAL LOCATION CALCULATION
						double theta_ph = atan(((2.0*y-200.0)/200.0)*TAN_FOVH_DIV_2);		// angle of the pixel
						double cos_theta_ph = cos(theta_ph);											// used in the computations
						double cos_theta_minus_ph = cos(theta - theta_ph);				// used in the computations
						
						yPos = ((double)((3.9)*((cos_theta_ph))))/(cos_theta_minus_ph) + 
											(2.1)*tan(theta - theta_ph);		// y distance from the bot
						xPos = ((yPos*(2.0*x - 320.0))/320.0)*TAN_FOVW_DIV_2;			// x distance from the bot
						
					if(frameloc[((int16_t)y)*RES_WIDTH + (int16_t)x] != 0) {
						*/
						frameloc[((int16_t)y-2)*RES_WIDTH + (int16_t)x - 1] = 0xFF;
										
						break;			// stop looking for the edge, break to the next x co-ordinate
					} // end if
					else {
						frameloc[(int16_t)(y+1)*RES_WIDTH + (int16_t)x+1] = 0xFF;
					}
						
		//				frameloc[(int16_t)yPos*RES_WIDTH + (int16_t)x] = 0xFF;
		//			}
					
				} // end yfor 
			} 	// end xfor
			
		// end front edge detection
			

		// tell chirp to use this buffer
		g_chirpUsb->useBuffer(frame, len+CAM_RES2_WIDTH*CAM_RES2_HEIGHT); 

}

int videoLoop()
{
	if (g_execArg==0)
		cam_getFrameChirp(CAM_GRAB_M1R2, 0, 0, CAM_RES2_WIDTH, CAM_RES2_HEIGHT, g_chirpUsb);
	else 
		sendCustom();
	return 0;
}

