
#ifndef EDGEDETECT_HIGHRES_H
#define EDGEDETECT_HIGHRES_H

	void edgeDetect_highres_run();
	uint16_t intensityCalc_BlueRedPixel(uint8_t* frame, uint16_t x, uint16_t y);
	uint16_t intensityCalc_GreenPixel(uint8_t* frame, uint16_t x, uint16_t y);

#endif
