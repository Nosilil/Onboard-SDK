/*
 * typedef.h
 *
 *  Created on: 10 set 2019
 *      Author: tommy
 */

#ifndef TYPEDEF_H_
#define TYPEDEF_H_

typedef struct {
	unsigned int l;				//long range count
	unsigned int aaddr;			//anchor address
	unsigned int taddr;			//tag address
	unsigned int txa;			//tx antenna delat
	unsigned int rxa;			//rx antenna delay
	unsigned int rng;			//range
	unsigned int rng_avg;		//pre-calculated average range
	unsigned int data_updt;		//is a new reading?
}dw1000_usb_data_t;



#endif /* TYPEDEF_H_ */
