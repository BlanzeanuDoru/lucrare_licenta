/*
 * samr21.h
 *
 * Created: 2/25/2018 6:11:20 PM
 *  Author: doru
 */ 


#ifndef SAMR21_H_
#define SAMR21_H_

#if defined(__SAMR21G18A__) || defined(__ATSAMR21G18A__)
#include "samr21g18a.h"
#else
	#error Library does not support the specified device
#endif

#endif /* SAMR21_H_ */