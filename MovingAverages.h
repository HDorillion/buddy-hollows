/*
 * MovingAverages.h
 *
 *  Created on: Nov 26, 2016
 *      Author: seth
 */

#ifndef SRC_MOVINGAVERAGES_H_
#define SRC_MOVINGAVERAGES_H_

#include <algorithm>
#include <iostream>
#include <utility>
#include <array>
#include "ControlSabertooth.h"

#define BUFFER_SIZE 24
#define BUFFER_BEGIN 16
#define CENTER_X 320
#define Z_CUTOFF_LOW  16384		//2^14
#define Z_CUTOFF_HIGH 32768		//2^15

static const char LogTable256[256] =
{
#define LT(n) n, n, n, n, n, n, n, n, n, n, n, n, n, n, n, n
		-1, 0, 1, 1, 2, 2, 2, 2, 3, 3, 3, 3, 3, 3, 3, 3,
		LT(4), LT(5), LT(5), LT(6), LT(6), LT(6), LT(6),
	    LT(7), LT(7), LT(7), LT(7), LT(7), LT(7), LT(7), LT(7)
};

class MovingAverages
{
public:
	//MovingAverages();
	//virtual ~MovingAverages();
	static void addXValue(unsigned int value, unsigned int weight);
	static unsigned int findLog2(unsigned int value);

private:
	static void drive();

	static std::pair<unsigned int, unsigned int> buffer[BUFFER_SIZE];
    static unsigned int averageX;
    static unsigned int averageZ;		//ToDo
	static int index;
	static ControlSabertooth controller;
};

#endif /* SRC_MOVINGAVERAGES_H_ */
