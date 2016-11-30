/*
 * MovingAverages.cpp
 *
 *  Created on: Nov 26, 2016
 *      Author: seth
 */

#include "MovingAverages.h"

std::pair<unsigned int, unsigned int> MovingAverages::buffer[];
unsigned int MovingAverages::averageZ = 0;
unsigned int MovingAverages::averageX = 640 >> 1;
int MovingAverages::index = 0;
ControlSabertooth MovingAverages::controller{};

/*
MovingAverages::MovingAverages()
{
	// TODO Auto-generated constructor stub

}

MovingAverages::~MovingAverages()
{
	// TODO Auto-generated destructor stub
}
*/

void MovingAverages::drive()
{
	int drive = averageZ;
	int turn = (averageX >> 4) - 20;	//max turn value is now between -20 and 20

	if(drive < Z_CUTOFF_LOW || drive > Z_CUTOFF_HIGH)		//constrain Z (must be positive or 0)
		drive = 0;
	else
	{
		drive = (Z_CUTOFF_HIGH - (drive - Z_CUTOFF_LOW)) - Z_CUTOFF_LOW; //now between 0 and 2^14
		drive >>= 9;	//max is now 2^5 == 32

		if(drive < 0 || drive > 32)
			drive = 0;
	}

	std::cout << "Drive value = " << drive << std::endl;
	std::cout << "Turn Value =  " << turn << std::endl;

	if(controller.getFile() > -1)	//check to see if file to ACM0 opened correctly
	{
		if((ControlSabertooth::readGPIO40() != 1) && (turn < 2) && (turn > -2))  //check sensors here!
		{
			if(turn < 2 && turn > -2)
				turn = 0;	//if sensors and x does not deviate too much, just go straight, don't turn
			else
				turn = drive = 0;
		}

		controller.leftMotor(drive - turn).rightMotor(drive + turn);
	}
}

void MovingAverages::addXValue(unsigned int value, unsigned int weight)
{
	if((int(weight) >> 8) > 0)
		weight >>= 8;
	else
		weight = 0;

	//add in weight
	buffer[index].first = weight;
	buffer[index].second = value;

	if(index < (BUFFER_SIZE - 1))
		index++;
	else
	{
		unsigned int totalWeight = 0, newAverageX = 0, newAverageZ = 0;

		std::sort(buffer, buffer + BUFFER_SIZE);
		index = 0;

		for(int i = BUFFER_BEGIN; i < BUFFER_SIZE; i++)
		{
			newAverageX += buffer[i].first * buffer[i].second;
			newAverageZ += buffer[i].first * buffer[i].first;
			totalWeight += buffer[i].first;
		}

		if(totalWeight)
		{
			newAverageX /= totalWeight;
			newAverageZ /= totalWeight;
		}
		else
			newAverageX = newAverageZ = 0;

		//calculate averageX using old value as well (for smoother transition)
		if((int(newAverageX) - int(averageX) < 40) && (int(newAverageX) - int(averageX)) > -40)
			newAverageX = ((3 * newAverageX) + averageX) >> 2;

		//only update averageX value if it differs by 5 from the old value
		if((int(newAverageX) - int(averageX) > 5) || (int(newAverageX) - int(averageX)  < -5))
			averageX = newAverageX;

		if((int(newAverageZ) - int(newAverageZ)) > 100 || (int(newAverageZ) - int(newAverageZ)) < -100)
			averageZ = newAverageZ;

		if(averageX)		//safety?
			drive();
	}
}

unsigned int MovingAverages::findLog2(unsigned int value)
{
	register unsigned int temp; // temporary

	if ((temp = value >> 24))
	  return 24 + LogTable256[temp];
	else if ((temp = value >> 16))
	  return 16 + LogTable256[temp];
	else if ((temp = value >> 8))
	  return 8 + LogTable256[temp];
	else
	  return LogTable256[value];
}
