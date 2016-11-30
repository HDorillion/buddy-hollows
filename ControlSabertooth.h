/*
 * ControlSabertooth.h
 *
 *  Created on: Sep 6, 2016
 *      Author: seth
 */

#ifndef CONTROLSABERTOOTH_H_
#define CONTROLSABERTOOTH_H_

#include <cstdio>
#include <cstdint>
#include <cstdlib>
#include <string>
#include <fstream>
#include <iostream>
#include "constants.h"

//for system calls
#include <fcntl.h>
#include <termios.h>
#include <unistd.h>

using Termios = struct termios;
using Ofstream = std::ofstream;

class ControlSabertooth
{
public:
	//Constructors
	//ControlSabertooth();
    ControlSabertooth(std::string path = "/dev/ttyACM0"); //default = "/dev/ttyACM0"

	//Rule of 5; all default since we're not handling memory
	ControlSabertooth(const ControlSabertooth &)=default;
	ControlSabertooth(ControlSabertooth &&)=default;
	ControlSabertooth &operator=(const ControlSabertooth &)=default;
	ControlSabertooth &operator=(ControlSabertooth &&)=default;
	virtual ~ControlSabertooth();

	//Accessors
    std::string getPath();
	Termios getTermios();
	int getFile();

	//Mutators
    ControlSabertooth &setPath(std::string);
	ControlSabertooth &setTermiosSettings(Termios);
	ControlSabertooth &setFile(int);

	//control
	ControlSabertooth &leftMotor(const int &power);		//if you call leftMotor, you must call rightMotor!
	ControlSabertooth &rightMotor(const int &power);
	ControlSabertooth &drive(const int &power);
	ControlSabertooth &turn(const int &power);
	ControlSabertooth &stop();

	//settings
	ControlSabertooth &setTimeout(const int &milliseconds);
	ControlSabertooth &setRamping(const uint8_t &value);
	ControlSabertooth &setDeadband(const uint8_t &value);

	//helper static function
	static int readGPIO40();
private:
	Termios settings;
	int file;
	Ofstream debugLog;

    int initialize(std::string = "/dev/ttyACM0");	//returns true if initialization is successful
	inline const int &constrain(const int &value, const int &min, const int &max);

	//make sabertooth do stuff
	ControlSabertooth &throttleCommand(const uint8_t &command, const int &power);
	ControlSabertooth &command(const uint8_t &command, const uint8_t &value);
};

#endif /* CONTROLSABERTOOTH_H_ */
