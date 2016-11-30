/*
 * ControlSabertooth.cpp
 *
 *  Created on: Sep 6, 2016
 *      Author: seth
 */

#include "ControlSabertooth.h"

ControlSabertooth::ControlSabertooth(std::string path): settings{}, file{}, debugLog{}
{
	//Arduino must be connected before we boot up!
	system(("stty -F " + path + " cs8 9600 ignbrk -brkint -icrnl -imaxbel "
			"-opost -onlcr -isig -icanon -iexten -echo -echoe -echok -echoctl -echoke noflsh "
			"-ixon -crtscts").c_str());

	#if LOG_DEBUG_COMMAND
		//overwrite Sabertooth.3.log (only keeping at most 4 logs)
		system("mv -f /home/ubuntu/DebugLogs/Sabertooth.2.log /home/ubuntu/DebugLogs/Sabertooth.3.log");
		system("mv -f /home/ubuntu/DebugLogs/Sabertooth.1.log /home/ubuntu/DebugLogs/Sabertooth.2.log");
		system("mv -f /home/ubuntu/DebugLogs/Sabertooth.0.log /home/ubuntu/DebugLogs/Sabertooth.1.log");
		debugLog.open("/home/ubuntu/DebugLogs/Sabertooth.0.log", Ofstream::out | Ofstream::app);
	#endif

	initialize(path);
}

ControlSabertooth::~ControlSabertooth()
{
	if(file > -1)
		system("echo 40 > /sys/class/gpio/unexport");

	debugLog.close();
}

int ControlSabertooth::initialize(std::string path)
{
	file = open(path.c_str(), O_RDWR | O_NOCTTY);

	#if PRINT_DEBUG_INIT
		std::cout << "Initialize: file opened as " << file << std::endl;
	#endif

	#if LOG_DEBUG_INIT
		debugLog << "Initialize: file opened as " << file << std::endl;
	#endif

	// get current serial port settings
	tcgetattr(file, &settings);

	// set 9600 baud both ways
	cfsetispeed(&settings, B9600);
	cfsetospeed(&settings, B9600);

	// 8 bits, no parity, no stop bits
	(settings.c_cflag) &= ~PARENB;
	(settings.c_cflag) &= ~CSTOPB;
	(settings.c_cflag) &= ~CSIZE;
	(settings.c_cflag) |= CS8;

	//Extra Code Modifications
	settings.c_cc[VTIME] = 1;	//set time timeout to 0.1 milliseconds (between bytes)
	settings.c_cc[VMIN] = 0;	//no minimum amount of bytes need to be received to return
	settings.c_lflag |= ~ICANON;	//change to non canonical mode

	//set settings
	tcsetattr(file, TCSANOW, &settings);

	//initialize GPIO pin
	if(file > -1)
	{
		system("echo 40 > /sys/class/gpio/export");
		system("echo in > /sys/class/gpio/gpio40/direction");
	}

	return file;
}

ControlSabertooth &ControlSabertooth::command(const uint8_t &command, const uint8_t &value)
{
	const uint8_t buff[4] = {(uint8_t)128, command, value, (uint8_t)((128 + command + value) & 0x7F)};
	int bytesWritten = 0;

	bytesWritten += write(file, &buff[0], 1);
	bytesWritten += write(file, &buff[1], 1);
	bytesWritten += write(file, &buff[2], 1);
	bytesWritten += write(file, &buff[3], 1);

	#if PRINT_DEBUG_COMMAND
		if(bytesWritten != 4)
			std::cout << "Warning: ControlSabertooth::command(uint8_t,uint8_t) did not send 4 bytes)" << std::endl;

		std::cout << "Address Byte: " << (int)buff[0] << std::endl;
		std::cout << "Command Byte: " << (int)buff[1] << std::endl;
		std::cout << "Value Byte: " << (int)buff[2] << std::endl;
		std::cout << "Checksum Byte: " << (int)buff[3] << std::endl;
	#endif

	#if LOG_DEBUG_COMMAND
		if(bytesWritten != 4)
			debugLog << "Warning: ControlSabertooth::command(uint8_t,uint8_t) did not send 4 bytes)" << std::endl;

		debugLog << "Address Byte: " << (int)buff[0] << std::endl;
		debugLog << "Command Byte: " << (int)buff[1] << std::endl;
		debugLog << "Value Byte: " << (int)buff[2] << std::endl;
		debugLog << "Checksum Byte: " << (int)buff[3] << std::endl;
	#endif

	return *this;
}

ControlSabertooth &ControlSabertooth::throttleCommand(const uint8_t &command, const int &power)
{
	return this->command( command, (uint8_t) abs(constrain(power,-126,126)));
}

const int &ControlSabertooth::constrain(const int &value, const int &min, const int &max)
{
	return value < min? min: value > max? max: value;
}

ControlSabertooth &ControlSabertooth::leftMotor(const int &power)
{
	#if PRINT_DEBUG_COMMAND
		std::cout << std::endl << "ControlSabertooth::leftMotor(const int&) sending packets..." << std::endl;
	#endif

	#if LOG_DEBUG_COMMAND
		debugLog << std::endl << "ControlSabertooth::leftMotor(const int&) sending packets..." << std::endl;
	#endif

	return throttleCommand(power < 0? 0: 1, power);
}

ControlSabertooth &ControlSabertooth::rightMotor(const int &power)
{
	#if PRINT_DEBUG_COMMAND
		std::cout << std::endl << "ControlSabertooth::rightMotor(const int&) sending packets..." << std::endl;
	#endif

	#if LOG_DEBUG_COMMAND
		debugLog << std::endl << "ControlSabertooth::rightMotor(const int&) sending packets..." << std::endl;
	#endif

	return throttleCommand(power < 0? 5: 4, power);
}

ControlSabertooth &ControlSabertooth::drive(const int &power)
{
	return leftMotor(power).rightMotor(power);
}

ControlSabertooth &ControlSabertooth::turn(const int &power)
{
	#if PRINT_DEBUG_COMMAND
		std::cout << std::endl << "ControlSabertooth::turn(const int&) sending packets..." << std::endl;
	#endif

	#if LOG_DEBUG_COMMAND
		debugLog << std::endl << "ControlSabertooth::turn(const int&) sending packets..." << std::endl;
	#endif

	return throttleCommand(power < 0? 0: 1, power).throttleCommand(power < 0? 4: 5, power);
}

ControlSabertooth &ControlSabertooth::stop()
{
	return leftMotor(0).rightMotor(0);
}

ControlSabertooth &ControlSabertooth::setTimeout(const int &milliseconds)
{
	#if PRINT_DEBUG_COMMAND
		std::cout << std::endl << "ControlSabertooth::setTimeout(const int&) sending packets..." << std::endl;
	#endif

	#if LOG_DEBUG_COMMAND
		debugLog << std::endl << "ControlSabertooth::setTimeout(const int&) sending packets..." << std::endl;
	#endif

	return command(14, (uint8_t)((constrain(milliseconds, 0, 12700) + 99) / 100));
}

ControlSabertooth &ControlSabertooth::setRamping(const uint8_t &value)
{
	#if PRINT_DEBUG_COMMAND
		std::cout << std::endl << "ControlSabertooth::setRamping(const uint8_t&) sending packets..." << std::endl;
	#endif

	#if LOG_DEBUG_COMMAND
		debugLog << std::endl << "ControlSabertooth::setRamping(const uint8_t&) sending packets..." << std::endl;
	#endif

	return command(16, (uint8_t)constrain(value, 0, 80));
}

ControlSabertooth &ControlSabertooth::setDeadband(const uint8_t &value)
{
	#if PRINT_DEBUG_COMMAND
		std::cout << std::endl << "ControlSabertooth::setDeadband(const uint8_t&) sending packets..." << std::endl;
	#endif

	#if LOG_DEBUG_COMMAND
		debugLog << std::endl << "ControlSabertooth::setDeadBand(const uint8_t&): sending packets..." << std::endl;
	#endif

	return command(17, (uint8_t) (value < 127? value: 127));
}

int ControlSabertooth::readGPIO40()
{
	char c_str[3];
	int fd = open("sys/class/gpio/gpio40/value", O_RDONLY);

	if(fd < 0)
	{
		std::cout << "Error; Class: ControlSabertooth, Method: readGPIO40, could not open file to read GPIO pin" << std::endl;
		return -1;
	}
	else
	{
		if(read(fd, c_str, 3) < 0)
		{
			std::cout << "Error; Class: ControlSabertooth, Method: readGPIO40, could not read file" << std::endl;
			return -1;
		}
		else
		{
			return atoi(c_str);
		}
	}
}

int ControlSabertooth::getFile()
{
	return file;
}
