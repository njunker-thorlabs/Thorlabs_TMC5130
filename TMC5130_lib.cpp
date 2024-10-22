/*
 * TMC5130_lib.c
 *
 *  Created on: Apr 29, 2024
 *      Author: NJunker
 */

#include "TMC5130_lib.h"

void Thorlabs_TMC5130::begin(int8_t CS_pin)
{
	_cs = CS_pin;

	A1 = 0x000088B8;    // (35,000)
	V1 = 0x00004E20;    // (20,000)
	AMAX = 0x00002710;  // (10,000)
	VMAX = 0x00030D40;  // (200,000)
	DMAX = 0x00003A98;  // (15,000)
	D1 = 0x0000C350;    // (50,000)

	Thorlabs_SPI_setup();
}

void Thorlabs_TMC5130::write_register(uint8_t addr, uint32_t data)
{
	const int cmd_size = 5;
	uint8_t cmd[cmd_size];
	//build command word
	cmd[0] = addr^0x80; //  bitwise XOR to set the write bit
	cmd[1] = (data >> 24) & 0xFF; //  rest of command
	cmd[2] = (data >> 16) & 0xFF;
	cmd[3] = (data >> 8) & 0xFF;
	cmd[4] = data & 0xFF;
	
	//Begin Transaction
	Thorlabs_SPI_begin();

	Thorlabs_SPI_transfer(cmd, cmd_size);

	Thorlabs_SPI_end();
}

uint8_t Thorlabs_TMC5130::read_register(uint8_t addr, uint32_t data, int32_t* out)
{
	const int buf_size = 5;
	uint8_t cmd[buf_size];
	uint8_t _dummy_cmd[buf_size];
	//build command word
	cmd[0] = addr^0x00; // bitwise XOR to set the read byte
	cmd[1] = (data >> 24) & 0xFF;
	cmd[2] = (data >> 16) & 0xFF;
	cmd[3] = (data >> 8) & 0xFF;
	cmd[4] = data & 0xFF;
	
	for (int i = 0; i < buf_size; i++) {
		_dummy_cmd[i] = cmd[i];
	}
	
	//Begin Transaction
	Thorlabs_SPI_begin();

	Thorlabs_SPI_transfer(_dummy_cmd, buf_size);

	Thorlabs_SPI_transfer(cmd, buf_size);
	
	Thorlabs_SPI_end();

	uint8_t _status = cmd[0];
	int32_t _out = ((int32_t) cmd[1]) << 24; // put the MSB in place
	_out |= ((int32_t) cmd[2]) << 16; // add next byte
	_out |= ((int32_t) cmd[3]) << 8; // add next byte
	_out |= ((int32_t) cmd[4]); // add LSB

	*out = _out;

	return _status;
}

void Thorlabs_TMC5130::jog(int32_t uSteps)
{
	int32_t buf;
	int32_t target;

	read_register(MCL_XACTUAL, 0x00000000, &buf);
	target = buf + uSteps;
	write_register(MCL_XTARGET, target);
}

void Thorlabs_TMC5130::moveTo(int32_t pos)
{
	write_register(MCL_XTARGET, pos);
}

bool Thorlabs_TMC5130::isStopped()
{
	int32_t buf;
	read_register(MCL_VACTUAL, 0x00000000, &buf);
	if ((int)buf == 0) {
		return true;
	}
	else {
		return false;
	}
}

void Thorlabs_TMC5130::setRampMode(rampMode mode)
{
	write_register(MCL_RAMPMODE, mode);
}

void Thorlabs_TMC5130::setVelocity(int32_t velocity)
{	VMAX = velocity;
	write_register(MCL_VMAX, VMAX);
}

//-----------------------------------------------------------------------
//------------------- To be implemented by user -------------------------
//-----------------------(Platform Specific)-----------------------------
//-----------------------------------------------------------------------

void Thorlabs_TMC5130::Thorlabs_SPI_transfer(void *buf, size_t count) {
	//Implement this in a parent class or modify for your platform
	
	//Take in an array of single bytes (buf) of size (count)
	//Replace the transmitted bytes with the received data
}

void Thorlabs_TMC5130::Thorlabs_SPI_begin() {
	//Implement this in a parent class or modify for your platform

	//Used if your platform has an SPI transaction begin function (i.e. Arduino)
}

void Thorlabs_TMC5130::Thorlabs_SPI_end() {
	//Implement this in a parent class or modify for your platform

	//Used if your platform has an SPI transaction end function (i.e. Arduino)
}

void Thorlabs_TMC5130::Thorlabs_SPI_setup() {
	//Implement this in a parent class or modify for your platform

	//Platform specific startup code, i.e. pin assignments / SPI initialization
}
