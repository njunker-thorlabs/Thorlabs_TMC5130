/*
 * TMC5130_lib.c
 *
 *  Created on: Apr 29, 2024
 *      Author: NJunker
 */

#include "TMC5130_lib.h"

SPISettings TMC_Settings(4000000, MSBFIRST, SPI_MODE3);

void Thorlabs_TMC5130::begin(int8_t CS_pin)
{
	_CS_pin = CS_pin;
	
	pinMode(_CS_pin, OUTPUT);
	digitalWrite(_CS_pin, HIGH);

	A1 = 0x000088B8;    // (35,000)
	V1 = 0x00004E20;    // (20,000)
	AMAX = 0x00002710;  // (10,000)
	VMAX = 0x00030D40;  // (200,000)
	DMAX = 0x00003A98;  // (15,000)
	D1 = 0x0000C350;    // (50,000)
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
	SPI.beginTransaction(TMC_Settings);
	
	//Pull CS low, wait 1ms to ensure we hit minimum delay
	digitalWrite(_CS_pin, LOW);
	delay(1);
	
	//Transfer all 5 bytes, wait 1ms to hit minimum delay
	SPI.transfer(cmd, cmd_size);
	delay(1);
	
	//Pull CS high, end transaction
	digitalWrite(_CS_pin, HIGH);
	SPI.endTransaction();
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
	SPI.beginTransaction(TMC_Settings);
	
	//Pull CS low, do a dummy RX since we get requested data on NEXT frame
	digitalWrite(_CS_pin, LOW);
	delay(1);
	
	//Transfer data, wait 1ms
	SPI.transfer(_dummy_cmd, buf_size);
	delay(1);
	
	//Pull CS high
	digitalWrite(_CS_pin, HIGH);
	delay(1);
	
	//Repeat pulling CS low, save RX data we requested on previous frame
	digitalWrite(_CS_pin, LOW);
	delay(1);
	
	//Get data, do full transfer
	SPI.transfer(cmd, buf_size);
	delay(1);
	
	digitalWrite(_CS_pin, HIGH);

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
