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

	//Default parameters that work with most stepper setups
	A1 = 0x000088B8;    // (35,000)
	V1 = 0x00004E20;    // (20,000)
	AMAX = 0x00002710;  // (10,000)
	VMAX = 0x00030D40;  // (200,000)
	DMAX = 0x00003A98;  // (15,000)
	D1 = 0x0000C350;    // (50,000)
	VSTOP = 0x0000000A; // (10)

	Thorlabs_SPI_setup();

	updateMotionProfile();
	basicMotorConfig();
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

uint8_t Thorlabs_TMC5130::read_register(uint8_t addr, int32_t* out)
{
	const int buf_size = 5;
	uint8_t cmd[buf_size];
	uint8_t _dummy_cmd[buf_size];

	//build command word. Rest of cmd word [1-4] is all 0
	cmd[0] = addr^0x00; // bitwise XOR to set the read byte
	
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

	read_register(MCL_XACTUAL, &buf);
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
	read_register(MCL_VACTUAL, &buf);
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

void Thorlabs_TMC5130::enableStealthChop(bool enabled)
{
	int32_t currentConfig;
	int32_t newConfig;
	int32_t configMask;

	int8_t en_pwm_mode_reg_offset = 2;

	read_register(MCL_GCONF, &currentConfig);

	//Make mask to select en_pwm_mode bit
	configMask = ~(1 << en_pwm_mode_reg_offset);

	//Reset en_pwm_mode bit from current config
	newConfig = currentConfig & configMask;

	//Set en_pwm_mode bit to our selection
	newConfig |= (enabled << en_pwm_mode_reg_offset);

	write_register(MCL_GCONF, newConfig);
}

void Thorlabs_TMC5130::reverseDirection(bool enabled)
{
	int32_t currentConfig;
	int32_t newConfig;
	int32_t configMask;

	int8_t shaft_reg_offset = 4;

	read_register(MCL_GCONF, &currentConfig);

	//Make mask to select bit
	configMask = ~(1 << shaft_reg_offset);

	//Reset bit from current config
	newConfig = currentConfig & configMask;

	//Set bit to our selection
	newConfig |= (enabled << shaft_reg_offset);

	write_register(MCL_GCONF, newConfig);
}

void Thorlabs_TMC5130::setPosition(int32_t pos)
{
	write_register(MCL_XACTUAL, pos);
}

int32_t Thorlabs_TMC5130::getPosition()
{
	int32_t pos;
	read_register(MCL_XACTUAL, &pos);
	return pos;
}

void Thorlabs_TMC5130::setCurrentLimits(float iHoldCurrent, float iRunCurrent, int8_t iHoldDelay)
{
	float VfsVoltage;
	bool VfsBit;
	float Rsense = 0.15;
	int8_t iHold, iRun;

	//If above 750mA, use Vsense scaling of 0.32V. Otherwise use scaling of 0.18V.
	VfsVoltage = (iHoldCurrent > 0.75 || iRunCurrent > 0.75) ? 0.32 : 0.18;

	//Same as above, but getting the actual register value to write
	VfsBit = ~(iHoldCurrent > 0.75 || iRunCurrent > 0.75);

	//Calculate 5 bit scalar values for iHold and iRun from motor current
	//Equation is rearranged from section 10 of TMC5130 datasheet
	iHold = abs(((32 * sqrt(2) * iHoldCurrent * (Rsense + 0.02)) / VfsVoltage) - 1);
	iRun = abs(((32 * sqrt(2) * iRunCurrent * (Rsense + 0.02)) / VfsVoltage) - 1);

	//Format and write to IHOLD_IRUN register
	int32_t IHOLD_IRUN_CONFIG;
	IHOLD_IRUN_CONFIG |= ((iHoldDelay & 0xF) << 16);
	IHOLD_IRUN_CONFIG |= ((iRun & 0x1F) << 8);
	IHOLD_IRUN_CONFIG |= (iHold & 0x1F);

	//Write newly formatted IHOLD_IRUN register
	write_register(MCL_IHOLD_IRUN, IHOLD_IRUN_CONFIG);

	//Format and write to CHOPCONF register based on our Vfs selection
	int32_t currentChopconf;
	int32_t newChopconf;
	int32_t configMask;
	int8_t vsense_reg_offset = 17;

	//Read current settings so we don't overwrite anything else
	read_register(MCL_CHOPCONF, &currentChopconf);

	//get bitmask for our specific register
	configMask = ~(1 << vsense_reg_offset);

	//Reset the bit that we want to modify
	newChopconf = currentChopconf & configMask;

	//Set the bit to the value we want
	newChopconf |= (VfsBit << vsense_reg_offset);

	//Write newly formatted settings to register
	write_register(MCL_CHOPCONF, newChopconf);
}

void Thorlabs_TMC5130::updateMotionProfile()
{
	write_register(MCL_A1, A1); // write value 0x000003E8 = A1 to address 11 = 0x24(A1)
	write_register(MCL_V1, V1); // write value 0x000088B8 = V1 to address 12 = 0x25(V1)
	write_register(MCL_AMAX, AMAX); // write value 0x00002710 = AMAX to address 13 = 0x26(AMAX)
	write_register(MCL_VMAX, VMAX); // write value 0x0000C350 = VMAX to address 14 = 0x27(VMAX)
	write_register(MCL_DMAX, DMAX); // write value 0x00002710 = DMAX to address 15 = 0x28(DMAX)
	write_register(MCL_D1, D1); // write value 0x000003E8 = D1 to address 16 = 0x2A(D1)
	write_register(MCL_VSTOP, VSTOP); // write value 0x0000000A = 10 = 10.0 to address 17 = 0x2B(VSTOP)
}

int32_t Thorlabs_TMC5130::getEncoderPosition() 
{
	int32_t pos;
	read_register(MCL_X_ENC, &pos);
	return pos;
}

void Thorlabs_TMC5130::setEncoderPosition(int32_t pos)
{
	write_register(MCL_X_ENC, pos);
}

void Thorlabs_TMC5130::basicMotorConfig()
{
	//Setting CHOPCONF in here since general user doesn't need to tweak TOFF/HSTRT values
	write_register(MCL_CHOPCONF, 0x000301D5);

	//Setting PWMCONF in here since user can get funky results if manually tweaking
	write_register(MCL_PWMCONF, 0x000501C8);
}

//TODO: add helper function to set encoder mode and scaling value


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
