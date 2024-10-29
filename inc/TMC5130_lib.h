/**************************************************************************//**
This SPI driver is used to interface the Trinamic TMC5130 Stepper Driver chip.

   \date          29-April-2024
   \copyright     2024 Thorlabs Spectral Works
   \author       Nicholas Junker (njunker@thorlabs.com)

******************************************************************************/


#ifndef INC_TMC5130_LIB_H_
#define INC_TMC5130_LIB_H_

#include <cstdint> //for uint8_t, etc
#include <cstddef> //for size_t
#include <cmath> //for sqrt

//Register definitions
#define MCL_GCONF       0x00	// (Address: 0)
#define MCL_SLAVECONF 	0x03	// (Address: 1)
#define MCL_X_COMPARE 	0x05	// (Address: 2)
#define MCL_IHOLD_IRUN  0x10	// (Address: 3)
#define MCL_TPOWERDOWN  0x11	// (Address: 4)
#define MCL_TPWMTHRS 	0x13	// (Address: 5)
#define MCL_TCOOLTHRS 	0x14	// (Address: 6)
#define MCL_THIGH       0x15	// (Address: 7)
#define MCL_RAMPMODE 	0x20	// (Address: 8)
#define MCL_XACTUAL 	0x21	// (Address: 9)
#define MCL_VACTUAL     0x22    // VACTUAL register
#define MCL_VSTART      0x23	// (Address: 10)
#define MCL_A1          0x24	// (Address: 11)
#define MCL_V1          0x25	// (Address: 12)
#define MCL_AMAX        0x26	// (Address: 13)
#define MCL_VMAX        0x27	// (Address: 14)
#define MCL_DMAX        0x28	// (Address: 15)
#define MCL_D1          0x2A	// (Address: 16)
#define MCL_VSTOP       0x2B	// (Address: 17)
#define MCL_TZEROWAIT 	0x2C	// (Address: 18)
#define MCL_XTARGET 	0x2D	// (Address: 19)
#define MCL_VDCMIN      0x33	// (Address: 20)
#define MCL_SW_MODE 	0x34	// (Address: 21)
#define MCL_XLATCH      0x36    // XLATCH register
#define MCL_ENCMODE 	0x38	// (Address: 22)
#define MCL_X_ENC       0x39	// (Address: 23)
#define MCL_ENC_CONST 	0x3A	// (Address: 24)
#define MCL_ENC_STATUS  0x3B    // Encoder N event detected register
#define MCL_ENC_LATCH   0x3C    // Encoder Latch position register
#define MCL_MS_LUT_0 	0x60	// (Address: 25)
#define MCL_MS_LUT_1 	0x61	// (Address: 26)
#define MCL_MS_LUT_2 	0x62	// (Address: 27)
#define MCL_MS_LUT_3 	0x63	// (Address: 28)
#define MCL_MS_LUT_4 	0x64	// (Address: 29)
#define MCL_MS_LUT_5 	0x65	// (Address: 30)
#define MCL_MS_LUT_6 	0x66	// (Address: 31)
#define MCL_MS_LUT_7 	0x67	// (Address: 32)
#define MCL_MS_LUTSEL 	0x68	// (Address: 33)
#define MCL_MS_LUTSTART 0x69	// (Address: 34)
#define MCL_CHOPCONF 	0x6C	// (Address: 35)
#define MCL_COOLCONF 	0x6D	// (Address: 36)
#define MCL_DCCTRL      0x6E	// (Address: 37)
#define MCL_PWMCONF 	0x70	// (Address: 38)
#define MCL_ENCM_CTRL   0x72	// (Address: 39)


class Thorlabs_TMC5130 {
public:
	//TODO Add more helper functions for setting up driver (hold & run current,
	//stealthChop/coolStep/fullStep thresholds, etc)

	typedef enum {
		positionMode = 0x00000000,
		velocityModePos = 0x00000001,
		velocityModeNeg = 0x00000002,
		holdMode = 0x00000003
	} rampMode;

	//Initialize object with SPI bus & CS pin, set default ramp values.
	void begin(int8_t CS_pin);

	//Write to a specific register.
	void write_register(uint8_t addr, uint32_t data);

	//Read a specific register. Returns the SPI_STATUS bit, with requested register data
	//located at the provided pointer
	uint8_t read_register(uint8_t addr, int32_t* out);

	//Set ramp generator between position, velocity, and hold mode
	void setRampMode(rampMode mode);

	//jog a specified number of microsteps from current position
	void jog(int32_t uSteps);

	//move to a specific position, regardless of current position
	void moveTo(int32_t pos);

	//Set VMAX. In position mode, this controls the max velocity during movement.
	//In velocity mode, this is the target speed it will run at.
	void setVelocity(int32_t velocity);

	//Toggle to enable or disable stealthChop. Use ONLY at standstill. Recommend enabling.
	void enableStealthChop(bool enabled);

	//Toggle to swap motor direction. Intended to help correct direction after installing motor. Not intended
	//as a quick "swap direction" during movement.
	void reverseDirection(bool enabled);

	//Manually set position register. Intended to help reset position counter on MCU restart or when homing.
	void setPosition(int32_t pos);

	//Get current stepper position from ramp genreator.
	int32_t getPosition();

	//Configure motor current limits. Set in Amps, with a max value of 1.35A. iHoldDelay scales between 1-15.
	//Keep iHoldDelay at default value if not needed.
	void setCurrentLimits(float iHoldCurrent, float iRunCurrent, int8_t iHoldDelay = 7);

	//Call to update A1, V1, AMAX, VMAX, DMAX, D1, and VSTOP register values if changed.
	//All values are in uSteps/second
	void updateMotionProfile();

	//Get current encoder position
	int32_t getEncoderPosition();

	//Manually set current stepper position. Intended to help reset counter to zero on MCU restart or when homing.
	void setEncoderPosition(int32_t pos);

	//Check if motor is moving or not.
	bool isStopped();

	uint32_t A1;
	uint32_t V1;
	uint32_t AMAX;
	uint32_t VMAX;
	uint32_t DMAX;
	uint32_t D1;
	uint32_t VSTOP;

protected:

	int8_t _cs;

	//Our own SPI transfer to facilitate different platforms
	virtual void Thorlabs_SPI_transfer(void *buf, size_t count);

	//User-implemented SPI begin function, if needed
	virtual void Thorlabs_SPI_begin();

	//User-implemented SPI end function, if needed
	virtual void Thorlabs_SPI_end();

	//User-implemented SPI setup function, if needed
	virtual void Thorlabs_SPI_setup();
	

private:

	//Quick little function to set starter values to get a stepper up and running.
	void basicMotorConfig();

};


#endif /* INC_TMC5130_LIB_H_ */
