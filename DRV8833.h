
#ifndef DRV8833_H
#define DRV8833_H

#include "Arduino.h"
#include <dummy.h>

class DRV8833
{
public:
	// Constructor for the class:
	DRV8833();

	// Motor control functions:
	void motorAReverse();
	void motorAReverse(int speed);
	void motorAForward();
	void motorAForward(int speed);
	void motorAStop();

	void motorBReverse();
	void motorBReverse(int speed);
	void motorBForward();
	void motorBForward(int speed);
	void motorBStop();

	// Functions to attach motors:
	void attachMotorA(int a1 /* Input pin A1 */, int a2 /* Input pin A2 */);
	void attachMotorB(int b1 /* Input pin B1 */, int b2 /* Input pin B2 */);

private:
	// Fields for the class:
	int a1, a2, b1, b2;
	boolean motorAAttached = false;
	boolean motorBAttached = false;
};

#endif // DRV8833_H
