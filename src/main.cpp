// IDEAS:
// - get homing algorithm working; make it continuous during operation in case steps get lost (limited by inability to get current step when in operation)
// - implement simple state machine to govern operation of the entire device (ehh)
// - do pulse filtering and targetPosition calculation in ISR (only 50 Hz if filtered properly) (DONE)
// - implement a timer interrupt so we can have timed tasks instead of letting loop() run willy-nilly. (not needed)
// - refactor/clean up - move constants and add prototypes to the header file, etc. (DONE)

// WIRING 
// CPU Board Pin
// VCC -> 5V
// GND -> GND
// 5 -> A4988 pin 1 (EN)
// 6 -> A4988 pin 8 (DIR)
// 7 -> Servo Tester OUT S pin
// 8 -> Hall Effect Sensor board AO pin
// 9 -> A4988 pin 7 (STEP)
//
// A4988 Board Pin
// 5 (RST) -> A4988 pin 6 (SLP)
// 16 -> +12V (100 uF to GND)
// 15 -> GND
// 11-14 -> stepper coils
// 10 -> 5V
// 11 -> GND
//
// Servo Tester
// + -> 5V
// - -> GND
//
// Hall Effect Sensor Board
// + -> 5V
// G -> GND
//

#include <Arduino.h>
#include "FastAccelStepper.h"
#include "AVRStepperPins.h"
#include <FIR.h>
#include "main.h"

#define DEBUG_OUTPUT	1

// Set up time variables for RC Read
volatile uint32_t pWidthUs = 1500;
volatile uint32_t CurrentTime = 0;
volatile uint32_t StartTime = 0;
volatile uint16_t PulseWidth = 1500;

// Other globals
int32_t targetPosition = 0;

// Input filter instance
const uint16_t FILTER_DEPTH	= 13;
FIR<int32_t, FILTER_DEPTH> fir;

// Define the stepper engine
FastAccelStepperEngine engine = FastAccelStepperEngine();
FastAccelStepper *stepper = NULL;

// Running estimate of the home position
int16_t magEdges[2][2] = { 0 };			// Contains edge positions for each direction

// Get sensor state
bool magDetected(void)
{
	// The sensor is ACTIVE LOW.
	return (digitalRead(HOME_IN_PIN) == LOW);
}

// Sweep the shaft to both extremes until the magnetic sensor state changes, then zero the position.
void findHome(void)
{	
	int32_t homePosition = 0;
	int32_t stepNum = 0;
	int32_t initial = stepper->getCurrentPosition();;
	int32_t start = 0;
	int32_t stop = 0;
	bool magFound = magDetected();
	bool dir = false;

	// Give time for serial monitor to start
	delay(3000);

	Serial.print("Mag Detected: ");
	Serial.print(magFound);	
	Serial.print(" Init Pos: ");
	Serial.print(initial);		

	if (magFound)
	{
		// We woke up already on the magnet. Find the center.
		while (magDetected())
		{
			stepper->move(dir ? 1:-1);
			delay(2);			// Delay in ms to keep from over-revving the stepper.
		}

		// Wait until stepper stops.
		while(stepper->isRunning());

		stop = stepper->getCurrentPosition();

		// Return to initial position
		stepper->moveTo(initial);

		// Wait until stepper stops.
		while(stepper->isRunning());

		// Now go in reverse to the other side of the magnet
		dir = !dir;
		while (magDetected())
		{
			stepper->move(dir ? 1:-1);
			delay(2);			// Delay in ms to keep from over-revving the stepper.
		}

		// Wait until stepper stops.
		while(stepper->isRunning());

		start = stepper->getCurrentPosition();
	}
	else
	{
		// We woke up off the magnet. Hunt for it.
		// Sweep up to half a rotation
		while (!magDetected() && (stepNum <= maxOutputDeflectionSteps/2))
		{
			stepper->move(dir ? 1:-1);
			stepNum++;
			delay(2);			// Delay in ms to keep from over-revving the stepper.
		}

		// Wait until stepper stops.
		while(stepper->isRunning());

		if (!magDetected())
		{
			// Try going the other way for a full rotation.
			stepNum = 0;
			dir = !dir;
			
			while (!magDetected() && (stepNum <= maxOutputDeflectionSteps))
			{
				stepper->move(dir ? 1:-1);
				stepNum++;
				delay(2);			// Delay in ms to keep from over-revving the stepper.
			}
		}

		// Wait until stepper stops.
		while(stepper->isRunning());

		start = stepper->getCurrentPosition();

		// Keep going the same direction until we reach the other side of the magnet.
		while (magDetected())
		{
			stepper->move(dir ? 1:-1);
			delay(2);			// Delay in ms to keep from over-revving the stepper.
		}

		// Wait until stepper stops.
		while(stepper->isRunning());

		stop = stepper->getCurrentPosition();
	}

	// Average the start and stop to get the home position.
	homePosition = (start + stop)/2;

	// Center the shaft
	stepper->moveTo(homePosition);

	// Wait until stepper stops.
	while(stepper->isRunning());

	// Current position should be 'home', so set the home position in the stepper driver
	stepper->setCurrentPosition(0);

	Serial.print(" Edge1: ");
	Serial.print(start);
	Serial.print(" Edge2: ");
	Serial.print(stop);
	Serial.print(" Home: ");
	Serial.println(homePosition);
}

// Function to measure the length of the pulses from the remote control input
void PulseTimer(void)
{	
	// Get the current microsecond count
  	CurrentTime = micros();

	// We're only interested in the pulse width between low->high and high-low transitions
	if (digitalRead(PWM_IN_PIN) == HIGH)
	{
		// This is a low->high transition, record the start time
		StartTime = CurrentTime;
	}
	else
	{
		// This is a high->low transition
		// Calculate the time interval
		pWidthUs = CurrentTime - StartTime;

		// Only interested in short pulse lengths
		if (pWidthUs < rcMaxPulseWidth)
		{
			// Filter the noisy pulse width			
			PulseWidth = fir.processReading((int32_t)pWidthUs);		
		}
	}
}

void setup(void)
{
	// initialize the serial port:
	Serial.begin(19200);

	// Set up the HOME input pin	
	pinMode(HOME_IN_PIN, INPUT_PULLUP);

	// Set up the PWM input pin
	pinMode(PWM_IN_PIN, INPUT_PULLUP);

	// Set up the stepper motor
	engine.init();
	stepper = engine.stepperConnectToPin(STEPPER_STEP_PIN);

	if (stepper)
	{
		stepper->setDirectionPin(STEPPER_DIR_PIN);
		stepper->setEnablePin(STEPPER_EN_PIN);
		stepper->setAutoEnable(true);

		stepper->setSpeedInHz(stepperDefaultSpeed);
		stepper->setAcceleration(stepperDefaultAccel);
	}
	else
	{
		Serial.println("Unable to connect to stepper!");
		while(1);
	}

	// Find the home position
	findHome();

	// FIR Low-Pass, Rectangular Window, fs = 50 Hz, fL = 10 Hz, bL = 4 Hz
	//int32_t coef[FILTER_DEPTH] = { 5, 0, -7, -6, 9, 30, 39, 30, 9, -6, -7, 0, 5 };

	// FIR Low-Pass, Rectangular Window, fs = 50 Hz, fL = 7 Hz, bL = 4 Hz
	//int32_t coef[FILTER_DEPTH] = { -5, -7, -3, 6, 17, 27, 31, 27, 17, 6, -3, -7, -5 };

	// FIR Low-Pass, Rectangular Window, fs = 50 Hz, fL = 3 Hz, bL = 4 Hz
	int32_t coef[FILTER_DEPTH] = { 4, 5, 7, 9, 10, 10, 11, 10, 10, 9, 7, 5, 4 };

  	// Set the coefficients
  	fir.setFilterCoeffs(coef);

	// Set up interrupt
	attachInterrupt(digitalPinToInterrupt(PWM_IN_PIN), PulseTimer, CHANGE);	
}


/** Timed Tasks ***************************************************************************************************************/
void task_1ms(void)
{

}


// There appears to be some issues with the follow function. I think that 'getCurrentPosition' is only valid when the
// stepper isn't moving, which is a bummer.
void task_10ms(void)
{
	// Monitor for changes to the magnet detector pin to continually update home position (in case steps get lost)
	static bool lastState = HIGH;			// This assumes the shaft is in the home state initially.	

	// Stepper direction indices
	const int upDir = 0;
	const int downDir = 1;
	const int enter = 0;
	const int exit = 1;

	bool currentState = digitalRead(HOME_IN_PIN);
	
	// Check to see if there has been a state change
	if (currentState != lastState)
	{
		if(currentState)
		{
			// This is a low-high transition (entering magnet)
			if (stepper->getCurrentSpeedInUs() > 0)
			{
				// Stepper is counting up
				magEdges[upDir][enter] = stepper->getCurrentPosition();
			}
			else if (stepper->getCurrentSpeedInUs() < 0)
			{
				// Stepper is counting down
				magEdges[downDir][enter] = stepper->getCurrentPosition();
			}
		}
		else
		{
			// This is a high-low transition (exiting magnet)			/
			if (stepper->getCurrentSpeedInUs() > 0)
			{
				// Stepper is counting up
				magEdges[upDir][exit] = stepper->getCurrentPosition();
			}
			else if (stepper->getCurrentSpeedInUs() < 0)
			{
				// Stepper is counting down
				magEdges[downDir][exit] = stepper->getCurrentPosition();
			}			
		}

		// Update the last state
		lastState = currentState;
	}
}

void task_50ms(void)	// Aka 20 Hz
{
	// Map the latest pulse width to an absolute position between +/-maxOutputDeflection/2			
	int32_t newPosition = map(PulseWidth, rcMinPulseWidth, rcMaxPulseWidth, -maxOutputDeflectionSteps/2, maxOutputDeflectionSteps/2);			

	// Update stepper position only on changes
	if (abs(targetPosition - newPosition) > deadBand)
	{	
		targetPosition = newPosition;		
		
		// Move the stepper
		stepper->moveTo(targetPosition);
	}
}

void task_500ms(void)
{
#if DEBUG_OUTPUT
	uint32_t now = millis();

	int32_t currentPosition = stepper->getCurrentPosition();
	int32_t distanceToGo = 0;

	if (currentPosition > targetPosition)
	{
		distanceToGo = currentPosition - targetPosition;
	}
	else
	{
		distanceToGo = targetPosition - currentPosition;
	}

	Serial.print(now);
	Serial.print(": PW = ");
	Serial.print(PulseWidth);
	Serial.print(", Target Pos = ");
	Serial.print(targetPosition);
	Serial.print(", Current Pos = ");
	Serial.print(currentPosition);
	Serial.print(", Dist = ");
	Serial.print(distanceToGo);
	Serial.print(", MagEdges Ent = ");
	Serial.print(magEdges[0][0]);
	Serial.print(", ");
	Serial.print(magEdges[1][1]);
	Serial.print(", Exit = ");
	Serial.print(magEdges[0][1]);
	Serial.print(", ");
	Serial.print(magEdges[1][0]);
	Serial.println("");
#endif
}

void loop(void) 
{	
	static uint32_t task_1ms_run_time = false;
	static uint32_t task_10ms_run_time = false;
	static uint32_t task_50ms_run_time = false;
	static uint32_t task_500ms_run_time = false;
	uint32_t now = micros();

	// Time-based tasks:

	// 1 ms task
	if ((now - task_1ms_run_time) >= 1000)
	{
		task_1ms();
		task_1ms_run_time = now;
	}

	// 10 ms task
	if ((now - task_10ms_run_time) >= 10000)
	{		
		task_10ms();
		task_10ms_run_time = now;
	}

	// 50 ms task
	if ((now - task_50ms_run_time) >= 50000)
	{		
		task_50ms();
		task_50ms_run_time = now;
	}

	// 500 ms task
	if ((now - task_500ms_run_time) >= 500000)
	{		
		task_500ms();
		task_500ms_run_time = now;
	}

	// Do "idle" stuff here
}

