// Operational Parameters
// Max output shaft deflection (180 or 360 degrees)
const int maxOutputDeflection = 360;

// Stepper Parameters
// Number of steps per stepper shaft rotation
const int stepsPerRevolution = 200;
const int stepperDefaultSpeed = 1600;		// Hz (steps/sec)
const int stepperDefaultAccel = 10000;		// steps/sec^2

const int deadBand			  = 25;			// Delta in steps before motion occurs (helps prevent trembling)

// Number of turns of the stepper required to make one revolution of the output hub
const int32_t multFactor = 10;
const int32_t gearRatio = (14*18*38*multFactor)/(10*6*6); // 26.6 * 10

const int32_t maxOutputDeflectionSteps = ((int32_t)maxOutputDeflection*stepsPerRevolution*gearRatio)/(360ul*multFactor);

// Task timing tolerance
const int taskTimeTol = 250;        // micro-seconds

// Pin Definitions
#define STEPPER_DIR_PIN		6
#define STEPPER_EN_PIN		5
#define STEPPER_STEP_PIN	stepPinStepper1A		// aka pin 9
#define PWM_IN_PIN			7						// NOTE: don't use any of the 'PCINTx' pins.
#define HOME_IN_PIN			8						// From magnet sensor

// Expected Input Pulse Width Range: ~1000 to ~2000
// Empirically, I'm seeing 896 - 1992 with the cheap servo tester.
// This will need to be calibrated to the FC output.
const int rcMinPulseWidth = 896;
const int rcMaxPulseWidth = 1992;
const int rcInputRange = rcMaxPulseWidth - rcMinPulseWidth;

