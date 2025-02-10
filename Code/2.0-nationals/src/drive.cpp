/**
 * Drivetrain stuff
 */
#include "main.h"

/**
 * Cheesy Drive Constants
 */
#define DRIVE_DEADBAND 0.1f
#define DRIVE_SLEW 0.08f
#define CD_TURN_NONLINEARITY                                                   \
  0.65 // This factor determines how fast the wheel
       // traverses the "non linear" sine curve
#define CD_NEG_INERTIA_SCALAR 4.0
#define CD_SENSITIVITY 0.5

int scaleValue = 100;

// Unused constants from 254's implementation
// static double kLowWheelNonLinearity = 0.5;
// static double kHighNegInertiaScalar = 4.0;
// static double kLowNegInertiaThreshold = 0.65;
// static double kLowNegInertiaTurnScalar = 3.5;
// static double kLowNegInertiaCloseScalar = 4.0;
// static double kLowNegInertiaFarScalar = 5.0;
// static double kHighSensitivity= 0.95;
// static double kLowSensitiity = 1.3;
// static double kQuickStopDeadband = 0.2;
// static double kQuickStopWeight = 0.1;
// static double kQuickStopScalar = 5.0;

/**
 * The "curvature" drive control written by FRC team 254.
 *
 * See the Java source at
 * https://github.dev/Team254/FRC-2015/blob/9dcc11886a49d29f16e597e317c995ca248efaed/src/com/team254/frc2015/CheesyDriveHelper.java#L24
 *
 * Essentially this does three things:
 *
 * Cheesy Drive applies some non-linearity to the joystick input so that
 * there is more control at the low speeds (larger changes in joystick inputs
 * result in smaller changes in real speed here) but when the joystick is
 * pushed to a high speed, you jump up to full speed faster.
 *
 * Additionally, when you a driving forward but also turning a bit, the turn
 * input affects the "curvature" of the movement rather than adding/subtracting
 * linearly from the wheel speeds. The turn output is a sum of the throttle and
 * turn inputs, meaning that the robot will turn faster when it's moving
 * forward at a higher speed. Again, the goal here is more control at low
 * speeds.
 *
 * Third, that turn input is affected by a negative inertia accumulator. Most
 * robots have a fair bit of turning inertia, which can make it easy to
 * accidentally overshoot a turn. The negative inertia accumulator acts almost
 * like a reverse integral controller - the longer the robot has been turning
 * (fast) for, the slower the robot will turn.
 */

// We apply a sinusoidal curve (twice) to the joystick input to give finer
// control at small inputs.
static double _turnRemapping(double iturn) {
	double denominator = sin(M_PI / 2 * CD_TURN_NONLINEARITY);
	double firstRemapIteration =
	    sin(M_PI / 2 * CD_TURN_NONLINEARITY * iturn) / denominator;
	return sin(M_PI / 2 * CD_TURN_NONLINEARITY * firstRemapIteration) / denominator;
}

// On each iteration of the drive controller (where we aren't point turning) we
// constrain the accumulators to the range [-1, 1].
double quickStopAccumlator = 0.0;
double negInertiaAccumlator = 0.0;
static void _updateAccumulators() {
	if (negInertiaAccumlator > 1) {
		negInertiaAccumlator -= 1;
	} else if (negInertiaAccumlator < -1) {
		negInertiaAccumlator += 1;
	} else {
		negInertiaAccumlator = 0;
	}

	if (quickStopAccumlator > 1) {
		quickStopAccumlator -= 1;
	} else if (quickStopAccumlator < -1) {
		quickStopAccumlator += 1;
	} else {
		quickStopAccumlator = 0.0;
	}
}

double prevTurn = 0.0;
double prevThrottle = 0.0;
std::pair<float, float> cheesyDrive(double ithrottle, double iturn) {
	ithrottle /= 127;
	iturn /= 127;

    bool turnInPlace = false;
	double linearCmd = ithrottle;
	if (fabs(ithrottle) < DRIVE_DEADBAND && fabs(iturn) > DRIVE_DEADBAND) {
		// The controller joysticks can output values near zero when they are
		// not actually pressed. In the case of small inputs like this, we
		// override the throttle value to 0.
		linearCmd = 0.0;
		turnInPlace = true;
	} //else if (ithrottle - prevThrottle > DRIVE_SLEW) {
	// 	linearCmd = prevThrottle + DRIVE_SLEW;
	// } else if (ithrottle - prevThrottle < -(DRIVE_SLEW * 2)) {
	// 	// We double the drive slew rate for the reverse direction to get
	// 	// faster stopping.
	// 	linearCmd = prevThrottle - (DRIVE_SLEW * 2);
	// }

	double remappedTurn = _turnRemapping(iturn);

	double left, right;
	if (turnInPlace) {
		// The remappedTurn value is squared when turning in place. This
		// provides even more fine control over small speed values.
		left = remappedTurn * std::abs(remappedTurn);
		right = -remappedTurn * std::abs(remappedTurn);

		// The FRC Cheesy Drive Implementation calculated the
		// quickStopAccumulator here:
		// if (Math.abs(linearPower) < 0.2) {
		// 	double alpha = 0.1;
		// 	quickStopAccumulator = (1 - alpha) * quickStopAccumulator
		// 			+ alpha * Util.limit(wheel, 1.0) * 5;
		// }
		// But I apparently left that out of my implementation? Seemed to work
		// without it though.
	} else {
		double negInertiaPower = (iturn - prevTurn) * CD_NEG_INERTIA_SCALAR;
		negInertiaAccumlator += negInertiaPower;

		double angularCmd =
		    fabs(linearCmd) *  // the more linear vel, the faster we turn
		        (remappedTurn + negInertiaAccumlator) *
		        CD_SENSITIVITY -  // we can scale down the turning amount by a
		                          // constant
		    quickStopAccumlator;

		right = left = linearCmd;
		left += angularCmd;
		right -= angularCmd;

		_updateAccumulators();
	}

    return std::make_pair(left*127, right*127);

	prevTurn = iturn;
	prevThrottle = ithrottle;
}

float expDriveCurve(float input, float scale, int type) {
    if (scale != 0) {
        if (type==0) {return int((powf(2.718, -(scale / 10)) + powf(2.718, (fabs(input) - 127) / 10) * (1 - powf(2.718, -(scale / 10)))) * input);}
		if (type==1) {return int((powf(2.718, (((fabs(input)-127)*scale)/1000))) * input);}
		//https://www.desmos.com/calculator/sdcgzah5ya
		// blue is 1 red is 0
    }
    return std::round(input);
}

std::pair<float, float> arcadeDrive(double ithrottle, double iturn, float curveGainY = 0, float curveGainX = 0, int curveTypeY = 0, int curveTypeX = 0) {

	ithrottle = expDriveCurve(ithrottle/127, curveGainY, curveTypeY);
	iturn = expDriveCurve(iturn/127, curveGainX, curveTypeX);

    bool turnInPlace = false;
	double linearCmd = ithrottle;
	if (fabs(ithrottle) < DRIVE_DEADBAND && fabs(iturn) > DRIVE_DEADBAND) {
		// The controller joysticks can output values near zero when they are
		// not actually pressed. In the case of small inputs like this, we
		// override the throttle value to 0.
		linearCmd = 0.0;
		turnInPlace = true;
	} else if (ithrottle - prevThrottle > DRIVE_SLEW) { // tune da slew
		linearCmd = prevThrottle + DRIVE_SLEW;
	} else if (ithrottle - prevThrottle < -(DRIVE_SLEW * 2)) {
		// We double the drive slew rate for the reverse direction to get
		// faster stopping.
		linearCmd = prevThrottle - (DRIVE_SLEW * 2);
	}

	double remappedTurn = _turnRemapping(iturn);

	double left, right;
	if (turnInPlace) {
		// The remappedTurn value is squared when turning in place. This
		// provides even more fine control over small speed values.
		left = remappedTurn * std::abs(remappedTurn);
		right = -remappedTurn * std::abs(remappedTurn);

		// The FRC Cheesy Drive Implementation calculated the
		// quickStopAccumulator here:
		// if (Math.abs(linearPower) < 0.2) {
		// 	double alpha = 0.1;
		// 	quickStopAccumulator = (1 - alpha) * quickStopAccumulator
		// 			+ alpha * Util.limit(wheel, 1.0) * 5;
		// }
		// But I apparently left that out of my implementation? Seemed to work
		// without it though.
	} else {
		double negInertiaPower = (iturn - prevTurn) * CD_NEG_INERTIA_SCALAR;
		negInertiaAccumlator += negInertiaPower;

		double angularCmd =
		        (remappedTurn + negInertiaAccumlator) *
		        CD_SENSITIVITY -  // we can scale down the turning amount by a
		                          // constant
		    quickStopAccumlator;

		right = left = linearCmd;
		left += angularCmd;
		right -= angularCmd;

		_updateAccumulators();
	}

    return std::make_pair(left*127, right*127);

	prevTurn = iturn;
	prevThrottle = ithrottle;
}

std::pair<float, float> arcade(int throttle, int turn, float curveGainY = 0, float curveGainX = 0, int curveTypeY = 0, int curveTypeX = 0) {
    throttle = expDriveCurve(throttle, curveGainY, curveTypeY);
    turn = expDriveCurve(turn, curveGainX, curveTypeX);

    float leftPower = throttle + turn;
    float rightPower = throttle - turn;
    return std::make_pair(leftPower, rightPower);
}

//drive function

std::pair<float, float> scale(float left, float right, int s = 100) {
	left *= s/100.0;
    right *= s/100.0;
    return std::make_pair(left, right);
}

void drive() {
	while (true) {
        if (master.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_X)) {
            scaleValue += 10;
        } else if (master.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_B)) {
            scaleValue -= 10;
        }

        double power = master.get_analog(pros::E_CONTROLLER_ANALOG_LEFT_Y);
		double turn = master.get_analog(pros::E_CONTROLLER_ANALOG_RIGHT_X);

        // auto [left, right] = cheesyDrive(power, turn);
		// auto [left, right] = arcadeDrive(power, turn, 6.2,4.2,1,0); //14.6

		auto [left, right] = arcade(power, turn, 6.2,4.2,1,0); //14.6
        auto[sleft, sright] = scale(left, right, scaleValue);

	    dt_left.move(sleft);
	    dt_right.move(sright);

		pros::delay(20);
	}
}