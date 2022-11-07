package org.firstinspires.ftc.teamcode.SwerveTest;

public interface Drive {

	enum State {
		STOPPED,
		MOVING
	}

	/**
	 * Moves the robot forward and backward.
	 *
	 * @param power power at which to run the motors (+ is forward, - is backwards)
	 */
	void move( double power );

	/**
	 * Turns the robot on the spot
	 *
	 * @param power power at which to run the motors (+ is a right turn, - is a left turn)
	 */
	void turn( double power );

	/**
	 * Stops Motors
	 */
	void stop( );

	/**
	 * Sets power to the wheel motors
	 *
	 * @param move power for forward and back motion
	 * @param turn power for rotating the robot
	 */
	void drive( double move, double turn );

	/**
	 * Returns the current state of the drive train
	 *
	 * @return current state
	 */
	State getState( );

	/**
	 * convert a distance (in inches) to ticks
	 *
	 * @param distance            the distance you want to convert to ticks
	 * @param circumference       the circumference of the encoder wheel
	 * @param pulsesPerRevolution the encoder's pulses per revolution
	 * @param gearRatio           the ratio of the gears
	 * @return the number of ticks in that distance
	 */
	static int convertDistTicks( double distance, double circumference, double pulsesPerRevolution, double gearRatio ) {
		double revolutions = distance / circumference;
		return (int) Math.round( pulsesPerRevolution * revolutions / gearRatio );
	}

	/**
	 * convert a number of ticks to distance (in inches)
	 *
	 * @param ticks               the ticks you want to convert to distance
	 * @param circumference       the circumference of the encoder wheel
	 * @param pulsesPerRevolution the encoder's pulses per revolution
	 * @param gearRatio           the ratio of the gears
	 * @return the distance (in inches) in that number of ticks
	 */
	static double convertTicksDist( int ticks, double circumference, double pulsesPerRevolution, double gearRatio ) {
		return (ticks * circumference * gearRatio) / pulsesPerRevolution;
	}

	/**
	 * @param inputNum the number to be normalized
	 * @param oldMin   the minimum input
	 * @param oldMax   the maximum input
	 * @param newMin   the minimum output
	 * @param newMax   the maximum output
	 * @return the input number normalized to the new range
	 */
	static double normalize( double inputNum, double oldMin, double oldMax, double newMin, double newMax ) {
		return (inputNum - oldMin) / (oldMax - oldMin) * (newMax - newMin) + newMin;
	}


}
