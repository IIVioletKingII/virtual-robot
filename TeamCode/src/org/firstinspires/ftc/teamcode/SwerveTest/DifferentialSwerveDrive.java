package org.firstinspires.ftc.teamcode.SwerveTest;

import static com.qualcomm.robotcore.hardware.DcMotor.RunMode.RUN_USING_ENCODER;
import static com.qualcomm.robotcore.hardware.DcMotor.RunMode.STOP_AND_RESET_ENCODER;
import static com.qualcomm.robotcore.hardware.DcMotor.ZeroPowerBehavior.BRAKE;
import static com.qualcomm.robotcore.hardware.DcMotorSimple.Direction.REVERSE;
import static java.lang.Math.sqrt;

import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.util.Angle;
import com.qualcomm.robotcore.hardware.DcMotor.RunMode;
import com.qualcomm.robotcore.hardware.DcMotor.ZeroPowerBehavior;
import com.qualcomm.robotcore.hardware.DcMotorExImpl;
import com.qualcomm.robotcore.hardware.DcMotorSimple.Direction;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

import java.text.DecimalFormat;

public class DifferentialSwerveDrive implements Drive {

	public DcMotorExImpl topLeft;
	public DcMotorExImpl bottomLeft;
	public DcMotorExImpl topRight;
	public DcMotorExImpl bottomRight;

	final double PULSES_PER_REVOLUTION = 560; // 537.7

	private State currentState = State.STOPPED;

	public double wheelRadius;
	public double wheelGearRatio;

	public double rotateGearRadius;
	public double rotateGearRatio;

	public double maxAngularVelocity;
	public double storedMaxAngularPow;
	public double storedMaxAngularVel;

	public double wheelBase;
	/**
	 * sussy balls
	 */
	int ticksInRotation = 250;

	double driveWeight = 1;
	double rotateWeight = 1;

	double headingWeight = 1;

	final double TWO_PI = 2 * Math.PI;

	double MAX_VELOCITY = 5 * TWO_PI; // Radians per seconds

	public DifferentialSwerveDrive( HardwareMap hardwareMap ) {
		this( hardwareMap, "topLeft", "bottomLeft", "topRight", "bottomRight" );
	}

	/**
	 * Sets up motors from the hardware map
	 *
	 * @param hardwareMap     robot's hardware map
	 * @param topLeftName     name of left bottom motor in the hardware map
	 * @param bottomLeftName  name of right bottom motor in the hardware map
	 * @param topRightName    name of left top motor in the hardware map
	 * @param bottomRightName name of right top motor in the hardware map
	 */
	public DifferentialSwerveDrive( HardwareMap hardwareMap, String topLeftName, String bottomLeftName, String topRightName, String bottomRightName ) {
		topLeft = hardwareMap.get( DcMotorExImpl.class, topLeftName );
		bottomLeft = hardwareMap.get( DcMotorExImpl.class, bottomLeftName );
		topRight = hardwareMap.get( DcMotorExImpl.class, topRightName );
		bottomRight = hardwareMap.get( DcMotorExImpl.class, bottomRightName );

//		setMotorDirections( FORWARD, FORWARD, FORWARD, FORWARD );
		setMotorDirections( REVERSE, REVERSE, REVERSE, REVERSE );
		setZeroPowerBehavior( BRAKE, BRAKE, BRAKE, BRAKE );
		setRunMode( STOP_AND_RESET_ENCODER, STOP_AND_RESET_ENCODER, STOP_AND_RESET_ENCODER, STOP_AND_RESET_ENCODER );
		setRunMode( RUN_USING_ENCODER, RUN_USING_ENCODER, RUN_USING_ENCODER, RUN_USING_ENCODER );
	}

	public void setUpWheelRatios( double wheelRadius, double wheelGearRatio, double rotateGearRadius, double rotateGearRatio ) {

		this.wheelRadius = wheelRadius;
		this.wheelGearRatio = wheelGearRatio;

		this.rotateGearRadius = rotateGearRadius;
		this.rotateGearRatio = rotateGearRatio;

		ticksInRotation = Drive.convertDistTicks( TWO_PI, TWO_PI * rotateGearRadius, this.rotateGearRatio, PULSES_PER_REVOLUTION );

		storedMaxAngularPow = 2 / wheelBase;
		storedMaxAngularVel = 2 * maxAngularVelocity / wheelBase;
	}

	public void setMovementWeights( double driveWeight, double rotateWeight ) {
		this.driveWeight = driveWeight;
		this.rotateWeight = rotateWeight;
	}

	/**
	 * @param maxAngularVelocity the new max rotation speed of a wheel pod in rad / s
	 */
	public void setMaxAngularVelocity( double maxAngularVelocity ) {
		this.maxAngularVelocity = maxAngularVelocity;
		storedMaxAngularPow = 2 / wheelBase;
		storedMaxAngularVel = 2 * maxAngularVelocity / wheelBase;
	}

	public void setWheelBase( double wheelBase ) {
		this.wheelBase = wheelBase;
	}

	/**
	 * @param topMotor    the motor controlling the wheel hub's upper gear
	 * @param bottomMotor the motor controlling the wheel hub's lower gear
	 * @return the number of ticks the motor hub has rotated
	 */
	public int getWheelRotation( DcMotorExImpl topMotor, DcMotorExImpl bottomMotor ) {
		return (topMotor.getCurrentPosition( ) + bottomMotor.getCurrentPosition( )) / 2;
	}

	/**
	 * @param topMotor    the motor controlling the wheel hub's upper gear
	 * @param bottomMotor the motor controlling the wheel hub's lower gear
	 * @param angleUnit   the angle unit to measure the wheel's rotation
	 * @return the number of angleUnit the motor hub has rotated
	 */
	public double getWheelRotation( DcMotorExImpl topMotor, DcMotorExImpl bottomMotor, AngleUnit angleUnit ) {
		int rotatedTicks = getWheelRotation( topMotor, bottomMotor );
		double rotations = rotatedTicks / (PULSES_PER_REVOLUTION * 4/*rotateGearRatio*/);
		double radians = rotations * TWO_PI;
//		System.out.println( "average position: " + rotatedTicks );
//		System.out.println( "rotations: " + rotations );
//		System.out.println( "degrees rotated: " + Math.toDegrees( radians ) );

//		double radians = Drive.convertTicksDist( rotatedTicks, TWO_PI * rotateGearRadius, rotateGearRatio, PULSES_PER_REVOLUTION );
		return (angleUnit == AngleUnit.RADIANS) ? radians : Math.toDegrees( radians );
	}

	/**
	 * @param angle the input angle between 0 and 2π
	 * @return the angle normalized between 0 and 1
	 */
	public double normalizeAngle( double angle ) {
		return Drive.normalize( angle, 0, TWO_PI, 0, 1 );
	}

	/**
	 * @param vectorAngle the angle from a vector
	 * @return the angle rotated 90° counterclockwise
	 */
	public double normalizeVectorAngle( double vectorAngle ) {
		return Angle.norm( vectorAngle + Math.PI / 2 );
	}

	/**
	 * @param vector the vector to get an angle from
	 * @return the angle rotated 90° counterclockwise
	 */
	public double normalizeVectorAngle( Vector2d vector ) {
		return normalizeVectorAngle( vector.angle( ) );
	}

	public double removeNegZero( double number ) {
		return number != 0 ? number : 0.0;
	}

	public void move( double drive, double strafe, double rotate ) {

//		Vector2d move = new Vector2d( drive, strafe );
//
//		double moveX = move.getX( );
//		double moveY = move.getY( );
//
//		double power = sqrt( moveX * moveX + moveY * moveY - (moveX * moveX * moveY * moveY) );

		move( new Vector2d( drive, strafe ), removeNegZero( rotate ) ); // this one

//		move( Angle.norm( Math.atan2( strafe, drive ) ), sqrt( drive * drive + strafe * strafe - (drive * drive * strafe * strafe) ), removeNegZero( rotate ), 1 );
	}

	public boolean valueBetweenAB( double input, double a, double b, boolean inclusive ) {
		return (!inclusive && input > a && input < b) || (inclusive && input >= a && input <= b);
	}

	private void move( Vector2d move, double rotate ) {
		DecimalFormat decimalFormat = new DecimalFormat( "0.000" );

		double moveX = move.getX( );
		double moveY = move.getY( );

		double angle = move.angle( );

		System.out.println( "angle: " + Math.toDegrees( angle ) );

		// if target angle
		double leftTargetHeading = angle / rotate ;
		double rightTargetHeading = angle;

		double targetHubAngle = angle - Math.abs( rotate ) * angle; // Angle.norm( Math.atan2( moveY, moveX ) ); // 0 is straight, pi/2 (90) is perp (to the right)
		double power = sqrt( moveX * moveX + moveY * moveY - (moveX * moveX * moveY * moveY) );

		double leftRotation = getWheelRotation( topLeft, bottomLeft, AngleUnit.RADIANS );
		double rightRotation = getWheelRotation( topRight, bottomRight, AngleUnit.RADIANS );

//		double angularPower = rotate * storedMaxAngularPow;

		double leftTurnAngle = targetHubAngle % Math.PI - (leftRotation % Math.PI + TWO_PI) % Math.PI;

		if( Math.abs( leftTurnAngle ) > Math.PI / 2 )
			leftTurnAngle -= Math.signum( leftTurnAngle ) * Math.PI;
		double leftTurnPower = 2 * leftTurnAngle / Math.PI;

		double rightTurnAngle = targetHubAngle % Math.PI - (rightRotation % Math.PI + TWO_PI) % Math.PI;
		if( Math.abs( rightTurnAngle ) > Math.PI / 2 )
			rightTurnAngle -= Math.signum( rightTurnAngle ) * Math.PI;
		double rightTurnPower = 2 * rightTurnAngle / Math.PI;

		boolean leftForward = valueBetweenAB( leftRotation, targetHubAngle - Math.toRadians( 90 ), targetHubAngle + Math.toRadians( 90 ), false ) || valueBetweenAB( leftRotation, targetHubAngle + Math.toRadians( 270 ), targetHubAngle + Math.toRadians( 450 ), false );
		boolean rightForward = valueBetweenAB( rightRotation, targetHubAngle - Math.toRadians( 90 ), targetHubAngle + Math.toRadians( 90 ), false ) || valueBetweenAB( rightRotation, targetHubAngle + Math.toRadians( 270 ), targetHubAngle + Math.toRadians( 450 ), false );

		// sets direction of power
		leftRotation = (leftRotation % TWO_PI + TWO_PI) % TWO_PI;
		rightRotation = (rightRotation % TWO_PI + TWO_PI) % TWO_PI;
		double leftDrivePow = power * (leftForward ? 1 : -1);
		double rightDrivePow = power * (rightForward ? 1 : -1);

		leftDrivePow += rotate * (leftForward ? 1 : -1);
		rightDrivePow -= rotate * (rightForward ? 1 : -1);

		// just for rotating (aka turning the small wheel)
//		double leftRotatePow = angularPower * normalizeAngle( /*targetRotateAngle*/ -leftRotation );
//		double rightRotatePow = angularPower * normalizeAngle( /*targetRotateAngle*/ -rightRotation );

//		System.out.println( "targetHubAngle: " + Math.toDegrees( targetHubAngle ) + "°" );
//		System.out.println( "rotation (l, r): " + Math.toDegrees( leftRotation ) + ", " + Math.toDegrees( rightRotation ) );
//		System.out.println( "turn angle (l, r): " + decimalFormat.format( Math.toDegrees( leftTurnAngle ) ) + ", " + decimalFormat.format( Math.toDegrees( rightTurnAngle ) ) );
//		System.out.println( "turn power (l, r): " + leftTurnPower + ", " + rightTurnPower );
////		System.out.println( "right turn power (real, test 1-3): " + rightTurnPower + ", " + testPower1 + ", " + testPower2 + ", " + testPower3 );
//		System.out.println( "drive power (l, r): " + leftDrivePow + ", " + rightDrivePow );
//		System.out.println( "normPower: " + normPower );
//		System.out.println( "-----------------" );
//		System.out.println( "lD: " + decimalFormat.format( leftDrivePow ) + ", rD: " + decimalFormat.format( rightDrivePow ) );//+ ", p: " + decimalFormat.format(normPower) );

		double[] powers = new double[]{
				-leftDrivePow * driveWeight + leftTurnPower * rotateWeight,
				leftDrivePow * driveWeight + leftTurnPower * rotateWeight,
				rightDrivePow * driveWeight + rightTurnPower * rotateWeight,
				-rightDrivePow * driveWeight + rightTurnPower * rotateWeight
		};

		double max = 1;
		for( int i = 0; i < powers.length; i++ )
			if( Math.abs( powers[i] ) > max )
				max = Math.abs( powers[i] );


		topLeft.setPower( powers[0] / max );
		bottomLeft.setPower( powers[1] / max );

		topRight.setPower( powers[2] / max );
		bottomRight.setPower( powers[3] / max );

//		topLeft.setPower( (-leftDrivePow + leftTurnPower) / denom );
//		bottomLeft.setPower( (leftDrivePow + leftTurnPower) / denom );
//
//		topRight.setPower( (rightDrivePow + rightTurnPower) / denom );
//		bottomRight.setPower( (-rightDrivePow + rightTurnPower) / denom );
	}

	public static String format( double input, double decimals ) {
		return String.format( "%" + Integer.toString( (int) input ).length( ) + "." + decimals + "f", input );
	}

	@Override
	public void drive( double drive, double strafe ) {

		drive( new Vector2d( strafe, drive ) );
	}

	public void drive( Vector2d strafe ) {

		double targetHubAngle = normalizeVectorAngle( strafe );
		double power = strafe.norm( );

		double leftStrafePow = normalizeAngle( targetHubAngle - getWheelRotation( topLeft, bottomLeft, AngleUnit.RADIANS ) );
		double rightStrafePow = normalizeAngle( targetHubAngle - getWheelRotation( topRight, bottomRight, AngleUnit.RADIANS ) );

		// if the target angle = the current angle (of the hubs) then it will just drive

		topLeft.setPower( power * (1 + leftStrafePow) ); // norm + norm * strafe
		bottomLeft.setPower( power * (1 - leftStrafePow) ); // norm - norm * strafe

		topRight.setPower( power * (1 + rightStrafePow) ); // norm + norm * strafe
		bottomRight.setPower( power * (1 - rightStrafePow) ); // norm - norm * strafe
	}

	@Override
	public void move( double power ) {

		topLeft.setPower( power );
		bottomLeft.setPower( power );
		topRight.setPower( power );
		bottomRight.setPower( power );
	}

	@Override
	public void turn( double power ) {

		double leftRotation = getWheelRotation( topLeft, bottomLeft, AngleUnit.RADIANS );
		double rightRotation = getWheelRotation( topRight, bottomRight, AngleUnit.RADIANS );

		double targetRotateAngle = 0; // perpendicular to robot's point of rotation

		topLeft.setPower( power );
		bottomLeft.setPower( power * normalizeAngle( targetRotateAngle - leftRotation ) );

		topRight.setPower( power );
		bottomRight.setPower( power * normalizeAngle( targetRotateAngle - rightRotation ) );

	}

	public void moveWheel( double power, double distance, DcMotorExImpl topMotor, DcMotorExImpl bottomMotor ) {

		topMotor.setMode( RunMode.RUN_TO_POSITION );
		bottomMotor.setMode( RunMode.RUN_TO_POSITION );

		int target = Drive.convertDistTicks( distance, TWO_PI * wheelRadius, rotateGearRadius, PULSES_PER_REVOLUTION );
		topMotor.setTargetPosition( target );
		topMotor.setTargetPosition( target );

		topMotor.setPower( power );
		bottomMotor.setPower( -power );

		new Thread( ( ) -> {
			while( topMotor.isBusy( ) || bottomMotor.isBusy( ) ) ;
			topMotor.setPower( 0 );
			bottomMotor.setPower( 0 );
		} ).start( );
	}

	@Override
	public void stop( ) {
		setMotorPower( 0, 0, 0, 0 );
	}

	@Override
	public State getState( ) {
		updateState( );
		return currentState;
	}

	private void updateState( ) {
		currentState = (topLeft.getPower( ) != 0 || topRight.getPower( ) != 0 || bottomLeft.getPower( ) != 0 || bottomRight.getPower( ) != 0)
				? State.MOVING : State.STOPPED;
	}

	/**
	 * Sets specified power to the motors
	 *
	 * @param topLeftPower     power at which to run the front left motor.
	 * @param bottomLeftPower  power at which to run the back left motor.
	 * @param topRightPower    power at which to run the front right motor.
	 * @param bottomRightPower power at which to run the back right motor.
	 */
	protected void setMotorPower( double topLeftPower, double bottomLeftPower, double topRightPower, double bottomRightPower ) {
		topLeft.setPower( topLeftPower );
		bottomLeft.setPower( bottomLeftPower );
		topRight.setPower( topRightPower );
		bottomRight.setPower( bottomRightPower );
	}

	/**
	 * Sets the direction of the motors
	 *
	 * @param topLeftDirection     direction of the front left motor
	 * @param bottomLeftDirection  direction of the back left motor
	 * @param topRightDirection    direction of the front right motor
	 * @param bottomRightDirection direction of the back right motor
	 */
	public void setMotorDirections( Direction topLeftDirection, Direction bottomLeftDirection, Direction topRightDirection, Direction bottomRightDirection ) {
		topLeft.setDirection( topLeftDirection );
		bottomLeft.setDirection( bottomLeftDirection );
		topRight.setDirection( topRightDirection );
		bottomRight.setDirection( bottomRightDirection );
	}

	/**
	 * Sets the zero power behavior of the motors
	 *
	 * @param topLeftBehavior     zero power behavior of the front left motor
	 * @param bottomLeftBehavior  zero power behavior of the back left motor
	 * @param topRightBehavior    zero power behavior of the front right motor
	 * @param bottomRightBehavior zero power behavior of the back right motor
	 */
	public void setZeroPowerBehavior( ZeroPowerBehavior topLeftBehavior, ZeroPowerBehavior bottomLeftBehavior, ZeroPowerBehavior topRightBehavior, ZeroPowerBehavior bottomRightBehavior ) {
		topLeft.setZeroPowerBehavior( topLeftBehavior );
		bottomLeft.setZeroPowerBehavior( bottomLeftBehavior );
		topRight.setZeroPowerBehavior( topRightBehavior );
		bottomRight.setZeroPowerBehavior( bottomRightBehavior );
	}

	/**
	 * Sets the run modes of the motors
	 *
	 * @param topLeftMode     run mode of the front left motor
	 * @param bottomLeftMode  run mode of the back left motor
	 * @param topRightMode    run mode of the front right motor
	 * @param bottomRightMode run mode of the back right motor
	 */
	public void setRunMode( RunMode topLeftMode, RunMode bottomLeftMode, RunMode topRightMode, RunMode bottomRightMode ) {
		topLeft.setMode( topLeftMode );
		bottomLeft.setMode( bottomLeftMode );
		topRight.setMode( topRightMode );
		bottomRight.setMode( bottomRightMode );
	}
}
