package org.firstinspires.ftc.teamcode.SwerveTest;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import com.qualcomm.robotcore.hardware.DcMotorExImpl;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

@TeleOp(name = "Swerve Test TeleOp", group = "TeleOp")
public class DiffSwerveTeleOp extends OpMode {

	DifferentialSwerveDrive drive;

	DcMotorExImpl motor;
	DcMotorExImpl motor2;

	final double rotateGearRatio = (18 / 15.0) * (32 / 13.0) * (107 / 32.0);
	final double wheelGearRatio = rotateGearRatio * (18 / 63.0);

	@Override
	public void init( ) {

		drive = new DifferentialSwerveDrive( hardwareMap, "top_left_motor", "bottom_left_motor", "top_right_motor", "bottom_right_motor" );
		drive.setWheelBase( 14 );
		drive.setUpWheelRatios( 2.25 / 2, wheelGearRatio, 1.703, rotateGearRatio );
		drive.setMovementWeights( 1, 5 );
//		drive.setMaxAngularVelocity( Math.PI / 2 );

//		motor = hardwareMap.get( DcMotorExImpl.class, "top_left_motor" );
//		motor2 = hardwareMap.get( DcMotorExImpl.class, "bottom_left_motor" );
//
//		// with both motors at full power your max 0.9 s per rotation
//		// 1.11 rot/sec
//
//		motor.setMode( DcMotor.RunMode.RUN_USING_ENCODER );
//		motor2.setMode( DcMotor.RunMode.RUN_USING_ENCODER );
//		motor.setPower( 1 );
//		motor2.setPower( 1 );
////		motor.setVelocity( 5 * (2 * Math.PI), AngleUnit.RADIANS );
//		doForTime( 3, ( ) -> {
//			telemetry.addLine( "Velocity: " + motor.getVelocity( AngleUnit.RADIANS ) );
//			telemetry.update( );
//		} );
//		motor.setVelocity( 0 );
//		motor2.setPower( 0 );


		telemetry.addLine( "Init Finished" );
		telemetry.update( );
	}

	public void doForTime( double seconds, Runnable run ) {
		double startTime = getRuntime( );
		while( startTime + seconds > getRuntime( ) )
			run.run( );
	}

	@Override
	public void loop( ) {
		telemetry.addLine( "LY: " + -gamepad1.left_stick_y );
		telemetry.addLine( "LX: " + gamepad1.left_stick_x );
		telemetry.addLine( "RX: " + gamepad1.right_stick_x );
//		telemetry.addLine( "Ld: " + motor.getCurrentPosition( ) );
//		telemetry.addLine( "Ld: " + motor.getCurrentPosition( ) );
		telemetry.update( );
//		motor.setPower( -gamepad1.left_stick_y );
//		motor2.setPower( -gamepad1.right_stick_y );
		drive.move( -gamepad1.left_stick_y, gamepad1.left_stick_x, gamepad1.right_stick_x );
	}

}
