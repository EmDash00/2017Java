package org.usfirst.frc.team9135.robot.commands;

import org.usfirst.frc.team9135.robot.CommandBase;
import org.usfirst.frc.team9135.robot.OI;
import org.usfirst.frc.team9135.robot.subsystems.*;

import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/**
 *
 */
public class DriveWithJoysticks extends Command {
	private double leftJoystickValue = 0.0;
	private double rightJoystickValue = 0.0;

	private double convertedLeftJoystickValue = 0.0;
	private double convertedRightJoystickValue = 0.0;

	private double actualUsedLeftJoystickValue = 0.0;
	private double actualUsedRightJoystickValue = 0.0;

	private static final double POV_DRIVE_TRAIN_MOTOR_POWER = .65;

	private double gyroAngle = 0.0;

	private boolean throttleUp = false;
	private double povMotorPower = 0.0;

	private static final double LOW_POV_DRIVE_TRAIN_MOTOR_POWER = .3;
	private static final double HIGH_POV_DRIVE_TRAIN_MOTOR_POWER = .7;


	private boolean rightDriveEncoderDetected = false;
	private boolean leftDriveEncoderDetected = false;

	private double desiredMaxMotorPower = 0.0;
	private boolean fastMotorPower = false;

	private int rightEncoderValue = 0;
	private int leftEncoderValue = 0;
	
    public DriveWithJoysticks() {
        // Use requires() here to declare subsystem dependencies
        // eg. requires(chassis);
    	requires(CommandBase.driveTrain);
    }

    // Called just before this Command runs the first time
    protected void initialize() {
    	CommandBase.driveTrain.ZeroDriveTrainEncoder(DriveTrain.LEFT_SIDE_ENCODER);
    	CommandBase.driveTrain.ZeroDriveTrainEncoder(DriveTrain.RIGHT_SIDE_ENCODER);
    	CommandBase.driveTrain.ZeroGyroAngle();
    }

    // Called repeatedly when this Command is scheduled to run
    protected void execute() {

    	/*rightDriveEncoderDetected = CommandBase.driveTrain.MakeSureDriveTrainEncoderIsPluggedIn(DriveTrain.RIGHT_SIDE_ENCODER_BOOLEAN);
    	leftDriveEncoderDetected = CommandBase.driveTrain.MakeSureDriveTrainEncoderIsPluggedIn(DriveTrain.LEFT_SIDE_ENCODER_BOOLEAN);
    	if (rightDriveEncoderDetected == false) {
    		std.cout << "Right Drive Train Encoder Is Unplugged" << std.endl;
    	}

    	if (leftDriveEncoderDetected == false) {
    		std.cout << "Left Drive Train Encoder Is Unplugged" << std.endl;
    	} */

    	leftJoystickValue = CommandBase.oi.GetYAxis(OI.LEFT_DRIVE_JOYSTICK);
    	rightJoystickValue = CommandBase.oi.GetYAxis(OI.RIGHT_DRIVE_JOYSTICK);

    	desiredMaxMotorPower = CommandBase.driveTrain.getDesiredDriveTrainMaxMotorPower();

    	if (desiredMaxMotorPower == DriveTrain.FAST_MAX_DRIVE_TRAIN_MOTOR_POWER) {
    		fastMotorPower = true;
    	}
    	else if (desiredMaxMotorPower == DriveTrain.SLOW_MAX_DRIVE_TRAIN_MOTOR_POWER) {
    		fastMotorPower = false;
    	}

    	SmartDashboard.putBoolean("Full Drive Train Motor Power", fastMotorPower);

    	convertedLeftJoystickValue = (leftJoystickValue * desiredMaxMotorPower);
    	convertedRightJoystickValue = (rightJoystickValue * desiredMaxMotorPower);

    	CommandBase.driveTrain.DriveTank(convertedLeftJoystickValue, convertedRightJoystickValue);

    	throttleUp = CommandBase.oi.GetThrottleUp(OI.RIGHT_DRIVE_JOYSTICK);
    	if (throttleUp) {
    		povMotorPower = HIGH_POV_DRIVE_TRAIN_MOTOR_POWER;
    	}
    	else if (throttleUp == false) {
    		povMotorPower = LOW_POV_DRIVE_TRAIN_MOTOR_POWER;
    	}

    	gyroAngle = CommandBase.driveTrain.getGyroAngle();
    	SmartDashboard.putNumber("Gyro Angle", gyroAngle);

    	if (CommandBase.oi.POVDirectionPressed(OI.RIGHT_DRIVE_JOYSTICK, OI.TOP_POV)) {
    		CommandBase.driveTrain.DriveStraightWithGyro(povMotorPower, gyroAngle);
    	}
    	else if (CommandBase.oi.POVDirectionPressed(OI.RIGHT_DRIVE_JOYSTICK, OI.RIGHT_POV)) {
    		CommandBase.driveTrain.DriveTank(POV_DRIVE_TRAIN_MOTOR_POWER, -POV_DRIVE_TRAIN_MOTOR_POWER);
    	}
    	else if (CommandBase.oi.POVDirectionPressed(OI.RIGHT_DRIVE_JOYSTICK, OI.BOTTOM_POV)) {
    		CommandBase.driveTrain.DriveStraightWithGyro(-povMotorPower, gyroAngle);
    	}
    	else if (CommandBase.oi.POVDirectionPressed(OI.RIGHT_DRIVE_JOYSTICK, OI.LEFT_POV)) {
    		CommandBase.driveTrain.DriveTank(-POV_DRIVE_TRAIN_MOTOR_POWER, POV_DRIVE_TRAIN_MOTOR_POWER);
    	}
    	else {
    		CommandBase.driveTrain.ZeroGyroAngle();
    	}

    	rightEncoderValue = CommandBase.driveTrain.getEncoderPosition(DriveTrain.RIGHT_SIDE_ENCODER);
    	leftEncoderValue = CommandBase.driveTrain.getEncoderPosition(DriveTrain.LEFT_SIDE_ENCODER);
    	SmartDashboard.putNumber("Right Drive Train Encoder Value", rightEncoderValue);
    	SmartDashboard.putNumber("Left Drive Train Encoder Value", leftEncoderValue);
    }

    // Make this return true when this Command no longer needs to run execute()
    protected boolean isFinished() {
        return false;
    }

    // Called once after isFinished returns true
    protected void end() {
    	CommandBase.driveTrain.DriveTank(0.0, 0.0);
    }

    // Called when another command which requires one or more of the same
    // subsystems is scheduled to run
    protected void interrupted() {
    	end();
    }
}
