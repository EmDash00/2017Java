package org.usfirst.frc.team9135.robot.commands;

import org.usfirst.frc.team9135.robot.CommandBase;
import org.usfirst.frc.team9135.robot.subsystems.DriveTrain;

import edu.wpi.first.wpilibj.command.Command;

/**
 *
 */
public class DriveDistance extends Command {

	private double desiredDistanceToTravel;
	private double motorPower;
	private boolean zeroDriveTrainEncoders;

	private boolean zeroedDriveTrainEncoders = false;

	private double initialDistanceTraveled = 0.0;
	private double measuredCurrentDistanceTraveled = 0.0;
	private double actualCurrentDistanceTraveled = 0.0;
	private boolean measuredInitialDistanceTraveled = false;

	private boolean distanceTraveled = false;

	private double gyroAngle = 0.0;
	private boolean zeroGyro = false;

	private boolean requiresDriveTrain = false;
	
    public DriveDistance(double desiredDistanceToTravel, double motorPower, boolean zeroDriveTrainEncoders) {
        // Use requires() here to declare subsystem dependencies
        // eg. requires(chassis);
    	requires(CommandBase.driveTrain);
    	this.desiredDistanceToTravel = desiredDistanceToTravel;
    	this.motorPower = motorPower;
    	this.zeroDriveTrainEncoders = zeroDriveTrainEncoders;
    }

    // Called just before this Command runs the first time
    protected void initialize() {
    	initialDistanceTraveled = CommandBase.driveTrain.getDistance(DriveTrain.RIGHT_SIDE_ENCODER);
    	measuredInitialDistanceTraveled = true;
    	distanceTraveled = false;
    	CommandBase.driveTrain.ZeroGyroAngle();
    	zeroGyro = true;
    	CommandBase.driveTrain.is_aiming = true;
    	requiresDriveTrain = true;
    	if (this.zeroDriveTrainEncoders) {
    		CommandBase.driveTrain.ZeroDriveTrainEncoder(DriveTrain.RIGHT_SIDE_ENCODER);
    		CommandBase.driveTrain.ZeroDriveTrainEncoder(DriveTrain.LEFT_SIDE_ENCODER);
    		zeroedDriveTrainEncoders = true;
    	}
    }

    // Called repeatedly when this Command is scheduled to run
    protected void execute(){
    	if (requiresDriveTrain == false) {
    		CommandBase.driveTrain.is_aiming = true;
    		requiresDriveTrain = true;
    	}

    	if (this.zeroDriveTrainEncoders && zeroedDriveTrainEncoders == false) {
    		CommandBase.driveTrain.ZeroDriveTrainEncoder(DriveTrain.RIGHT_SIDE_ENCODER);
    		CommandBase.driveTrain.ZeroDriveTrainEncoder(DriveTrain.LEFT_SIDE_ENCODER);
    		zeroedDriveTrainEncoders = true;
    	}

    	if (measuredInitialDistanceTraveled == false) {
    		initialDistanceTraveled = CommandBase.driveTrain.getDistance(DriveTrain.RIGHT_SIDE_ENCODER);
    		measuredInitialDistanceTraveled = true;
    	}

    	if (zeroGyro == false) {
    		CommandBase.driveTrain.ZeroGyroAngle();
    		zeroGyro = true;
    	}

    	gyroAngle = CommandBase.driveTrain.getGyroAngle();

    	measuredCurrentDistanceTraveled = CommandBase.driveTrain.getDistance(DriveTrain.RIGHT_SIDE_ENCODER);
    	actualCurrentDistanceTraveled = (Math.abs(measuredCurrentDistanceTraveled - initialDistanceTraveled));
    	if (actualCurrentDistanceTraveled >= this.desiredDistanceToTravel) {
    		CommandBase.driveTrain.DriveStraightWithGyro(0.0, 0.0);
    		distanceTraveled = true;
    	}
    	else {
    		CommandBase.driveTrain.DriveStraightWithGyro(this.motorPower, gyroAngle);
    	}
    }
    

    // Make this return true when this Command no longer needs to run execute()
    protected boolean isFinished() {
    	return distanceTraveled;
    }

    // Called once after isFinished returns true
    protected void end() {
    	CommandBase.driveTrain.DriveTank(0.0, 0.0);
    	zeroedDriveTrainEncoders = false;
    	distanceTraveled = false;
    	measuredInitialDistanceTraveled = false;
    	zeroGyro = false;
    	CommandBase.driveTrain.is_aiming = false;
    }

    // Called when another command which requires one or more of the same
    // subsystems is scheduled to run
    protected void interrupted() {
    	end();
    }
}
