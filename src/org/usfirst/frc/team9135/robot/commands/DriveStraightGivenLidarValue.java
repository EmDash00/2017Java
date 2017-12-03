package org.usfirst.frc.team9135.robot.commands;

import org.usfirst.frc.team9135.robot.CommandBase;
import org.usfirst.frc.team9135.robot.subsystems.*;

import edu.wpi.first.wpilibj.command.Command;

/**
 *
 */
public class DriveStraightGivenLidarValue extends Command {
	private double driveTrainMotorPower;

	private boolean getDesiredDistanceToTravel = false;
	private double frontUltrasonicSensorValue = 0.0;
	private double desiredDistanceToTravel = 0.0;

	private boolean initializeDistanceTraveled = false;
	private double initialDistanceTraveled = 0.0;
	private double currentDistanceTraveled = 0.0;
	private double differenceBetweenCurrentAndInitialDistance = 0.0;

	private boolean zeroGyro = false;
	private double currentGyroAngle = 0.0;

	private boolean droveToDistance = false;
    public DriveStraightGivenLidarValue() {
        // Use requires() here to declare subsystem dependencies
        // eg. requires(chassis);
    	requires(CommandBase.lidars);
    	requires(CommandBase.ultrasonicSensor);
    	requires(CommandBase.driveTrain);

    	this.driveTrainMotorPower = driveTrainMotorPower;
    }

    // Called just before this Command runs the first time
    protected void initialize() {
    	frontUltrasonicSensorValue = CommandBase.ultrasonicSensor.GetGearUltrasonicSensorValueInches();
    	desiredDistanceToTravel = CommandBase.lidars.GetDistanceToTravelToHopper(frontUltrasonicSensorValue);
    	getDesiredDistanceToTravel = true;

    	initialDistanceTraveled = CommandBase.driveTrain.getDistance(DriveTrain.RIGHT_SIDE_ENCODER);
    	initializeDistanceTraveled = true;

    	CommandBase.driveTrain.ZeroGyroAngle();
    	zeroGyro = true;

    	droveToDistance = false;
    }

    // Called repeatedly when this Command is scheduled to run
    protected void execute() {
    	if (getDesiredDistanceToTravel == false) {
    		frontUltrasonicSensorValue = CommandBase.ultrasonicSensor.GetGearUltrasonicSensorValueInches();
    		desiredDistanceToTravel = CommandBase.lidars.GetDistanceToTravelToHopper(frontUltrasonicSensorValue);
    		getDesiredDistanceToTravel = true;
    	}

    	if (initializeDistanceTraveled == false) {
    		initialDistanceTraveled = CommandBase.driveTrain.getDistance(DriveTrain.RIGHT_SIDE_ENCODER);
    		initializeDistanceTraveled = true;
    	}

    	if (zeroGyro == false) {
    		CommandBase.driveTrain.ZeroGyroAngle();
    		zeroGyro = true;
    	}

    	currentGyroAngle = CommandBase.driveTrain.getGyroAngle();

    	currentDistanceTraveled = CommandBase.driveTrain.getDistance(DriveTrain.RIGHT_SIDE_ENCODER);
    	differenceBetweenCurrentAndInitialDistance = (Math.abs(currentDistanceTraveled - initialDistanceTraveled));
    	if (differenceBetweenCurrentAndInitialDistance >= desiredDistanceToTravel) {
    		CommandBase.driveTrain.DriveTank(0.0, 0.0);
    		droveToDistance = true;
    	}
    	else if (differenceBetweenCurrentAndInitialDistance < desiredDistanceToTravel) {
    		CommandBase.driveTrain.DriveStraightWithGyro(this.driveTrainMotorPower, currentGyroAngle);
    	}
    }

    // Make this return true when this Command no longer needs to run execute()
    protected boolean isFinished() {
    	return droveToDistance;
    }

    // Called once after isFinished returns true
    protected void end() {
    	CommandBase.driveTrain.DriveTank(0.0, 0.0);
    	getDesiredDistanceToTravel = false;
    	initializeDistanceTraveled = false;
    	zeroGyro = false;
    	droveToDistance = false;
    }

    // Called when another command which requires one or more of the same
    // subsystems is scheduled to run
    protected void interrupted() {
    	end();
    }
}
