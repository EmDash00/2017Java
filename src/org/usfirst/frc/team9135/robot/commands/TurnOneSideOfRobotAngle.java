package org.usfirst.frc.team9135.robot.commands;

import org.usfirst.frc.team9135.robot.CommandBase;

import edu.wpi.first.wpilibj.command.Command;

/**
 *
 */
public class TurnOneSideOfRobotAngle extends Command {

	
	private double desiredAngle;
	private boolean driveRightSide;
	private double motorPower;

	private boolean zeroGyro = false;
	private double currentGyroAngle = 0.0;
	private boolean turnedDesiredAngle = false;
	
    public TurnOneSideOfRobotAngle(double desiredAngle, boolean driveRightSide, double motorPower) {
        // Use requires() here to declare subsystem dependencies
        // eg. requires(chassis);
    	requires(CommandBase.driveTrain);

    	this.desiredAngle = desiredAngle;
    	this.driveRightSide = driveRightSide;
    	this.motorPower = motorPower;
    }

    // Called just before this Command runs the first time
    protected void initialize() {
    	CommandBase.driveTrain.ZeroGyroAngle();
    	zeroGyro = true;
    	turnedDesiredAngle = false;

    	CommandBase.driveTrain.is_aiming = true;
    }

    // Called repeatedly when this Command is scheduled to run
    protected void execute() {
    	if (zeroGyro == false) {
    		CommandBase.driveTrain.ZeroGyroAngle();
    		zeroGyro = true;
    	}

    	currentGyroAngle = (Math.abs(CommandBase.driveTrain.getGyroAngle()));
    	if (this.driveRightSide) {
    		if (currentGyroAngle >= this.desiredAngle) {
    			CommandBase.driveTrain.DriveTank(0.0, 0.0);
    			turnedDesiredAngle = true;
    		}
    		else {
    			CommandBase.driveTrain.DriveTank(0.0, this.motorPower);
    		}
    	}
    	else if (this.driveRightSide == false) {
    		if (currentGyroAngle >= this.desiredAngle) {
    			CommandBase.driveTrain.DriveTank(0.0, 0.0);
    			turnedDesiredAngle = true;
    		}
    		else {
    			CommandBase.driveTrain.DriveTank(this.motorPower, 0.0);
    		}
    	}
    }

    // Make this return true when this Command no longer needs to run execute()
    protected boolean isFinished() {
    	return turnedDesiredAngle;
    }

    // Called once after isFinished returns true
    protected void end() {
    	zeroGyro = false;
    	turnedDesiredAngle = false;

    	CommandBase.driveTrain.is_aiming = false;
    }

    // Called when another command which requires one or more of the same
    // subsystems is scheduled to run
    protected void interrupted() {
    	end();
    }
}
