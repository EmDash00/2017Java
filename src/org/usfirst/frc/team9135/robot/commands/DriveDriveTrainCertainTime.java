package org.usfirst.frc.team9135.robot.commands;

import org.usfirst.frc.team9135.robot.CommandBase;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.command.Command;

/**
 *
 */
public class DriveDriveTrainCertainTime extends Command {

	private double desiredTimerValue;
	private double motorPower;

	private Timer timer;
	private double currentTime = 0.0;

	private boolean restartTimer = false;

	private boolean doneDrivingWithTimer = false;
	
    private DriveDriveTrainCertainTime(double desiredTimerValue, double motorPower) {
        // Use requires() here to declare subsystem dependencies
        // eg. requires(chassis);
    	requires(CommandBase.driveTrain);

    	this.desiredTimerValue = desiredTimerValue;
    	this.motorPower = motorPower;

    	timer = new Timer();
    }

    // Called just before this Command runs the first time
    protected void initialize() {
    	timer.reset();
    	timer.start();
    	restartTimer = true;
    	doneDrivingWithTimer = false;
    }

    // Called repeatedly when this Command is scheduled to run
    protected void execute() {
    	if (restartTimer == false) {
    		timer.reset();
    		timer.start();
    		restartTimer = true;
    	}

    	currentTime = timer.get();

    	if (doneDrivingWithTimer == false) {
    		if (currentTime >= this.desiredTimerValue) {
    			CommandBase.driveTrain.DriveTank(0.0, 0.0);
    			timer.stop();
    			timer.reset();
    			doneDrivingWithTimer = true;
    		}
    		else {
    			CommandBase.driveTrain.DriveTank(this.motorPower, this.motorPower);
    			doneDrivingWithTimer = false;
    		}
    	}
    }

    // Make this return true when this Command no longer needs to run execute()
    protected boolean isFinished() {
    	return doneDrivingWithTimer;
    }

    // Called once after isFinished returns true
    protected void end() {
    	CommandBase.driveTrain.DriveTank(0.0, 0.0);
    	timer.stop();
    	timer.reset();
    	restartTimer = false;
    	doneDrivingWithTimer = false;
    }

    // Called when another command which requires one or more of the same
    // subsystems is scheduled to run
    protected void interrupted() {
    	end();
    }
}
