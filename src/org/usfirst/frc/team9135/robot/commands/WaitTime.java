package org.usfirst.frc.team9135.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.command.Command;

/**
 *
 */
public class WaitTime extends Command {
	
	private double timeToWait;

	private Timer timer;
	private double currentTimerValue = 0.0;

	private boolean doneWaiting = false;

	private boolean startTimer = false;

    public WaitTime(double timeToWait) {
        // Use requires() here to declare subsystem dependencies
        // eg. requires(chassis);
    	this.timeToWait = timeToWait;

    	timer = new Timer();
    }

    // Called just before this Command runs the first time
    protected void initialize() {
    	doneWaiting = false;
    	startTimer = false;
    }

    // Called repeatedly when this Command is scheduled to run
    protected void execute() {
    	if (startTimer == false) {
    		timer.reset();
    		timer.start();
    		startTimer = true;
    	}
    	currentTimerValue = timer.get();
    	if (currentTimerValue >= timeToWait) {
    		doneWaiting = true;
    	}

    	System.out.println("Timer Wait");

    }

    // Make this return true when this Command no longer needs to run execute()
    protected boolean isFinished() {
    	return doneWaiting;
    }

    // Called once after isFinished returns true
    protected void end() {
    	doneWaiting = false;
    	startTimer = false;
    	timer.stop();
    	timer.reset();
    }

    // Called when another command which requires one or more of the same
    // subsystems is scheduled to run
    protected void interrupted() {
    	end();
    }
}
