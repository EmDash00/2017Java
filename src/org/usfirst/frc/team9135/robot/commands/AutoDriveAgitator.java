package org.usfirst.frc.team9135.robot.commands;

import org.usfirst.frc.team9135.robot.RobotMap;
import org.usfirst.frc.team9135.robot.CommandBase;

import edu.wpi.first.wpilibj.command.Command;

import edu.wpi.first.wpilibj.Timer;
	
/**
 *
 */
public class AutoDriveAgitator extends Command {
	private double timeToDriveAgitator;

	private boolean shooterUpToSpeed = false;
	private boolean turnOffCollection = false;

	private static double AGITATOR_MOTOR_POWER = 1.0;
	private static double AGITATOR_2_MOTOR_POWER = 1.0;

	private Timer timer;
	double timerValue = 0.0;

	private boolean startTimer = false;
	private boolean doneDrivingAgitator = false;

	private boolean initializeContinueDrivingShooter = false;
	
	
    public AutoDriveAgitator() {
    	
        // Use requires() here to declare subsystem dependencies
        // eg. requires(chassis);
    }

    // Called just before this Command runs the first time
    protected void initialize() {
    	setTimeout((this.timeToDriveAgitator + .5));
    	shooterUpToSpeed = false;
    	doneDrivingAgitator = false;

    	timer.reset();
    	timer.start();
    	startTimer = false;

    	turnOffCollection = false;

    	CommandBase.shooter.stopShooterFromDriving = false;
    	initializeContinueDrivingShooter = true;
    }

    // Called repeatedly when this Command is scheduled to run
    protected void execute() {
    	if (initializeContinueDrivingShooter == false) {
    		CommandBase.shooter.stopShooterFromDriving = false;
    		initializeContinueDrivingShooter = true;
    	}

    	if (turnOffCollection == false) {
    		CommandBase.collection.SetAutoDriveCollection(false);
    		turnOffCollection = true;
    	}

    	timerValue = timer.get();

    	shooterUpToSpeed = CommandBase.shooter.getShooterUpToSpeed();

    	if (shooterUpToSpeed == false) {
    		CommandBase.agitator.DriveAgitator(0.0, 0.0);
    	}
    	else if (shooterUpToSpeed) {
    		if (startTimer == false) {
    			timer.stop();
    			timer.reset();
    			timer.start();
    			CommandBase.agitator.DriveAgitator(AGITATOR_MOTOR_POWER, AGITATOR_2_MOTOR_POWER);
    			startTimer = true;
    		}

    		timerValue = timer.get();

    		if (startTimer && timerValue < this.timeToDriveAgitator) {
    			CommandBase.agitator.DriveAgitator(AGITATOR_MOTOR_POWER, AGITATOR_2_MOTOR_POWER);
    		}
    		else if (startTimer && timerValue >= this.timeToDriveAgitator) {
    			CommandBase.agitator.DriveAgitator(0.0, 0.0);
    			CommandBase.shooter.stopShooterFromDriving = true;
    			shooterUpToSpeed = false;
    			startTimer = false;
    			doneDrivingAgitator = true;
    		}
    	}

    }

    // Make this return true when this Command no longer needs to run execute()
    protected boolean isFinished() {
    	return (doneDrivingAgitator || isTimedOut());
    }

    // Called once after isFinished returns true
    protected void end() {
    	CommandBase.agitator.DriveAgitator(0.0, 0.0);
    	CommandBase.collection.DriveCollection(0.0);
    	initializeContinueDrivingShooter = false;
    	CommandBase.shooter.stopShooterFromDriving = true;
    	shooterUpToSpeed = false;
    	turnOffCollection = false;
    	startTimer = false;
    	doneDrivingAgitator = false;
    	timer.stop();
    	timer.reset();
    }

    // Called when another command which requires one or more of the same
    // subsystems is scheduled to run
    protected void interrupted() {
    	end();
    }
}
