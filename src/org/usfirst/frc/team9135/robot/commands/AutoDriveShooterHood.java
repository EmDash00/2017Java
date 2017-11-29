package org.usfirst.frc.team9135.robot.commands;

import org.usfirst.frc.team9135.robot.CommandBase;

import edu.wpi.first.wpilibj.command.Command;

/**
 *
 */
public class AutoDriveShooterHood extends Command {
	private int desiredShooterHoodEncoderValue;
	private boolean shooterHoodAtDesiredEncoderPosition = false;
	
    public AutoDriveShooterHood(int desiredShooterHoodEncoderValue) {
        // Use requires() here to declare subsystem dependencies
        // eg. requires(chassis);
    	requires(CommandBase.shooterHood.get());

    	this.desiredShooterHoodEncoderValue = desiredShooterHoodEncoderValue;
    }

    // Called just before this Command runs the first time
    protected void initialize() {
    	shooterHoodAtDesiredEncoderPosition = false;
        	
    }

    // Called repeatedly when this Command is scheduled to run
    protected void execute() {
    	shooterHoodAtDesiredEncoderPosition = CommandBase.shooterHood.DriveShooterHoodToDesiredEncoderValue(this.desiredShooterHoodEncoderValue);
    }

    // Make this return true when this Command no longer needs to run execute()
    protected boolean isFinished() {
    	return shooterHoodAtDesiredEncoderPosition;
    }

    // Called once after isFinished returns true
    protected void end() {
    	shooterHoodAtDesiredEncoderPosition = false;
    }

    // Called when another command which requires one or more of the same
    // subsystems is scheduled to run
    protected void interrupted() {
    	end();
    }
}
