package org.usfirst.frc.team9135.robot.commands;

import edu.wpi.first.wpilibj.command.Command;

import org.usfirst.frc.team9135.robot.CommandBase;

/**
 *
 */
public class AutoDriveCollection extends Command {

	private boolean initializeAutoDriveCollection = false;
	private boolean autoDriveCollection = false;
	private static double COLLECTION_MOTOR_POWER = 1.0;
	private boolean doneAutoDrivingCollection = false;
	
    public AutoDriveCollection() {
        // Use requires() here to declare subsystem dependencies
        // eg. requires(chassis);
    	CommandBase.collection.SetAutoDriveCollection(true);
    	initializeAutoDriveCollection = true;
    	doneAutoDrivingCollection = false;
    }

    // Called just before this Command runs the first time
    protected void initialize() {
    }

    // Called repeatedly when this Command is scheduled to run
    protected void execute() {
    }

    // Make this return true when this Command no longer needs to run execute()
    protected boolean isFinished() {
        return false;
    }

    // Called once after isFinished returns true
    protected void end() {
    }

    // Called when another command which requires one or more of the same
    // subsystems is scheduled to run
    protected void interrupted() {
    }
}
