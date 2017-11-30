package org.usfirst.frc.team9135.robot.commands;

import org.usfirst.frc.team9135.robot.CommandBase;

import edu.wpi.first.wpilibj.command.Command;

/**
 *
 */
public class DriveCollection extends Command {
	
	private boolean driveForwards;
	private static final double COLLECTION_MOTOR_POWER = 1.0;
    
	public DriveCollection(boolean driveForwards) {
        // Use requires() here to declare subsystem dependencies
        // eg. requires(chassis);
		requires(CommandBase.collection);
		this.driveForwards = driveForwards;
    }

    // Called just before this Command runs the first time
    protected void initialize() {
    	
    }

    // Called repeatedly when this Command is scheduled to run
    protected void execute() {
    	if (this.driveForwards) {
    		CommandBase.collection.DriveCollection(COLLECTION_MOTOR_POWER);
    	}
    	else if (this.driveForwards == false) {
    		CommandBase.collection.DriveCollection(-COLLECTION_MOTOR_POWER);
    	}
    }

    // Make this return true when this Command no longer needs to run execute()
    protected boolean isFinished() {
        return false;
    }

    // Called once after isFinished returns true
    protected void end() {
    	CommandBase.collection.DriveCollection(0.0);
    }

    // Called when another command which requires one or more of the same
    // subsystems is scheduled to run
    protected void interrupted() {
    	end();
    }
}
