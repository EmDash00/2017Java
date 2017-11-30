package org.usfirst.frc.team9135.robot.commands;

import org.usfirst.frc.team9135.robot.CommandBase;

import edu.wpi.first.wpilibj.command.Command;

/**
 *
 */
public class DriveAgitator extends Command {

	private static final double AGITATOR_MOTOR_POWER = 1.0;
	private static final double AGITATOR_2_MOTOR_POWER = 1.0;
	private boolean driveForwards;
	
    public DriveAgitator(boolean driveForwards) {
        // Use requires() here to declare subsystem dependencies
        // eg. requires(chassis);
    	// eg. Requires(Robot::chassis.get());
    	requires(CommandBase.agitator);
    	this.driveForwards = driveForwards;
    }

    // Called just before this Command runs the first time
    protected void initialize() {
    }

    // Called repeatedly when this Command is scheduled to run
    protected void execute() {
    	if (this.driveForwards) {
    		CommandBase.agitator.DriveAgitator(AGITATOR_MOTOR_POWER, AGITATOR_2_MOTOR_POWER);
    	}
    	else if (this.driveForwards == false) {
    		CommandBase.agitator.DriveAgitator(-AGITATOR_MOTOR_POWER, -AGITATOR_2_MOTOR_POWER);
    	}
    }

    // Make this return true when this Command no longer needs to run execute()
    protected boolean isFinished() {
        return false;
    }

    // Called once after isFinished returns true
    protected void end() {
    	CommandBase.agitator.DriveAgitator(0.0, 0.0);
    }

    // Called when another command which requires one or more of the same
    // subsystems is scheduled to run
    protected void interrupted() {
    	end();
    }
}
