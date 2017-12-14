package org.usfirst.frc.team9135.robot.commands;

import org.usfirst.frc.team9135.robot.CommandBase;

import edu.wpi.first.wpilibj.command.Command;

/**
 *
 */
public class SwitchBetweenHighAndLowShot extends Command {
	
	private boolean closeShot = true;
	private boolean switchedFarAndCloseShotRPM = false;

    public SwitchBetweenHighAndLowShot() {
        // Use requires() here to declare subsystem dependencies
        // eg. requires(chassis);
    	requires(CommandBase.shooter);
    }

    // Called just before this Command runs the first time
    protected void initialize() {
    	switchedFarAndCloseShotRPM = false;
    }

    // Called repeatedly when this Command is scheduled to run
    protected void execute() {
    	if (switchedFarAndCloseShotRPM == false) {
    		closeShot = !closeShot;
    		CommandBase.shooter.SetSwitchFarAndCloseShotShooterRPM(closeShot);
    		switchedFarAndCloseShotRPM = true;
    	}
    }

    // Make this return true when this Command no longer needs to run execute()
    protected boolean isFinished() {
    	return switchedFarAndCloseShotRPM;
    }

    // Called once after isFinished returns true
    protected void end() {
    	switchedFarAndCloseShotRPM = false;
    }

    // Called when another command which requires one or more of the same
    // subsystems is scheduled to run
    protected void interrupted() {
    	end();
    }
}
