package org.usfirst.frc.team9135.robot.commands;

import org.usfirst.frc.team9135.robot.CommandBase;
import org.usfirst.frc.team9135.robot.subsystems.*;

import edu.wpi.first.wpilibj.command.Command;

/**
 *
 */
public class AutoMoveGearHolder extends Command {
	private boolean moveGearHolderUpwards;

	private static final double GEAR_HOLDER_MOTOR_POWER = .9;

	private boolean minLimitSwitchPressed = false;
	private boolean maxLimitSwitchPressed = false;

	private boolean movedGearHolderToDesiredLimitSwitch = false;

	static final double AUTO_MOVE_GEAR_HOLDER_TIMEOUT = 2.3;
	
	
    public AutoMoveGearHolder() 
    {
        // Use requires() here to declare subsystem dependencies
        // eg. requires(chassis);
    	requires(CommandBase.gearHolder);

    	this.moveGearHolderUpwards = moveGearHolderUpwards;
    
    }

    // Called just before this Command runs the first time
    protected void initialize() {
    	setTimeout(AUTO_MOVE_GEAR_HOLDER_TIMEOUT);
    	movedGearHolderToDesiredLimitSwitch = false;
    }

    // Called repeatedly when this Command is scheduled to run
    protected void execute() {
    	if (this.moveGearHolderUpwards) {
    		maxLimitSwitchPressed = CommandBase.gearHolder.getLimitSwitchValue(GearHolder.UPPER_LIMIT_SWITCH_PORT);
    		if (maxLimitSwitchPressed) {
    			CommandBase.gearHolder.DriveGearHolderMotor(0.0);
    			movedGearHolderToDesiredLimitSwitch = true;
    		}
    		else {
    			CommandBase.gearHolder.DriveGearHolderMotor(GEAR_HOLDER_MOTOR_POWER);
    			movedGearHolderToDesiredLimitSwitch = false;
    		}
    	}
    	else if (this.moveGearHolderUpwards == false) {
    		minLimitSwitchPressed = CommandBase.gearHolder.getLimitSwitchValue(GearHolder.LOWER_LIMIT_SWITCH_PORT);
    		if (minLimitSwitchPressed) {
    			CommandBase.gearHolder.DriveGearHolderMotor(0.0);
    			movedGearHolderToDesiredLimitSwitch = true;
    		}
    		else {
    			CommandBase.gearHolder.DriveGearHolderMotor(-GEAR_HOLDER_MOTOR_POWER);
    			movedGearHolderToDesiredLimitSwitch = false;
    		}
    	}
    }

    // Make this return true when this Command no longer needs to run execute()
    protected boolean isFinished() {
    	return (movedGearHolderToDesiredLimitSwitch || isTimedOut());
    }

    // Called once after isFinished returns true
    protected void end() 
    {
    	movedGearHolderToDesiredLimitSwitch = false;
    }

    // Called when another command which requires one or more of the same
    // subsystems is scheduled to run
    protected void interrupted() {
    	end();
    }
}
