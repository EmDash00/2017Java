package org.usfirst.frc.team9135.robot.commands;

import org.usfirst.frc.team9135.robot.CommandBase;
import org.usfirst.frc.team9135.robot.subsystems.GearHolder;

import edu.wpi.first.wpilibj.command.Command;


/**
 *
 */

	
public class GearUpAndDown extends Command {
	private static final double GEAR_HOLDER_MOTOR_POWER = .5;

	private boolean minLimitSwitchPressed = false;
	private boolean maxLimitSwitchPressed = false;

	private int gearDirectionCounter = 1;

    public GearUpAndDown() {
        // Use requires() here to declare subsystem dependencies
        // eg. requires(chassis);
    	requires(CommandBase.gearHolder);
    }

    // Called just before this Command runs the first time
    protected void initialize() {
    	gearDirectionCounter = 1;
    	CommandBase.gearHolder.gearUsedNotDefaultly = true;
    }

    // Called repeatedly when this Command is scheduled to run
    protected void execute() {
    	minLimitSwitchPressed = CommandBase.gearHolder.getLimitSwitchValue(GearHolder.LOWER_LIMIT_SWITCH_PORT);
    	maxLimitSwitchPressed = CommandBase.gearHolder.getLimitSwitchValue(GearHolder.UPPER_LIMIT_SWITCH_PORT);

    	if ((gearDirectionCounter % 2) == 1) {
    		if (minLimitSwitchPressed) {
    			gearDirectionCounter++;
    		}
    		else {
    			CommandBase.gearHolder.DriveGearHolderMotor(-GEAR_HOLDER_MOTOR_POWER);
    			System.out.println("Gear Holder Driving Down");
    		}
    	}
    	else if ((gearDirectionCounter % 2) == 0) {
    		if (maxLimitSwitchPressed) {
    			gearDirectionCounter++;
    		}
    		else {
    			CommandBase.gearHolder.DriveGearHolderMotor(GEAR_HOLDER_MOTOR_POWER);
    			System.out.println("Gear Holder Driving Up");
    		}
    	}

    	System.out.println("Gear Direction Counter: " + gearDirectionCounter);
    }

    // Make this return true when this Command no longer needs to run execute()
    protected boolean isFinished() {
        return false;
    }

    // Called once after isFinished returns true
    protected void end() {
    	gearDirectionCounter = 1;
    	CommandBase.gearHolder.gearUsedNotDefaultly = false;
    }

    // Called when another command which requires one or more of the same
    // subsystems is scheduled to run
    protected void interrupted() {
    	end();
    }
}
