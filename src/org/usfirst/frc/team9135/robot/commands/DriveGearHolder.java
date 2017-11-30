package org.usfirst.frc.team9135.robot.commands;

import org.usfirst.frc.team9135.robot.CommandBase;
import org.usfirst.frc.team9135.robot.OI;
import org.usfirst.frc.team9135.robot.subsystems.GearHolder;

import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/**
 *
 */
public class DriveGearHolder extends Command {

	private static final double GEAR_HOLDER_MOTOR_POWER = .5;

	private double gearHolderServoValue = 0.0;

	private boolean gearHolderUpwardsButtonPressed = false;
	private boolean gearHolderDownwardsButtonPressed = false;

	private boolean photoElectricSensorValue = false;

	private boolean upperLimitSwitchPressed = false;
	private boolean lowerLimitSwitchPressed = false;
	
    public DriveGearHolder() {
        // Use requires() here to declare subsystem dependencies
        // eg. requires(chassis);
    	requires(CommandBase.gearHolder);
    }

    // Called just before this Command runs the first time
    protected void initialize() {
    }

    // Called repeatedly when this Command is scheduled to run
    protected void execute() {
    	gearHolderUpwardsButtonPressed = CommandBase.oi.GetButtonPressed(OI.MANIPULATOR_JOYSTICK, OI.GEAR_HOLDER_UPWARDS_BUTTON);
    	gearHolderDownwardsButtonPressed = CommandBase.oi.GetButtonPressed(OI.MANIPULATOR_JOYSTICK, OI.GEAR_HOLDER_DOWNWARDS_BUTTON);

    	if (gearHolderUpwardsButtonPressed) {
    		CommandBase.gearHolder.DriveGearHolderMotor(GEAR_HOLDER_MOTOR_POWER);
    	}
    	else if (gearHolderDownwardsButtonPressed) {
    		CommandBase.gearHolder.DriveGearHolderMotor(-GEAR_HOLDER_MOTOR_POWER);
    	}
    	else {
    		CommandBase.gearHolder.DriveGearHolderMotor(0.0);
    	}

    	if (CommandBase.oi.POVDirectionPressed(OI.MANIPULATOR_JOYSTICK, OI.TOP_POV)) {
    		CommandBase.gearHolder.setGearHolderServoValue(GearHolder.SERVO_IN_POSITION);
    	}
    	else if (CommandBase.oi.POVDirectionPressed(OI.MANIPULATOR_JOYSTICK, OI.BOTTOM_POV)) {
    		CommandBase.gearHolder.setGearHolderServoValue(GearHolder.SERVO_OUT_POSITION);
    	}

    	photoElectricSensorValue = CommandBase.gearHolder.getPhotoElectricSensorValue();
    	SmartDashboard.putBoolean("Gear In Gear Holder", photoElectricSensorValue);

    	upperLimitSwitchPressed = CommandBase.gearHolder.getLimitSwitchValue(GearHolder.UPPER_LIMIT_SWITCH_PORT);
    	lowerLimitSwitchPressed = CommandBase.gearHolder.getLimitSwitchValue(GearHolder.LOWER_LIMIT_SWITCH_PORT);
    	SmartDashboard.putBoolean("Gear Upper Limit Switch Pressed", upperLimitSwitchPressed);
    	SmartDashboard.putBoolean("Gear Lower Limit Switch Pressed", lowerLimitSwitchPressed);

    }

    // Make this return true when this Command no longer needs to run execute()
    protected boolean isFinished() {
        return false;
    }

    // Called once after isFinished returns true
    protected void end() {
    	CommandBase.gearHolder.DriveGearHolderMotor(0.0);
    }

    // Called when another command which requires one or more of the same
    // subsystems is scheduled to run
    protected void interrupted() {
    	end();
    }
}
