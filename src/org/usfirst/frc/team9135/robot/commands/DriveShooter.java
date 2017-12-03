package org.usfirst.frc.team9135.robot.commands;

import org.usfirst.frc.team9135.robot.CommandBase;
import org.usfirst.frc.team9135.robot.OI;
import org.usfirst.frc.team9135.robot.RobotMap;
import org.usfirst.frc.team9135.robot.RobotMap.ShooterPIDSelection;
import org.usfirst.frc.team9135.robot.subsystems.*;


import edu.wpi.first.wpilibj.Preferences;
import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/**
 *
 */
public class DriveShooter extends Command {
	RobotMap.ShooterPIDSelection shooterMode;

	private double setpointRPM = 0.0;
	private double desiredShooterVoltage = 0.0;

	private double chosenSetpoint = 0.0;
	private double chosenVoltage = 0.0;

	private int shooterMotorRPM = 0;
	private int shooterMotorNUPer100Ms = 0;

	private boolean initializePID = false;
	private boolean initializeVoltageMode = false;

	private boolean shooterForwardsButtonPressed = false;
	private boolean shooterBackwardsButtonPressed = false;

	private static final double MAX_PERCENT_OF_SETPOINT_TO_RAMP_VOLTAGE = .65;

	private double shooterOutputCurrent = 0.0;

	private boolean throttleUp = false;

	private double shooterVoltage = 0.0;

	private double throttleValue = 0.0;

	private boolean closeShotMode = false;

	private boolean initializedCloseShotPIDSlot = false;
	private boolean initializedFarShotPIDSlot = false;

    public DriveShooter() {
        // Use requires() here to declare subsystem dependencies
        // eg. requires(chassis);
    	requires(CommandBase.shooter);

    	this.shooterMode = shooterMode;
    }

    // Called just before this Command runs the first time
    protected void initialize() {
    	setpointRPM = Preferences.getInstance().getDouble("Shooter PID Setpoint", 2400.0);
    	desiredShooterVoltage = Preferences.getInstance().getDouble("Shooter Voltage", 8.0);
    	CommandBase.shooter.ConfigureShooterVoltageMode();
    	initializeVoltageMode = true;
    	initializePID = false;

    	initializedCloseShotPIDSlot = false;
    	initializedFarShotPIDSlot = false;
    }

    // Called repeatedly when this Command is scheduled to run
    protected void execute() {
    	closeShotMode = CommandBase.shooter.GetSwitchBetweenFarAndCloseShotShooterRPM();

    	if (closeShotMode) {
    		if (initializedCloseShotPIDSlot == false) {
    			CommandBase.shooter.SelectPIDProfileSlot(Shooter.CLOSE_SHOT_PID_VALUES);
    			initializedCloseShotPIDSlot = true;
    			initializedFarShotPIDSlot = false;
    		}
    		throttleValue = CommandBase.oi.GetThrottleValue(OI.MANIPULATOR_JOYSTICK);
    		chosenSetpoint = CommandBase.shooter.GetCloseShotShooterRPMGivenThrottleValue(throttleValue, Shooter.SHOOTER_RANGE_OF_RPM_PART_1, Shooter.SHOOTER_SETPOINT_MINIMUM_PART_1);
    		//chosenSetpoint = 2850.0;
    		chosenVoltage = Shooter.DESIRED_VOLTAGE_CLOSE_SHOT;
    	}
    	else if (closeShotMode == false) {
    		if (initializedFarShotPIDSlot == false) {
    			CommandBase.shooter.SelectPIDProfileSlot(Shooter.FAR_SHOT_PID_VALUES);
    			initializedFarShotPIDSlot = true;
    			initializedCloseShotPIDSlot = false;
    		}
    		throttleValue = CommandBase.oi.GetThrottleValue(OI.MANIPULATOR_JOYSTICK);
    		chosenSetpoint = CommandBase.shooter.GetCloseShotShooterRPMGivenThrottleValue(throttleValue, Shooter.SHOOTER_RANGE_OF_RPM_PART_2, Shooter.SHOOTER_SETPOINT_MINIMUM_PART_2);
    		//chosenSetpoint = 3150;
    		chosenVoltage = Shooter.DESIRED_VOLTAGE_FAR_SHOT;
    	}

    	SmartDashboard.putNumber("Desired Shooter RPM", chosenSetpoint);

    	shooterMotorRPM = CommandBase.shooter.GetShooterWheelRPM();
    	//shooterOutputCurrent = CommandBase.shooter.GetShooterMotorOutputCurrent();

    	SmartDashboard.putNumber("Shooter Motor RPM", shooterMotorRPM);
    	//SmartDashboard.PutNumber("Shooter Output Current", shooterOutputCurrent);

    	shooterForwardsButtonPressed = CommandBase.oi.GetButtonPressed(OI.MANIPULATOR_JOYSTICK, OI.TRIGGER_BUTTON);
    	shooterBackwardsButtonPressed = CommandBase.oi.GetButtonPressed(OI.MANIPULATOR_JOYSTICK, OI.THUMB_BUTTON);

    	if (this.shooterMode == ShooterPIDSelection.PID_CompetitionBot) {
    		if (shooterForwardsButtonPressed) {
    			if (initializePID == false) {
    				CommandBase.shooter.ConfigureShooterPID();
    				initializeVoltageMode = false;
    				initializePID = true;
    			}
    			CommandBase.shooter.DriveShooterMotor(chosenSetpoint);
    		}
    		else if (shooterBackwardsButtonPressed) {
    			if (initializeVoltageMode == false) {
    				CommandBase.shooter.ConfigureShooterVoltageMode();
    				initializePID = false;
    				initializeVoltageMode = true;
    			}
    			CommandBase.shooter.DriveShooterMotor(chosenVoltage);
    		}
    		else {
    			if (initializeVoltageMode == false) {
    				CommandBase.shooter.ConfigureShooterVoltageMode();
    				initializePID = false;
    				initializeVoltageMode = true;
    			}
    			CommandBase.shooter.DriveShooterMotor(0.0);
    		}
    	}
    	else if (this.shooterMode == ShooterPIDSelection.PID_Ramp_Up) {
    		if (shooterForwardsButtonPressed) {
    			if (shooterMotorRPM < (.45 * chosenSetpoint)) {
    				if (initializeVoltageMode == false) {
    					CommandBase.shooter.ConfigureShooterVoltageMode();
    					initializePID = false;
    					initializeVoltageMode = true;
    				}
    				CommandBase.shooter.DriveShooterMotor(.7 * chosenVoltage);
    			}
    			else if (shooterMotorRPM < (.7 * chosenSetpoint)) {
    				if (initializeVoltageMode == false) {
    					CommandBase.shooter.ConfigureShooterVoltageMode();
    					initializePID = false;
    					initializeVoltageMode = true;
    				}
    				CommandBase.shooter.DriveShooterMotor(chosenVoltage);
    			}
    			else {
    				if (initializePID == false) {
    					CommandBase.shooter.ConfigureShooterPID();
    					initializeVoltageMode = false;
    					initializePID = true;
    				}
    				CommandBase.shooter.DriveShooterMotor(chosenSetpoint);
    			}
    		}
    		else if (shooterBackwardsButtonPressed) {
    			if (initializeVoltageMode == false) {
    				CommandBase.shooter.ConfigureShooterVoltageMode();
    				initializePID = false;
    				initializeVoltageMode = true;
    			}
    			CommandBase.shooter.DriveShooterMotor(-chosenVoltage);
    		}
    		else {
    			if (initializeVoltageMode == false) {
    				CommandBase.shooter.ConfigureShooterVoltageMode();
    				initializePID = false;
    				initializeVoltageMode = true;
    			}
    			CommandBase.shooter.DriveShooterMotor(0.0);
    		}
    	}
    	else if (this.shooterMode == ShooterPIDSelection.Voltage) {
    		if (initializeVoltageMode == false) {
    			CommandBase.shooter.ConfigureShooterVoltageMode();
    			initializePID = false;
    			initializeVoltageMode = true;
    		}

    		if (shooterForwardsButtonPressed) {
    			CommandBase.shooter.DriveShooterMotor(chosenVoltage);
    		}
    		else if (shooterBackwardsButtonPressed) {
    			CommandBase.shooter.DriveShooterMotor(-chosenVoltage);
    		}
    		else {
    			CommandBase.shooter.DriveShooterMotor(0.0);
    		}
    	}
    }

    // Make this return true when this Command no longer needs to run execute()
    protected boolean isFinished() {
        return false;
    }

    // Called once after isFinished returns true
    protected void end() {
    	CommandBase.shooter.ConfigureShooterVoltageMode();
    	CommandBase.shooter.DriveShooterMotor(0.0);
    	initializeVoltageMode = false;
    	initializePID = false;

    	initializedCloseShotPIDSlot = false;
    	initializedFarShotPIDSlot = false;
    }

    // Called when another command which requires one or more of the same
    // subsystems is scheduled to run
    protected void interrupted() {
    	end();
    }
}
