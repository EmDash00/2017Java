package org.usfirst.frc.team9135.robot.commands;

import org.usfirst.frc.team9135.robot.CommandBase;
import org.usfirst.frc.team9135.robot.subsystems.*;

import edu.wpi.first.wpilibj.command.Command;

/**
 *
 */
public class DriveHoodAndShooterWithLidar extends Command {

	boolean turnOnLidar = false;
	boolean openI2CMultiplexerChannelForLidar = false;
	boolean configureLidar = false;

	int lidarUpperByte = 0;
	int lidarLowerByte = 0;
	double lidarValueIN = 0.0;

	boolean startGettingDesiredHoodEncoderValues = false;

	int desiredHoodEncoderValue = 0;

	boolean configurePID = false;
	boolean configureVoltageMode = false;
	boolean selectedPIDSlot = false;

	boolean intializeDesiredHoodEncoderPosition = false;
	boolean hoodAtDesiredEncoderPosition = false;
	
    public DriveHoodAndShooterWithLidar() {
        // Use requires() here to declare subsystem dependencies
        // eg. requires(chassis);
    	requires(CommandBase.lidars);
    	requires(CommandBase.shooter);
    	requires(CommandBase.shooterHood);
    }

    // Called just before this Command runs the first time
    protected void initialize() {
    	CommandBase.shooter.ConfigureShooterPID();
    	configurePID = true;
    	configureVoltageMode = false;

    	CommandBase.shooter.SelectPIDProfileSlot(Shooter.FAR_SHOT_PID_VALUES);
    	selectedPIDSlot = true;
    }

    // Called repeatedly when this Command is scheduled to run
    protected void execute() {
    	if (configurePID == false) {
    		CommandBase.shooter.ConfigureShooterPID();
    		configurePID = true;
    		configureVoltageMode = false;
    	}

    	if (selectedPIDSlot == false) {
    		CommandBase.shooter.SelectPIDProfileSlot(Shooter.FAR_SHOT_PID_VALUES);
    		selectedPIDSlot = true;
    	}

    	CommandBase.shooter.DriveShooterMotor(Shooter.SHOOTER_SETPOINT_RPM_FAR_SHOT);

    	if (turnOnLidar == false) {
    		CommandBase.lidars.TurnLidarOnOff(Lidars.TURN_LIDAR_ON);
    		turnOnLidar = true;
    	}
    	else if (configureLidar == false) {
    		CommandBase.lidars.ConfigureLidar();
    		configureLidar = true;
    	}
    	else if (configureLidar) {
    		lidarUpperByte = CommandBase.lidars.GetUpperByte();
    		wait((long).002);
    		lidarLowerByte = CommandBase.lidars.GetLowerByte();
    		lidarValueIN = CommandBase.lidars.GetLidarValue(lidarLowerByte, lidarUpperByte, Lidars.DISTANCE_UNIT_ARRAY[Lidars.INCHES]);
    		configureLidar = false;
    	}

    	if (openI2CMultiplexerChannelForLidar == false) {
    		CommandBase.lidars.OpenLidarChannelOnMultiplexer(Lidars.VALUE_TO_OPEN_FRONT_LIDAR_CHANNEL_6);
    		openI2CMultiplexerChannelForLidar = true;
    	}

    	if (startGettingDesiredHoodEncoderValues == false) {
    		if (lidarValueIN > 0.0) {
    			startGettingDesiredHoodEncoderValues= true;
    		}
    	}
    	else if (startGettingDesiredHoodEncoderValues) {
    		if (intializeDesiredHoodEncoderPosition) {
    			desiredHoodEncoderValue = CommandBase.shooterHood.GetDesiredHoodEncoderPositionGivenLidarValue(lidarValueIN);
    		}

    		if (hoodAtDesiredEncoderPosition == false) {
    			hoodAtDesiredEncoderPosition= CommandBase.shooterHood.DriveShooterHoodToDesiredEncoderValue(desiredHoodEncoderValue);
    		}
    		else if (hoodAtDesiredEncoderPosition) {
    			CommandBase.shooterHood.DriveShooterHoodMotor(0.0);
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
    	configureVoltageMode = true;
    	configurePID = false;
    	CommandBase.shooter.DriveShooterMotor(0.0);
    }

    // Called when another command which requires one or more of the same
    // subsystems is scheduled to run
    protected void interrupted() {
    }
}
