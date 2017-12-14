package org.usfirst.frc.team9135.robot.commands;

import org.usfirst.frc.team9135.robot.CommandBase;
import org.usfirst.frc.team9135.robot.OI;
import org.usfirst.frc.team9135.robot.subsystems.Lidars;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/**
 *
 */
public class ReadLidarValues extends Command {

	private boolean openFrontLidarChannel = false;
	private boolean turnOffDetectorBiasBetweenLidarAcquisitions = false;
	private boolean configuredLidar = false;

	private int lidarUpperByte = 0;
	private int lidarLowerByte = 0;
	private double lidarValueIN = 0;

	private boolean turnLidarOn = false;

	private boolean lidarPowerEnabledButtonPressed = false;
	
    public ReadLidarValues() {
        // Use requires() here to declare subsystem dependencies
        // eg. requires(chassis);
    	requires(CommandBase.lidars);
    }

    // Called just before this Command runs the first time
    protected void initialize() {
    	configuredLidar = false;
    }

    // Called repeatedly when this Command is scheduled to run
    protected void execute() {
    	if (turnLidarOn == false) {
    		CommandBase.lidars.TurnLidarOnOff(Lidars.TURN_LIDAR_ON);
    		turnLidarOn = true;
    	}

    	lidarPowerEnabledButtonPressed = CommandBase.oi.GetButtonPressed(OI.LEFT_DRIVE_JOYSTICK, 8);
    	if (lidarPowerEnabledButtonPressed) {
    		CommandBase.lidars.TurnLidarOnOff(Lidars.TURN_LIDAR_OFF);
    	}
    	else {
    		CommandBase.lidars.TurnLidarOnOff(Lidars.TURN_LIDAR_ON);
    	}

    	if (openFrontLidarChannel == false) {
    		CommandBase.lidars.OpenLidarChannelOnMultiplexer(Lidars.VALUE_TO_OPEN_FRONT_LIDAR_CHANNEL_6);
    		openFrontLidarChannel = true;
    	}

    	/* if (turnOffDetectorBiasBetweenLidarAcquisitions == false) {
    		CommandBase.lidars.TurnOffDetectorBiasBetweenLidarAcquisitions();
    		turnOffDetectorBiasBetweenLidarAcquisitions = true;
    	} */

    	if (configuredLidar == false) {
    		CommandBase.lidars.ConfigureLidar();
    		configuredLidar = true;
    	}
    	else if (configuredLidar) {
    		lidarUpperByte = CommandBase.lidars.GetUpperByte();
    		Timer.delay(.002);
    		lidarLowerByte = CommandBase.lidars.GetLowerByte();
    		lidarValueIN = CommandBase.lidars.GetLidarValue(lidarLowerByte, lidarUpperByte, Lidars.DISTANCE_UNIT_ARRAY[Lidars.INCHES]);
    		configuredLidar = false;
    	}

    	System.out.println("Lidar Value: " + lidarValueIN);

    	SmartDashboard.putNumber("Front Lidar Value IN:", lidarValueIN);
    }

    // Make this return true when this Command no longer needs to run execute()
    protected boolean isFinished() {
        return false;
    }

    // Called once after isFinished returns true
    protected void end() {
    	CommandBase.lidars.TurnLidarOnOff(Lidars.TURN_LIDAR_OFF);
    	turnLidarOn = false;
    	openFrontLidarChannel = false;
    	turnOffDetectorBiasBetweenLidarAcquisitions = false;
    	configuredLidar = false;
    }

    // Called when another command which requires one or more of the same
    // subsystems is scheduled to run
    protected void interrupted() {
    	end();
    }
}
