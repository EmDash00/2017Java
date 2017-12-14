package org.usfirst.frc.team9135.robot.commands;

import org.usfirst.frc.team9135.robot.CommandBase;
import org.usfirst.frc.team9135.robot.subsystems.Lidars;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.command.Command;

/**
 *
 */
public class GetLidarValueForHopperAndShoot extends Command {
	

	private boolean configureLidar = false;
	private int lidarLowerByte = 0;
	private int lidarUpperByte = 0;
	private double lidarValueIN = 0.0;

	private static final double GET_LIDAR_VALUE_FOR_HOPPER_AND_SHOOT_TIMEOUT = .15;
	

    public GetLidarValueForHopperAndShoot() {
        // Use requires() here to declare subsystem dependencies
        // eg. requires(chassis);
    	requires(CommandBase.lidars);
    	
    }

    // Called just before this Command runs the first time
    protected void initialize() {
    	setTimeout(GET_LIDAR_VALUE_FOR_HOPPER_AND_SHOOT_TIMEOUT);
    }

    // Called repeatedly when this Command is scheduled to run
    protected void execute() {
    	if (configureLidar == false) {
    		CommandBase.lidars.ConfigureLidar();
    		configureLidar = true;
    	}
    	else if (configureLidar) {
    		lidarUpperByte = CommandBase.lidars.GetUpperByte();
    		Timer.delay(.002);
    		lidarLowerByte = CommandBase.lidars.GetLowerByte();
    		lidarValueIN = CommandBase.lidars.GetLidarValue(lidarLowerByte, lidarUpperByte, Lidars.DISTANCE_UNIT_ARRAY[Lidars.INCHES]);
    		configureLidar = false;
    	}

    	if (lidarValueIN > 0.0) {
    		CommandBase.lidars.StoreLidarValueForHopperAndShoot(lidarValueIN);
    	}
    }

    // Make this return true when this Command no longer needs to run execute()
    protected boolean isFinished() {
        return isTimedOut();
    }

    // Called once after isFinished returns true
    protected void end() {
    }

    // Called when another command which requires one or more of the same
    // subsystems is scheduled to run
    protected void interrupted() {
    }
}
