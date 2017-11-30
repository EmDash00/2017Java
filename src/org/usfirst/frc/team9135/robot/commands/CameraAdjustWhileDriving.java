package org.usfirst.frc.team9135.robot.commands;

import org.usfirst.frc.team9135.robot.CommandBase;
import org.usfirst.frc.team9135.robot.OI;

import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/**
 *
 */
public class CameraAdjustWhileDriving extends Command {
	public double ultrasonicValue = 135.0;
	public static final double DISTANCE_TO_STOP = 24;
	public double voltageLeft = 0.0;
	public double voltageRight = 0.0;
	public double tableOffset = 0.0;
	public double actualOffset = 0.0;
	public static final double driveSpeed = -.25;
	public double curveValue = 0.0;
	public static final double proportionalConstant = .01;//.01;
	public boolean done = false;
	
    public CameraAdjustWhileDriving() {
        // Use requires() here to declare subsystem dependencies
        // eg. requires(chassis);
    	requires(CommandBase.driveTrain);
    	requires(CommandBase.ultrasonicSensor);
    }

    // Called just before this Command runs the first time
    protected void initialize() {
    	done = false;
    	CommandBase.driveTrain.is_aiming = true;
    }

    // Called repeatedly when this Command is scheduled to run
    protected void execute() {
    	ultrasonicValue = CommandBase.ultrasonicSensor.GetGearUltrasonicSensorValueInches();

    	if(ultrasonicValue > DISTANCE_TO_STOP)
    	{
    		tableOffset = GetTableOffset(ultrasonicValue);
    		actualOffset = CommandBase.server.get_angle(1); //actually returns an offset
    		//if(tableOffset == 0) done = true;
    		//Curve < 0 will turn left
    		System.out.println("offset value: " + actualOffset + " table offset: " + tableOffset);
    		curveValue = ((actualOffset - tableOffset) * proportionalConstant);
    		SmartDashboard.putNumber("curve Value", curveValue);
    		System.out.println("Curve Value: " + curveValue);
    		if(curveValue < -1) curveValue = -1;
    		else if(curveValue > 1) curveValue = 1;
    		CommandBase.driveTrain.chassis.drive(driveSpeed, curveValue);
    	}
    	else
    	{
    		done = true;
    	}
    }

    double GetTableOffset(double ultrasonicInput)
    {
    	if(ultrasonicInput >= 50)
    		return -93.5;
    	else if(ultrasonicInput == 46.5)
    		return -103.5;
    	else if(ultrasonicInput >= 43)
    		return -110.5;
    	else if(ultrasonicInput >= 39)
    		return -120.5;
    	else if(ultrasonicInput >= 34)
    		return -149.5;
    	else if(ultrasonicInput >= 29)
    		return -167.5;
    	else if(ultrasonicInput >= 24)
    		return -202.5;
    	else
    		return 0;
    }
    // Make this return true when this Command no longer needs to run execute()
    protected boolean isFinished() {
    	return done || (CommandBase.oi.GetAction(CommandBase.oi.LEFT_DRIVE_JOYSTICK, 10));
    }

    // Called once after isFinished returns true
    protected void end() {
    	CommandBase.driveTrain.is_aiming = false;
    }

    // Called when another command which requires one or more of the same
    // subsystems is scheduled to run
    protected void interrupted() {
    }
}
