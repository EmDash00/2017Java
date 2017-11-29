package org.usfirst.frc.team9135.robot.commands;

import org.usfirst.frc.team9135.robot.CommandBase;

import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.*;

/**
 *
 */
public class AimBotWithUltrasonicSenors extends Command 
{
	double leftUltrasonicSensorValue = 0.0;
	double rightUltrasonicSensorValue = 0.0;

	double desiredAngleToTurnDriveTrain = 0.0;
	Boolean sonBool;
	Timer timer;
	double timerValue = 0.0;
	boolean initializeAimBotWithUltrasonicSensors = false;
	static double MAX_TIME_FOR_EXECUTING_DRIVE_TRAIN_PID = .5;
	

    public AimBotWithUltrasonicSenors(Boolean sonarGood) {
    	// Use Requires() here to declare subsystem dependencies
    	Requires(CommandBase.ultrasonicSensor.get());
    	Requires(CommandBase.driveTrain.get());
    	sonBool = sonarGood;
    	timer = new Timer();
        // Use requires() here to declare subsystem dependencies
        // eg. requires(chassis);
    }

    // Called just before this Command runs the first time
    protected void initialize() {
    	leftUltrasonicSensorValue = CommandBase.ultrasonicSensor.GetGearUltrasonicSensorValueInches();
    	rightUltrasonicSensorValue = CommandBase.ultrasonicSensor.GetGearUltrasonicSensorValueInches();
    	if(leftUltrasonicSensorValue > 100.0) {
    		sonBool = false;
    		CommandBase.ultrasonicSensor.usingRightUltrasonicSensorForGearCamera = true;
    	}
    	else if(rightUltrasonicSensorValue > 100.0){
    		sonBool = false;
    		CommandBase.ultrasonicSensor.usingRightUltrasonicSensorForGearCamera = false;
    	}
    	else
    	{
    		sonBool = true;
    	}
    	//desiredAngleToTurnDriveTrain = ultrasonicSensor.GetAngleToTurnForGear(rightUltrasonicSensorValue, leftUltrasonicSensorValue);

    	CommandBase.driveTrain.TurnPIDEnable(desiredAngleToTurnDriveTrain);

    	CommandBase.driveTrain.is_aiming = true;

    	timer.reset();
    	timer.start();
    	initializeAimBotWithUltrasonicSensors = true;
    }

    // Called repeatedly when this Command is scheduled to run
    protected void execute() {
    	if (initializeAimBotWithUltrasonicSensors == false) {
    		leftUltrasonicSensorValue = CommandBase.ultrasonicSensor.GetGearUltrasonicSensorValueInches();
    		rightUltrasonicSensorValue = CommandBase.ultrasonicSensor.GetGearUltrasonicSensorValueInches();

    		//desiredAngleToTurnDriveTrain = ultrasonicSensor.GetAngleToTurnForGear(rightUltrasonicSensorValue, leftUltrasonicSensorValue);

    		CommandBase.driveTrain.TurnPIDEnable(desiredAngleToTurnDriveTrain);

    		CommandBase.driveTrain.is_aiming = true;

    		timer.reset();
    		timer.start();
    		initializeAimBotWithUltrasonicSensors = true;
    	}
    	timerValue = timer.get();
    	CommandBase.driveTrain.PIDTurning();

    	SmartDashboard.putNumber("Desired Angle To Turn With Ultrasonic Sensors", desiredAngleToTurnDriveTrain);
    }


    // Make this return true when this Command no longer needs to run execute()
    protected boolean isFinished() {
    	return (timerValue >= MAX_TIME_FOR_EXECUTING_DRIVE_TRAIN_PID) || (!CommandBase.driveTrain.turnController.IsEnabled()) || sonBool == false;
    }

    // Called once after isFinished returns true
    protected void end() {
    	CommandBase.driveTrain.is_aiming = false;
    	initializeAimBotWithUltrasonicSensors = false;
    	timer.stop();
    	timer.reset();
    }

    // Called when another command which requires one or more of the same
    // subsystems is scheduled to run
    protected void interrupted() {
    	end();
    }
}
