package org.usfirst.frc.team9135.robot.commands;

import org.usfirst.frc.team9135.robot.CommandBase;

import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/**
 *
 */
public class ReadUltrasonicSensorValue extends Command {

	private double leftUltrasonicSensorValueInches = 0.0;
	private double rightUltrasonicSensorValueInches = 0.0;

	private int readingUltrasonicSensorsCounter = 1;

	private double gearUltrasonicSensorValue = 0.0;
	
    public ReadUltrasonicSensorValue() {
        // Use requires() here to declare subsystem dependencies
        // eg. requires(chassis);
    	Requires(CommandBase.ultrasonicSensor;
    }

    // Called just before this Command runs the first time
    protected void initialize() {
    	readingUltrasonicSensorsCounter = 1;
    }

    // Called repeatedly when this Command is scheduled to run
    protected void execute() {
    	gearUltrasonicSensorValue = CommandBase.ultrasonicSensor.GetGearUltrasonicSensorValueInches();
    	SmartDashboard.putNumber("Gear Ultrasonic Sensor Value IN: ", gearUltrasonicSensorValue);
    }

    // Make this return true when this Command no longer needs to run execute()
    protected boolean isFinished() {
        return false;
    }

    // Called once after isFinished returns true
    protected void end() {
    }

    // Called when another command which requires one or more of the same
    // subsystems is scheduled to run
    protected void interrupted() {
    }
}
