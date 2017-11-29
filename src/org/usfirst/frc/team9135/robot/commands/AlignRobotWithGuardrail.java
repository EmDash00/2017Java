package org.usfirst.frc.team9135.robot.commands;

import edu.wpi.first.wpilibj.command.Command;

import org.usfirst.frc.team9135.robot.CommandBase;
import org.usfirst.frc.team9135.robot.subsystems.*;

/**
 *
 */
public class AlignRobotWithGuardrail extends Command {

	private double driveTrainMotorPower;
	private boolean rightHopper;

	private double initialFrontUltrasonicSensorValue = 0.0;
	private double initialBackUltrasonicSensorValue = 0.0;

	private boolean getDesiredAngleToTurn = false;
	private double desiredAngleToTurn = 0.0;

	private boolean zeroGyro = false;
	private double currentGyroAngle = 0.0;

	private boolean alignedRobotWithGuardRail = false;

	private boolean pingedFrontUltrasonicSensor = false;
	private boolean pingedBackUltrasonicSensor = true;
	
    public AlignRobotWithGuardrail(double driveTrainMotorPower, boolean rightHopper) {
        // Use requires() here to declare subsystem dependencies
        // eg. requires(chassis);
    	requires(CommandBase.ultrasonicSensor);
    	requires(CommandBase.driveTrain);

    	this.driveTrainMotorPower = driveTrainMotorPower;
    	this.rightHopper = rightHopper;
    }

    // Called just before this Command runs the first time
    protected void initialize() {
    	pingedFrontUltrasonicSensor = false;
    	pingedBackUltrasonicSensor = true;

    	getDesiredAngleToTurn = false;

    	CommandBase.driveTrain.ZeroGyroAngle();
    	zeroGyro = true;

    	alignedRobotWithGuardRail = false;

    	CommandBase.ultrasonicSensor.usingUltrasonicSensor = true;
    }

    // Called repeatedly when this Command is scheduled to run
    protected void execute() {
    	if (zeroGyro == false) {
    		CommandBase.driveTrain.ZeroGyroAngle();
    		zeroGyro = true;
    	}

    	if (pingedFrontUltrasonicSensor == false) {
    		CommandBase.ultrasonicSensor.PingGearUltrasonicSensor();
    		pingedFrontUltrasonicSensor = true;
    	}
    	else if (pingedFrontUltrasonicSensor) {
    		initialFrontUltrasonicSensorValue = CommandBase.ultrasonicSensor.GetGearUltrasonicSensorValueInches();
    		pingedFrontUltrasonicSensor = false;
    	}

    	if (pingedBackUltrasonicSensor == false) {
    		CommandBase.ultrasonicSensor.PingSideUltrasonicSensor(UltrasonicSensor.LEFT_SIDE_ULTRASONIC_SENSOR);
    		pingedBackUltrasonicSensor = true;
    	}
    	else if (pingedBackUltrasonicSensor) {
    		initialBackUltrasonicSensorValue = CommandBase.ultrasonicSensor.GetSideUltrasonicSensorValueInches(UltrasonicSensor.LEFT_SIDE_ULTRASONIC_SENSOR);
    		pingedBackUltrasonicSensor = false;
    	}

    	if (getDesiredAngleToTurn == false) {
    		if (initialFrontUltrasonicSensorValue < 5000.0 && initialFrontUltrasonicSensorValue > 0.0 && initialBackUltrasonicSensorValue < 5000.0 && initialBackUltrasonicSensorValue > 0.0) {
    			desiredAngleToTurn = CommandBase.ultrasonicSensor.GetAngleToTurnToAlignWithGuardRail(initialFrontUltrasonicSensorValue, initialBackUltrasonicSensorValue, this.rightHopper);
    			getDesiredAngleToTurn = true;
    		}
    	}
    	else if (getDesiredAngleToTurn) {
    		currentGyroAngle = CommandBase.driveTrain.GetGyroAngle();

    		if (desiredAngleToTurn > 0.0) {
    			//std::cout << "Turning Right" << std::endl;
    			if (currentGyroAngle >= desiredAngleToTurn) {
    				CommandBase.driveTrain.DriveTank(0.0, 0.0);
    				alignedRobotWithGuardRail = true;
    			}
    			else {
    				//std::cout << "Moving Right" << std::endl;
    				CommandBase.driveTrain.DriveTank(0.0, (-1 * this.driveTrainMotorPower));
    			}
    		}
    		else if (desiredAngleToTurn < 0.0) {
    			//std::cout << "Turning Left" << std::endl;
    			if (currentGyroAngle <= desiredAngleToTurn) {
    				CommandBase.driveTrain.DriveTank(0.0, 0.0);
    				alignedRobotWithGuardRail = true;
    			}
    			else {
    				//std::cout << "Moving Left" << std::endl;
    				CommandBase.driveTrain.DriveTank((-1 * this.driveTrainMotorPower), 0.0);
    			}
    		}
    		else if (desiredAngleToTurn == 0.0) {
    			alignedRobotWithGuardRail = true;
    		}
    	}
    }

    // Make this return true when this Command no longer needs to run execute()
    protected boolean isFinished() {
    	return alignedRobotWithGuardRail;
    }

    // Called once after isFinished returns true
    protected void end() {
    	CommandBase.driveTrain.DriveTank(0.0, 0.0);
    	getDesiredAngleToTurn = false;
    	alignedRobotWithGuardRail = false;
    	CommandBase.ultrasonicSensor.usingUltrasonicSensor = false;
    }

    // Called when another command which requires one or more of the same
    // subsystems is scheduled to run
    protected void interrupted() {
    	end();
    }
}
