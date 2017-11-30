package org.usfirst.frc.team9135.robot.commands;

import org.usfirst.frc.team9135.robot.CommandBase;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.command.Command;

/**
 *
 */
public class AutoRotateRobotForGearPeg extends Command {
	private double motorPower;
	private double desiredAngleToRotateRobot;

	private static final boolean TURN_RIGHT = false;
	private static final boolean TURN_LEFT = !TURN_RIGHT;

	private boolean zeroedGyro = false;

	Timer timer;
	
	private double timerValue = 0.0;
	private static final double FIRST_ROTATION_TIMEOUT = .35;
	private static final double TIME_TO_WAIT_BETWEEN_FIRST_AND_SECOND_ROTATIONS = .05;
	private static final double SECOND_ROTATION_TIMEOUT = .7;
	private static final double TIME_TO_WAIT_BETWEEN_SECOND_AND_THIRD_ROTATIONS = .05;
	private static final double THIRD_ROTATION_TIMEOUT = .35;

	private boolean initializeFirstRotationTimer = false;
	private boolean initializeFirstTimeWait = false;
	private boolean initializeSecondRotationTimer = false;
	private boolean initializeSecondTimeWait = false;
	private boolean initializeThirdRotationTimer = false;

	private boolean firstRotationComplete = false;
	private boolean waitBetweenFirstAndSecondRotations = false;
	private boolean secondRotationComplete = false;
	private boolean waitBetweenSecondAndThirdRotations = false;
	private boolean thirdRotationComplete = false;

	private double angleTurnedDuringFirstRotation = 0.0;
	private double angleToTurnForSecondRotation = 0.0;
	private double angleToTurnForThirdRotation = 0.0;

	private boolean initializedNewAutoRotateRobotFirstRotation = false;
	private boolean initializedNewAutoRotateRobotSecondRotation = false;
	private boolean initializedNewAutoRotateRobotThirdRotation = false;

    public AutoRotateRobotForGearPeg(double motorPower, double desiredAngleToRotateRobot) {
    	
        // Use requires() here to declare subsystem dependencies
        // eg. requires(chassis);
    	requires(CommandBase.driveTrain);

    	this.desiredAngleToRotateRobot = desiredAngleToRotateRobot;
    	this.motorPower = motorPower;

    	timer = new Timer();
    }

    // Called just before this Command runs the first time
    protected void initialize() {
    	CommandBase.driveTrain.ZeroGyroAngle();
    	zeroedGyro = true;

    	timer.reset();
    	timer.start();
    	initializeFirstRotationTimer = true;
    	initializeFirstTimeWait = false;
    	initializeSecondRotationTimer = false;
    	initializeSecondTimeWait = false;
    	initializeThirdRotationTimer = false;

    	firstRotationComplete = false;
    	waitBetweenFirstAndSecondRotations = false;
    	secondRotationComplete = false;
    	waitBetweenSecondAndThirdRotations = false;
    	thirdRotationComplete = false;

    	initializedNewAutoRotateRobotFirstRotation = false;
    	initializedNewAutoRotateRobotSecondRotation = false;
    	initializedNewAutoRotateRobotThirdRotation = false;

    	CommandBase.driveTrain.is_aiming = true;
    }

    // Called repeatedly when this Command is scheduled to run
    protected void execute() {
    	if (zeroedGyro == false) {
    		CommandBase.driveTrain.ZeroGyroAngle();
    		zeroedGyro = true;
    	}

    	if (firstRotationComplete == false) {
    		if (initializeFirstRotationTimer == false) {
    			timer.stop();
    			timer.reset();
    			timer.start();
    			initializeFirstRotationTimer = true;
    		}

    		timerValue = timer.get();

    		if (initializeFirstRotationTimer) {
    			if (timerValue >= FIRST_ROTATION_TIMEOUT) {
    				CommandBase.driveTrain.DriveTank(0.0, 0.0);
    				firstRotationComplete = true;
    			}
    			else {
    				firstRotationComplete = CommandBase.driveTrain.AutoRotateRobot(this.motorPower, this.desiredAngleToRotateRobot, TURN_LEFT, initializedNewAutoRotateRobotFirstRotation);
    				if (initializedNewAutoRotateRobotFirstRotation == false) {
    					initializedNewAutoRotateRobotFirstRotation = true;
    				}
    				System.out.println("First Rotation Complete: " + firstRotationComplete);
    			}
    		}
    	}
    	else if (firstRotationComplete && waitBetweenFirstAndSecondRotations == false) {
    		System.out.println("First Rotation Complete");
    		if (initializeFirstTimeWait == false) {
    			timer.stop();
    			timer.reset();
    			timer.start();
    			initializeFirstTimeWait = true;
    		}

    		timerValue = timer.get();

    		CommandBase.driveTrain.DriveTank(0.0, 0.0);
    		if (timerValue >= TIME_TO_WAIT_BETWEEN_FIRST_AND_SECOND_ROTATIONS) {
    			waitBetweenFirstAndSecondRotations = true;
    		}
    	}
    	else if (firstRotationComplete && waitBetweenFirstAndSecondRotations && secondRotationComplete == false) {
    		if (initializeSecondRotationTimer == false) {
    			angleTurnedDuringFirstRotation = Math.abs(CommandBase.driveTrain.getGyroAngle());
    			angleToTurnForSecondRotation = (2.0 * angleTurnedDuringFirstRotation);
    			timer.stop();
    			timer.reset();
    			timer.start();
    			initializeSecondRotationTimer = true;
    		}

    		timerValue = timer.get();

    		if (initializeSecondRotationTimer) {
    			if (timerValue >= SECOND_ROTATION_TIMEOUT) {
    				CommandBase.driveTrain.DriveTank(0.0, 0.0);
    				secondRotationComplete = true;
    			}
    			else {
    				secondRotationComplete = CommandBase.driveTrain.AutoRotateRobot(this.motorPower, angleToTurnForSecondRotation, TURN_RIGHT, initializedNewAutoRotateRobotSecondRotation);
    				if (initializedNewAutoRotateRobotSecondRotation == false) {
    					initializedNewAutoRotateRobotSecondRotation = true;
    				}
    			}
    		}

    	}
    	else if (secondRotationComplete && waitBetweenSecondAndThirdRotations == false) {
    		System.out.println("Second Rotation Complete");
    		if (initializeSecondTimeWait == false) {
    			timer.stop();
    			timer.reset();
    			timer.start();
    			initializeSecondTimeWait = true;
    		}

    		timerValue = timer.get();

    		CommandBase.driveTrain.DriveTank(0.0, 0.0);

    		if (timerValue >= TIME_TO_WAIT_BETWEEN_SECOND_AND_THIRD_ROTATIONS) {
    			waitBetweenSecondAndThirdRotations = true;
    		}
    	}
    	else if (secondRotationComplete && waitBetweenSecondAndThirdRotations && thirdRotationComplete == false) {
    		System.out.println("Third Rotation Running");
    		if (initializeThirdRotationTimer == false) {
    			angleToTurnForThirdRotation = Math.abs(CommandBase.driveTrain.getGyroAngle());
    			System.out.println("Desired Angle: " + angleToTurnForThirdRotation);
    			timer.stop();
    			timer.reset();
    			timer.start();
    			initializeThirdRotationTimer = true;
    		}

    		timerValue = timer.get();
    		System.out.println("Third Rotation Timer Value: " + timerValue);

    		if (initializeThirdRotationTimer) {
    			if (timerValue >= THIRD_ROTATION_TIMEOUT) {
    				CommandBase.driveTrain.DriveTank(0.0, 0.0);
    				thirdRotationComplete = true;
    				System.out.println("TImeeerrrreerrr");
    			}
    			else {
    				thirdRotationComplete = CommandBase.driveTrain.AutoRotateRobot(this.motorPower, angleToTurnForThirdRotation, TURN_LEFT, initializedNewAutoRotateRobotThirdRotation);
    				System.out.println("Third Rotation Complete? " + thirdRotationComplete);
    				if (initializedNewAutoRotateRobotThirdRotation == false) {
    					initializedNewAutoRotateRobotThirdRotation = true;
    				}
    			}
    		}
    	}
    }

    // Make this return true when this Command no longer needs to run execute()
    protected boolean isFinished() {
    	return thirdRotationComplete;
    }

    // Called once after isFinished returns true
    protected void end() {
    	System.out.println("Third Rotation Complete");
    	CommandBase.driveTrain.DriveTank(0.0, 0.0);

    	zeroedGyro = false;

    	initializeFirstRotationTimer = false;
    	initializeFirstTimeWait = false;
    	initializeSecondRotationTimer = false;
    	initializeSecondTimeWait = false;
    	initializeThirdRotationTimer = false;

    	firstRotationComplete = false;
    	waitBetweenFirstAndSecondRotations = false;
    	secondRotationComplete = false;
    	waitBetweenSecondAndThirdRotations = false;
    	thirdRotationComplete = false;

    	initializedNewAutoRotateRobotFirstRotation = false;
    	initializedNewAutoRotateRobotSecondRotation = false;
    	initializedNewAutoRotateRobotThirdRotation = false;

    	CommandBase.driveTrain.is_aiming = false;
    }

    // Called when another command which requires one or more of the same
    // subsystems is scheduled to run
    protected void interrupted() {
    	end();
    }
}
