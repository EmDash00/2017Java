package org.usfirst.frc.team9135.robot.commands;

import org.usfirst.frc.team9135.robot.CommandBase;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.command.Command;

/**
 *
 */
public class TurnDriveTrainAngle extends Command {
	
	private double desiredAngleToTurn;
	private double motorPower;
	private boolean turnRight;
	//boolean setTimeout;

	private boolean zeroedGyro = false;

	private double currenGyroAngle = 0.0;

	private boolean turnAngleComplete = false;

	private boolean requiresDriveTrain = false;

	private Timer timer;
	private boolean initializeTimer = false;
	private double timerValue = 0.0;
	private static final double TURN_ROBOT_INTO_HOPPER_TIMEOUT = 1.0;

    public TurnDriveTrainAngle(double desiredAngleToTurn, double motorPower, boolean turnRight) {
        // Use requires() here to declare subsystem dependencies
        // eg. requires(chassis);
    	requires(CommandBase.driveTrain);
    	this.desiredAngleToTurn = desiredAngleToTurn;
    	this.motorPower = motorPower;
    	this.turnRight = turnRight;
    	//this.setTimeout = setTimeout;

    	timer = new Timer();
    }

    // Called just before this Command runs the first time
    protected void initialize() {
    	CommandBase.driveTrain.ZeroGyroAngle();
    	zeroedGyro = true;
    	turnAngleComplete = false;
    	CommandBase.driveTrain.is_aiming = true;
    	requiresDriveTrain = true;
    	timer.reset();
    	timer.start();
    	initializeTimer = true;
    }

    // Called repeatedly when this Command is scheduled to run
    protected void execute() {
    	if (requiresDriveTrain == false) {
    		CommandBase.driveTrain.is_aiming = true;
    		requiresDriveTrain = true;
    	}

    	if (zeroedGyro == false) {
    		CommandBase.driveTrain.ZeroGyroAngle();
    		zeroedGyro = true;
    	}

    	if (initializeTimer == false) {
    		timer.reset();
    		timer.start();
    		initializeTimer = true;
    	}

    	timerValue = timer.get();

    	currenGyroAngle = Math.abs(CommandBase.driveTrain.getGyroAngle());

    	if (currenGyroAngle >= this.desiredAngleToTurn) {
    		CommandBase.driveTrain.RotateTank(0.0, this.turnRight);
    		turnAngleComplete = true;
    	}
    	else {
    		CommandBase.driveTrain.RotateTank(this.motorPower, this.turnRight);
    	}
    }

    // Make this return true when this Command no longer needs to run execute()
    protected boolean isFinished() {
    	return turnAngleComplete;
    }

    // Called once after isFinished returns true
    protected void end() {
    	CommandBase.driveTrain.RotateTank(0.0, this.turnRight);
    	zeroedGyro = false;
    	turnAngleComplete = false;
    	CommandBase.driveTrain.is_aiming = false;
    	requiresDriveTrain = false;
    	initializeTimer = false;
    	timer.stop();
    	timer.reset();
    }

    // Called when another command which requires one or more of the same
    // subsystems is scheduled to run
    protected void interrupted() {
    	end();
    }
}
