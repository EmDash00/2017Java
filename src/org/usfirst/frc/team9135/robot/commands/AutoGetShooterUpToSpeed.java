package org.usfirst.frc.team9135.robot.commands;

import org.usfirst.frc.team9135.robot.CommandBase;

import edu.wpi.first.wpilibj.Timer;

import edu.wpi.first.wpilibj.command.Command;

/**
 *
 */
public class AutoGetShooterUpToSpeed extends Command {
	private double desiredShooterRPM;
	private boolean closeShotPID;

	private boolean initializePID = false;
	private boolean initializeVoltageMode = false;
	private boolean initializeBooleans = false;

	private Timer timer;
	private boolean startTimer = false;
	private double timerValue = 0.0;
	private static final double TIME_TO_WAIT_FOR_SHOOTER_TO_MAINTAIN_VELOCITY = .1;

	private double currentShooterRPMValue = 0.0;

	private boolean maintainShooterRPM = false;
	private boolean shooterUpToSpeed = false;

	private boolean initializePIDSlot = false;
	
	
    public AutoGetShooterUpToSpeed() {
        // Use requires() here to declare subsystem dependencies
        // eg. requires(chassis);
    	requires(CommandBase.shooter);

    	this.desiredShooterRPM = desiredShooterRPM;
    	this.closeShotPID = closeShotPID;

    	timer = new Timer();
    }

    // Called just before this Command runs the first time
    protected void initialize() {
    	CommandBase.shooter.ConfigureShooterPID();
    	initializeVoltageMode = false;
    	initializePID = true;
    	maintainShooterRPM = false;
    	shooterUpToSpeed = false;
    	timer.reset();
    	timer.start();
    	startTimer = true;

    	initializePIDSlot = false;
    }

    // Called repeatedly when this Command is scheduled to run
    protected void execute() {
    	if (initializePID == false) {
    		CommandBase.shooter.ConfigureShooterPID();
    		initializeVoltageMode = false;
    		initializePID = true;
    	}

    	if (initializePIDSlot == false) {
    		if (this.closeShotPID) {
    			CommandBase.shooter.SelectPIDProfileSlot(Shooter.CLOSE_SHOT_PID_VALUES);
    		}
    		else if (this.closeShotPID == false) {
    			CommandBase.shooter.SelectPIDProfileSlot(Shooter.FAR_SHOT_PID_VALUES);
    		}
    		initializePIDSlot = true;
    	}

    	timerValue = timer.Get();

    	CommandBase.shooter.DriveShooterMotor(this.desiredShooterRPM);

    	currentShooterRPMValue = CommandBase.shooter.GetShooterWheelRPM();
    	if (((currentShooterRPMValue >= this.desiredShooterRPM) && shooterUpToSpeed == false) || maintainShooterRPM) {
    		if (startTimer == false) {
    			timer.Reset();
    			timer.Start();
    			startTimer = true;
    			maintainShooterRPM = true;
    		}

    		if (startTimer && (timerValue < TIME_TO_WAIT_FOR_SHOOTER_TO_MAINTAIN_VELOCITY)) {
    			shooterUpToSpeed = false;
    			maintainShooterRPM = true;
    		}
    		else if (startTimer && (timerValue >= TIME_TO_WAIT_FOR_SHOOTER_TO_MAINTAIN_VELOCITY)) {
    			maintainShooterRPM = false;
    			shooterUpToSpeed = true;
    		}
    		CommandBase.shooter.ShooterUpToSpeed(shooterUpToSpeed);
    	}
    }

    // Make this return true when this Command no longer needs to run execute()
    protected boolean isFinished() {
    	return CommandBase.shooter.stopShooterFromDriving;
    }

    // Called once after isFinished returns true
    protected void end() {
    	CommandBase.shooter.ConfigureShooterVoltageMode();
    	initializePID = false;
    	initializePIDSlot = true;
    	initializeVoltageMode = true;
    	CommandBase.shooter.DriveShooterMotor(0.0);
    	maintainShooterRPM = false;
    	shooterUpToSpeed = false;
    	CommandBase.shooter.ShooterUpToSpeed(shooterUpToSpeed);
    }

    // Called when another command which requires one or more of the same
    // subsystems is scheduled to run
    protected void interrupted() {
    	end();
    }
}
