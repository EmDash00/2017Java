package org.usfirst.frc.team9135.robot.commands;

import org.usfirst.frc.team9135.robot.CommandBase;

import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.Timer;

/**
 *
 */
public class AutoDriveShooter extends Command {
	private int setpointRPM;
	private boolean closeShotPIDSlot;

	private boolean initializePIDSlot = false;

	int currentShooterWheelRPM = 0;
	private boolean shooterVoltageMode = false;
	private boolean shooterPIDMode = false;

	private boolean beginTimerToLevelOutSetpoint = false;
	private boolean moveAgitatorToShootFuel = false;

	Timer timer;
	private double timerValue = 0.0;
	private static double WAIT_TIME_FOR_SHOOTER = .05;

	private static final double WAIT_TIME_FOR_FUEL_TO_SHOOT_WITHOUT_COLLECTION = 6.0;
	private static final double WAIT_TIME_FOR_FUEL_TO_SHOOT_WITH_COLLECTION = 6.0;

	private static final double TIME_TO_WAIT_UNTIL_RUNNING_COLLECTION_AFTER_AGITATOR_STARTS = 2.75;

	private static final double TIME_TO_DRIVE_COLLECTION = 1.0;

	private static final double AUTO_DRIVE_SHOOTER_TIMEOUT = 6.0;

	private static final double AGITATOR_MOTOR_POWER = 1.0;
	private static final double AGITATOR_2_MOTOR_POWER = 1.0;

	private boolean startTimerForAgitator = false;
	private boolean shotFuelIntoBoiler = false;

	private static final double COLLECTION_MOTOR_POWER = 1.0;
	private boolean startRunningCollection = false;

	private boolean initializeTimerToRunCollection = false;
	private double initialTimeForRunningCollection = 0.0;
	private double differenceBetweenCurrentTimeAndInitialTimeForRunningCollection = 0.0;

	private boolean doneDrivingCollection = false;
    public AutoDriveShooter(int setpointRPM, boolean closeShotPIDSlot) {
    	
        // Use requires() here to declare subsystem dependencies
        // eg. requires(chassis);
    	
    	// Use requires() here to declare subsystem dependencies
    	// eg. requires(Robot::chassis.get());
    	requires(CommandBase.shooter);
    	requires(CommandBase.agitator);
    	requires(CommandBase.collection);

    	this.setpointRPM = setpointRPM;
    	this.closeShotPIDSlot = closeShotPIDSlot;

    	timer = new Timer();
    }

    // Called just before this Command runs the first time
    protected void initialize() {
    	setTimeout(AUTO_DRIVE_SHOOTER_TIMEOUT);

    	CommandBase.shooter.ConfigureShooterPID();
    	shooterPIDMode = true;
    	shooterVoltageMode = false;

    	beginTimerToLevelOutSetpoint = false;
    	moveAgitatorToShootFuel = false;
    	startTimerForAgitator = false;
    	shotFuelIntoBoiler = false;

    	timer.reset();

    	initializeTimerToRunCollection = false;
    	startRunningCollection = false;

    	doneDrivingCollection = false;

    	initializePIDSlot = false;
    }

    // Called repeatedly when this Command is scheduled to run
    protected void execute() {
    	timerValue = timer.get();
    	System.out.println("shooting: " + timerValue);
    	if (shooterPIDMode == false) {
    		CommandBase.shooter.ConfigureShooterPID();
    		shooterPIDMode = true;
    		shooterVoltageMode = false;
    	}

    	if (initializePIDSlot == false) {
    		if (this.closeShotPIDSlot) {
    			CommandBase.shooter.SelectPIDProfileSlot(Shooter::CLOSE_SHOT_PID_VALUES);
    		}
    		else if (this.closeShotPIDSlot == false) {
    			CommandBase.shooter.SelectPIDProfileSlot(Shooter::FAR_SHOT_PID_VALUES);
    		}
    		initializePIDSlot = true;
    	}

    	currentShooterWheelRPM = CommandBase.shooter.getShooterWheelRPM();
    	CommandBase.shooter.DriveShooterMotor(this.setpointRPM);

    	if ((currentShooterWheelRPM >= this.setpointRPM) && beginTimerToLevelOutSetpoint == false) {
    		beginTimerToLevelOutSetpoint = true;
    		timer.reset();
    		timer.start();
    	}
    	else if (beginTimerToLevelOutSetpoint && timerValue >= WAIT_TIME_FOR_SHOOTER && moveAgitatorToShootFuel == false) {
    		moveAgitatorToShootFuel = true;
    		timer.stop();
    		timer.reset();
    	}

    	if (moveAgitatorToShootFuel) {
    		if (startTimerForAgitator == false) {
    			CommandBase.agitator.DriveAgitator(AGITATOR_MOTOR_POWER, AGITATOR_2_MOTOR_POWER);
    			timer.reset();
    			timer.start();
    			startTimerForAgitator = true;
    		}
    		else if (startTimerForAgitator && timerValue < WAIT_TIME_FOR_FUEL_TO_SHOOT_WITH_COLLECTION) {
    			CommandBase.agitator.DriveAgitator(AGITATOR_MOTOR_POWER, AGITATOR_2_MOTOR_POWER);
    		}
    		else if (startTimerForAgitator && timerValue >= WAIT_TIME_FOR_FUEL_TO_SHOOT_WITH_COLLECTION) {
    			CommandBase.agitator.DriveAgitator(0.0, 0.0);
    			timer.stop();
    			timer.reset();
    			if (shooterVoltageMode == false) {
    				CommandBase.shooter.ConfigureShooterVoltageMode();
    				shooterPIDMode = false;
    				shooterVoltageMode = true;
    			}
    			CommandBase.shooter.DriveShooterMotor(0.0);
    			shotFuelIntoBoiler = true;
    		}
    	}

    	if (timerValue >= TIME_TO_WAIT_UNTIL_RUNNING_COLLECTION_AFTER_AGITATOR_STARTS && moveAgitatorToShootFuel && startRunningCollection == false && doneDrivingCollection == false) {
    		startRunningCollection = true;
    	}

    	if (startRunningCollection && doneDrivingCollection == false) {
    		timerValue = timer.get();
    		if (initializeTimerToRunCollection == false) {
    			initialTimeForRunningCollection = timerValue;
    			initializeTimerToRunCollection = true;
    		}
    		differenceBetweenCurrentTimeAndInitialTimeForRunningCollection = (timerValue - initialTimeForRunningCollection);

    		if (differenceBetweenCurrentTimeAndInitialTimeForRunningCollection < TIME_TO_DRIVE_COLLECTION) {
    			CommandBase.collection.DriveCollection(COLLECTION_MOTOR_POWER);
    		}
    		else if (differenceBetweenCurrentTimeAndInitialTimeForRunningCollection >= TIME_TO_DRIVE_COLLECTION) {
    			CommandBase.collection.DriveCollection(0.0);
    			doneDrivingCollection = true;
    		}
    	}
    }

    // Make this return true when this Command no longer needs to run execute()
    protected boolean isFinished() {
    	return shotFuelIntoBoiler || isTimedOut();
    }

    // Called once after isFinished returns true
    protected void end() {
    	CommandBase.shooter.ConfigureShooterVoltageMode();
    	shooterPIDMode = false;
    	shooterVoltageMode = true;
    	CommandBase.shooter.DriveShooterMotor(0.0);
    	CommandBase.agitator.DriveAgitator(0.0, 0.0);
    	CommandBase.collection.DriveCollection(0.0);

    	beginTimerToLevelOutSetpoint = false;
    	moveAgitatorToShootFuel = false;
    	startTimerForAgitator = false;
    	shotFuelIntoBoiler = false;

    	startRunningCollection = false;
    	initializeTimerToRunCollection = false;
    	doneDrivingCollection = false;

    	initializePIDSlot = false;
    }

    // Called when another command which requires one or more of the same
    // subsystems is scheduled to run
    protected void interrupted() {
    	end();
    }
}
