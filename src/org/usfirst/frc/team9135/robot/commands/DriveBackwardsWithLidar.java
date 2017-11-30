package org.usfirst.frc.team9135.robot.commands;

import org.usfirst.frc.team9135.robot.CommandBase;
import org.usfirst.frc.team9135.robot.subsystems.DriveTrain;
import org.usfirst.frc.team9135.robot.subsystems.Lidars;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;


/**
 *
 */
public class DriveBackwardsWithLidar extends Command {
	private double initialDriveTrainMotorPower;
	private double desiredLidarValueToDriveUntil;
	private double desiredInitialDistanceToTravel;

	private boolean initializeI2CMultiplexerChannel = false;
	private boolean configureLidar = false;

	private int lidarUpperByte = 0;
	private int lidarLowerByte = 0;
	private double currentlidarValueIN = 0.0;

	private boolean startUsingLidarValues = false;

	private boolean robotAtDesiredLidarValue = false;

	private double currentGyroAngle = 0.0;
	private boolean zeroGyroAngle = false;

	private double differenceBetweenDesiredAndCurrentLidarValue = 0.0;
	private static final double DIFFERENCE_BETWEEN_DESIRED_AND_CURRENT_LIDAR_VALUES_TO_DRIVE_SLOWER_PART_1 = 40.0;
	private static final double DIFFERENCE_BETWEEN_DESIRED_AND_CURRENT_LIDAR_VALUES_TO_DRIVE_SLOWER_PART_2 = 20.0;
	private static final double SLOWER_DRIVE_TRAIN_MOTOR_POWER_PART_1 = -.25;
	private static final double SLOWER_DRIVE_TRAIN_MOTOR_POWER_PART_2 = -.15;

	private static final double ESTIMATED_COAST_DISTANCE_IN = 1.0;

	private static final double DISTANCE_BETWEEN_LIDAR_AND_EDGE_OF_BUMPER = 5.0;

	private boolean initializedEncoderDistanceTravel = false;
	private boolean startUsingEncoderValues = false;
	private double initialDistanceTraveled = 0.0;
	private double currentDistanceTraveled = 0.0;
	private double differenceBetweenInitialAndCurrentDistanceTraveled = 0.0;
	private static final double DISTANCE_TO_TRAVEL_SLOW_SPEED = 20.0;
	private static final double ENCODER_BASED_SLOW_MOTOR_POWER = -.45;
	private boolean startEncoderBasedSlowerMotorPower = false;
	private double initialDistanceTraveledForSlowerMotorPower = 0.0;
	private double differenceBetweenCurrentAndInitialEncoderValuesSlowerMotorPower = 0.0;
	private boolean robotTraveledDistanceWithEncoders = false;

	private Timer timer;
	private boolean initializeTimer = false;
	private double timerValue = 0.0;
	private static final double TIME_TO_WAIT_UNTIL_USING_ENCODER_VALUES = .4;

	private boolean turnOnLidar = false;
	private boolean turnOffDetectorBiasBetweenAcquisitions = false;

	private double savedLidarValueIN = 0.0;
	private int goodLidarCounter = 0;
	private int badLidarCounter = 0;

    public DriveBackwardsWithLidar(double initialDriveTrainMotorPower, double desiredLidarValueToDriveUntil, double desiredInitialDistanceToTravel) {
        // Use requires() here to declare subsystem dependencies
        // eg. requires(chassis);
    	requires(CommandBase.driveTrain);
    	requires(CommandBase.lidars);

    	this.initialDriveTrainMotorPower = initialDriveTrainMotorPower;
    	this.desiredLidarValueToDriveUntil = (desiredLidarValueToDriveUntil + DISTANCE_BETWEEN_LIDAR_AND_EDGE_OF_BUMPER);
    	this.desiredInitialDistanceToTravel = desiredInitialDistanceToTravel;

    	timer = new Timer();
    }

    // Called just before this Command runs the first time
    protected void initialize() {
    	CommandBase.lidars.TurnLidarOnOff(Lidars.TURN_LIDAR_ON);
    	turnOnLidar = true;

    	CommandBase.lidars.OpenLidarChannelOnMultiplexer(Lidars.VALUE_TO_OPEN_FRONT_LIDAR_CHANNEL_7);
    	initializeI2CMultiplexerChannel = true;

    	CommandBase.lidars.TurnOffDetectorBiasBetweenLidarAcquisitions();
    	turnOffDetectorBiasBetweenAcquisitions = true;

    	configureLidar = false;

    	startUsingLidarValues = false;

    	robotAtDesiredLidarValue = false;

    	CommandBase.driveTrain.ZeroGyroAngle();
    	zeroGyroAngle = true;

    	initialDistanceTraveled = CommandBase.driveTrain.getDistance(DriveTrain.RIGHT_SIDE_ENCODER);
    	initializedEncoderDistanceTravel = true;

    	startUsingEncoderValues = false;
    	startEncoderBasedSlowerMotorPower = false;
    	robotTraveledDistanceWithEncoders = false;

    	initializeTimer = false;
    }

    // Called repeatedly when this Command is scheduled to run
    protected void execute() {
    	if (turnOnLidar == false) {
    		CommandBase.lidars.TurnLidarOnOff(Lidars.TURN_LIDAR_ON);
    		turnOnLidar = true;
    	}

    	if (initializeI2CMultiplexerChannel == false) {
    		CommandBase.lidars.OpenLidarChannelOnMultiplexer(Lidars.VALUE_TO_OPEN_FRONT_LIDAR_CHANNEL_7);
    		initializeI2CMultiplexerChannel = true;
    	}

    	if (turnOffDetectorBiasBetweenAcquisitions == false) {
    		CommandBase.lidars.TurnOffDetectorBiasBetweenLidarAcquisitions();
    		turnOffDetectorBiasBetweenAcquisitions = true;
    	}

    	if (zeroGyroAngle == false) {
    		CommandBase.driveTrain.ZeroGyroAngle();
    		zeroGyroAngle = true;
    	}

    	if (initializedEncoderDistanceTravel == false) {
    		initialDistanceTraveled = CommandBase.driveTrain.getDistance(DriveTrain.RIGHT_SIDE_ENCODER);
    		initializedEncoderDistanceTravel = true;
    	}

    	currentGyroAngle = CommandBase.driveTrain.getGyroAngle();

    	if (configureLidar == false) {
    		CommandBase.lidars.ConfigureLidar();
    		configureLidar = true;
    	}
    	else if (configureLidar) {
    		lidarUpperByte = CommandBase.lidars.GetUpperByte();
    		try {
				wait((long) .002);
			} catch (InterruptedException e) {
				// TODO Auto-generated catch block
				e.printStackTrace();
			}
    		lidarLowerByte = CommandBase.lidars.GetLowerByte();
    		currentlidarValueIN = CommandBase.lidars.GetLidarValue(lidarLowerByte, lidarUpperByte, Lidars.DISTANCE_UNIT_ARRAY[Lidars.INCHES]);
    		configureLidar = false;
    	}

    	SmartDashboard.putNumber("Autonomous Lidar Value:", currentlidarValueIN);

    	if (currentlidarValueIN != 0.0 && startUsingLidarValues == false && startUsingEncoderValues == false) {
    		if (configureLidar == false) {
    			if (savedLidarValueIN != currentlidarValueIN) {
    				savedLidarValueIN = currentlidarValueIN;
    				goodLidarCounter++;
    			}
    			else if (savedLidarValueIN == currentlidarValueIN) {
    				badLidarCounter++;
    			}

    			if (badLidarCounter >= 3) {
    				startUsingEncoderValues = true;
    				startUsingLidarValues = false;
    			}
    			else if (goodLidarCounter >= 3) {
    				startUsingLidarValues = true;
    				startUsingEncoderValues = false;
    			}
    		}
    	}
    	else if (currentlidarValueIN == 0.0 && startUsingLidarValues == false && startUsingEncoderValues == false) {
    		if (initializeTimer == false) {
    			timer.reset();
    			timer.start();
    			initializeTimer = true;
    		}

    		timerValue = timer.get();

    		if (timerValue >= TIME_TO_WAIT_UNTIL_USING_ENCODER_VALUES) {
    			startUsingEncoderValues = true;
    		}
    	}

    	if (startUsingLidarValues) {
    		System.out.println("Lidar");
    		differenceBetweenDesiredAndCurrentLidarValue = (this.desiredLidarValueToDriveUntil - currentlidarValueIN);
    		if (differenceBetweenDesiredAndCurrentLidarValue <= ESTIMATED_COAST_DISTANCE_IN) {
    			CommandBase.driveTrain.DriveTank(0.0, 0.0);
    			robotAtDesiredLidarValue = true;
    		}
    		else if (differenceBetweenDesiredAndCurrentLidarValue < DIFFERENCE_BETWEEN_DESIRED_AND_CURRENT_LIDAR_VALUES_TO_DRIVE_SLOWER_PART_2) {
    			CommandBase.driveTrain.DriveStraightWithGyro(SLOWER_DRIVE_TRAIN_MOTOR_POWER_PART_2, currentGyroAngle);
    			robotAtDesiredLidarValue = false;
    		}
    		else if (differenceBetweenDesiredAndCurrentLidarValue < DIFFERENCE_BETWEEN_DESIRED_AND_CURRENT_LIDAR_VALUES_TO_DRIVE_SLOWER_PART_1) {
    			CommandBase.driveTrain.DriveStraightWithGyro(SLOWER_DRIVE_TRAIN_MOTOR_POWER_PART_1, currentGyroAngle);
    			robotAtDesiredLidarValue = false;
    		}
    		else {
    			CommandBase.driveTrain.DriveStraightWithGyro(this.initialDriveTrainMotorPower, currentGyroAngle);
    			robotAtDesiredLidarValue = false;
    		}
    	}
    	else if (startUsingEncoderValues) {
    		System.out.println("Encoder");
    		currentDistanceTraveled = CommandBase.driveTrain.getDistance(DriveTrain.RIGHT_SIDE_ENCODER);
    		if (startEncoderBasedSlowerMotorPower == false) {
    			differenceBetweenInitialAndCurrentDistanceTraveled = (Math.abs(currentDistanceTraveled - initialDistanceTraveled));
    			if (differenceBetweenInitialAndCurrentDistanceTraveled >= this.desiredInitialDistanceToTravel) {
    				initialDistanceTraveledForSlowerMotorPower = currentDistanceTraveled;
    				startEncoderBasedSlowerMotorPower = true;
    			}
    			else {
    				CommandBase.driveTrain.DriveStraightWithGyro(this.initialDriveTrainMotorPower, currentGyroAngle);
    			}
    		}

    		if (startEncoderBasedSlowerMotorPower) {
    			differenceBetweenCurrentAndInitialEncoderValuesSlowerMotorPower = (currentDistanceTraveled - initialDistanceTraveledForSlowerMotorPower);
    			if (differenceBetweenCurrentAndInitialEncoderValuesSlowerMotorPower >= DISTANCE_TO_TRAVEL_SLOW_SPEED) {
    				CommandBase.driveTrain.DriveStraightWithGyro(0.0, currentGyroAngle);
    				robotTraveledDistanceWithEncoders = true;
    			}
    			else {
    				CommandBase.driveTrain.DriveStraightWithGyro(ENCODER_BASED_SLOW_MOTOR_POWER, currentGyroAngle);
    			}
    		}
    	}
    	else {
    		CommandBase.driveTrain.DriveStraightWithGyro(this.initialDriveTrainMotorPower, currentGyroAngle);
    	}

    	currentDistanceTraveled = CommandBase.driveTrain.getDistance(DriveTrain.RIGHT_SIDE_ENCODER);
    	System.out.println("Current Distance Traveled: " + currentDistanceTraveled);

    }

    // Make this return true when this Command no longer needs to run execute()
    protected boolean isFinished() {
    	return robotAtDesiredLidarValue || robotTraveledDistanceWithEncoders;
    }

    // Called once after isFinished returns true
    protected void end() {
    	CommandBase.lidars.TurnLidarOnOff(Lidars.TURN_LIDAR_OFF);
    	turnOnLidar = false;

    	CommandBase.driveTrain.DriveTank(0.0, 0.0);

    	initializeI2CMultiplexerChannel = false;
    	turnOffDetectorBiasBetweenAcquisitions = false;
    	configureLidar = false;

    	startUsingLidarValues = false;

    	robotAtDesiredLidarValue = false;

    	zeroGyroAngle = false;

    	initializedEncoderDistanceTravel = false;
    	startUsingEncoderValues = false;
    	startEncoderBasedSlowerMotorPower = false;
    	robotTraveledDistanceWithEncoders = false;

    	initializeTimer = false;
    }

    // Called when another command which requires one or more of the same
    // subsystems is scheduled to run
    protected void interrupted() {
    	end();
    }
}
