package org.usfirst.frc.team9135.robot.commands;

import org.usfirst.frc.team9135.robot.CommandBase;
import org.usfirst.frc.team9135.robot.subsystems.*;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;


public class DriveToGearWithLidar extends Command {
	private double driveTrainMotorPower;
	private boolean rightGear;

	private boolean requiresDriveTrainSubsystem = false;

	private double gyroAngle = 0.0;
	private boolean zeroGyro = false;

	private boolean initializeI2CMultiplxerChannelToOpen = false;
	private boolean configureLidar = false;
	private boolean receiveLidarUpperByte = false;

	private int lidarUpperByte = 0;
	private int lidarLowerByte = 0;
	private double currentLidarValueIN = 0.0;

	private static final double DISTANCE_TO_TRAVEL_TO_START_DETECTING_AIRSHIP = 0.0;
	private boolean startLidarDetectingGearPeg = false;
	private double savedLidarTransparentAirshipValue = 0.0;
	private boolean waitingForGearPeg = false;
	private static final double DISTANCE_DROP_OF_LIDAR_FROM_ARISHIP_TO_GEAR_PEG_IN = 30.0;
	private boolean lidarDetectsGearPeg = false;
	private boolean traveledToTurningRadiusOfRobot = false;

	private static final boolean RIGHT_GEAR = true;
	private static final boolean LEFT_GEAR = !RIGHT_GEAR;

	Timer timer;
	private double currentTimerValue = 0.0;
	private static final double TIME_TO_WAIT_TO_REACT_TO_GEAR_PEG = .05;

	private boolean initializeGearPegLidarTimer = false;

	private static final double DISTANCE_BETWEEN_LIDAR_AND_TURNING_POINT_OF_ROBOT = 12.0;

	private boolean configureInitialDistanceBeforeTravelingExtraDistance = false;
	private double initialDistanceTraveledBeforeTravelingExtraDistance = 0.0;
	private double currentDistanceTraveledWhileTravelingExtraDistance = 0.0;
	private double differenceBetweenCurrentAndInitialDistanceWhileTravelingExtraDistance = 0.0;
	private static final double DRIVE_TRAIN_MOTOR_POWER_FOR_EXTRA_DISTANCE = .25;
	
	
    public DriveToGearWithLidar() {
        // Use requires() here to declare subsystem dependencies
        // eg. requires(chassis);
    	requires(CommandBase.driveTrain);
    	requires(CommandBase.lidars);


    	this.driveTrainMotorPower = driveTrainMotorPower;
    	this.rightGear = rightGear;

    	timer = new Timer();
    }

    // Called just before this Command runs the first time
    protected void initialize() {
    	CommandBase.driveTrain.ZeroGyroAngle();
    	zeroGyro = true;
    	CommandBase.driveTrain.is_aiming = true;
    	requiresDriveTrainSubsystem = true;

    	initializeI2CMultiplxerChannelToOpen = false;
    	configureLidar = false;
    	receiveLidarUpperByte = false;
    	startLidarDetectingGearPeg = false;
    	initializeGearPegLidarTimer = false;
    	waitingForGearPeg = false;
    	lidarDetectsGearPeg = false;
    	traveledToTurningRadiusOfRobot = false;
    	configureInitialDistanceBeforeTravelingExtraDistance = false;
    }

    // Called repeatedly when this Command is scheduled to run
    protected void execute() {
    	if (requiresDriveTrainSubsystem == false) {
    		CommandBase.driveTrain.is_aiming = true;
    		requiresDriveTrainSubsystem = true;
    	}

    	if (zeroGyro == false) {
    		CommandBase.driveTrain.ZeroGyroAngle();
    		zeroGyro = true;
    	}

    	gyroAngle = CommandBase.driveTrain.getGyroAngle();

    	if (lidarDetectsGearPeg == false) {
    		CommandBase.driveTrain.DriveStraightWithGyro(this.driveTrainMotorPower, gyroAngle);
    	}

    	if (initializeI2CMultiplxerChannelToOpen == false) {
    		if (this.rightGear == RIGHT_GEAR) {
    			CommandBase.lidars.OpenLidarChannelOnMultiplexer(Lidars.VALUE_TO_OPEN_LIDAR_CHANNEL_6_RIGHT_LIDAR);
    		}
    		else if (this.rightGear == LEFT_GEAR) {
    			CommandBase.lidars.OpenLidarChannelOnMultiplexer(Lidars.VALUE_TO_OPEN_LIDAR_CHANNEL_7_LEFT_LIDAR);
    		}
    		initializeI2CMultiplxerChannelToOpen = true;
    	}

    	if (configureLidar == false) {
    		CommandBase.lidars.ConfigureLidar();
    		configureLidar = true;
    	}
    	else if (configureLidar && receiveLidarUpperByte == false) {
    		lidarUpperByte = CommandBase.lidars.GetUpperByte();
    		receiveLidarUpperByte = true;
    	}
    	else if  (configureLidar && receiveLidarUpperByte) {
    		lidarLowerByte = CommandBase.lidars.GetLowerByte();
    		currentLidarValueIN = CommandBase.lidars.GetLidarValue(lidarLowerByte, lidarUpperByte, Lidars.DISTANCE_UNIT_ARRAY[Lidars.INCHES]);
    		configureLidar = false;
    		receiveLidarUpperByte = false;
    		startLidarDetectingGearPeg = true;
    	}

    	if (startLidarDetectingGearPeg && waitingForGearPeg == false) {
    		savedLidarTransparentAirshipValue = currentLidarValueIN;
    		waitingForGearPeg = true;
    	}
    	else if (startLidarDetectingGearPeg && waitingForGearPeg && lidarDetectsGearPeg == false) {
    		if ((currentLidarValueIN + DISTANCE_DROP_OF_LIDAR_FROM_ARISHIP_TO_GEAR_PEG_IN) <= savedLidarTransparentAirshipValue) {
    			if (initializeGearPegLidarTimer == false) {
    				timer.reset();
    				timer.start();
    				initializeGearPegLidarTimer = true;
    			}

    			currentTimerValue = timer.get();

    			if (initializeGearPegLidarTimer && (currentTimerValue >= TIME_TO_WAIT_TO_REACT_TO_GEAR_PEG)) {
    				lidarDetectsGearPeg = true;
    			}
    		}
    		else {
    			timer.stop();
    			timer.reset();
    			initializeGearPegLidarTimer = false;
    			lidarDetectsGearPeg = false;
    		}
    	}

    	currentDistanceTraveledWhileTravelingExtraDistance = CommandBase.driveTrain.getDistance(DriveTrain.RIGHT_SIDE_ENCODER);
    	if (lidarDetectsGearPeg) {
    		if (configureInitialDistanceBeforeTravelingExtraDistance == false) {
    			initialDistanceTraveledBeforeTravelingExtraDistance = currentDistanceTraveledWhileTravelingExtraDistance;
    			configureInitialDistanceBeforeTravelingExtraDistance = true;
    		}

    		differenceBetweenCurrentAndInitialDistanceWhileTravelingExtraDistance = (Math.abs(currentDistanceTraveledWhileTravelingExtraDistance - initialDistanceTraveledBeforeTravelingExtraDistance));

    		if (differenceBetweenCurrentAndInitialDistanceWhileTravelingExtraDistance >= DISTANCE_BETWEEN_LIDAR_AND_TURNING_POINT_OF_ROBOT) {
    			CommandBase.driveTrain.DriveTank(0.0, 0.0);
    			traveledToTurningRadiusOfRobot = true;
    		}
    		else {
    			CommandBase.driveTrain.DriveStraightWithGyro(DRIVE_TRAIN_MOTOR_POWER_FOR_EXTRA_DISTANCE, gyroAngle);
    			traveledToTurningRadiusOfRobot = false;
    		}
    	}

    	SmartDashboard.putNumber("Auto Lidar Value:", currentLidarValueIN);

    }

    // Make this return true when this Command no longer needs to run execute()
    protected boolean isFinished() {
    	return traveledToTurningRadiusOfRobot;
    }

    // Called once after isFinished returns true
    protected void end() {
    	CommandBase.driveTrain.DriveTank(0.0, 0.0);
    	requiresDriveTrainSubsystem = false;
    	CommandBase.driveTrain.is_aiming = false;
    	zeroGyro = false;

    	initializeI2CMultiplxerChannelToOpen = false;
    	configureLidar = false;
    	receiveLidarUpperByte = false;
    	startLidarDetectingGearPeg = false;
    	initializeGearPegLidarTimer = false;
    	waitingForGearPeg = false;
    	lidarDetectsGearPeg = false;
    	traveledToTurningRadiusOfRobot = false;
    	configureInitialDistanceBeforeTravelingExtraDistance = false;
    }

    // Called when another command which requires one or more of the same
    // subsystems is scheduled to run
    protected void interrupted() {
    	end();
    }
}
