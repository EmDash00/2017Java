package org.usfirst.frc.team9135.robot.commands;

import org.usfirst.frc.team9135.robot.CommandBase;

import org.usfirst.frc.team9135.robot.subsystems.*;

import edu.wpi.first.wpilibj.command.Command;

/**
 *
 */
public class DriveParallelWithGuardRailWithUltrasonic extends Command {

	private double desiredDistanceToTravel;
	private double distanceAwayFromGuardrailToDrive;
	private double driveTrainMotorPower;
	private boolean rightHopperAndShoot;

	private boolean initializeIsAimingBoolean = false;

	private double sideUltrasonicSensorValue = 0.0;

	private boolean initializeEncoderDriveDistance = false;
	private double initialDistanceTraveled = 0.0;
	private double currentDistanceTraveled = 0.0;
	private double differenceBetweenInitialAndCurrentDistance = 0.0;

	private boolean doneWithDrivingWithUltrasonicSensor = false;

	private double acutalDistanceRobotIsFromWall = 0.0;

	private boolean zeroGyro = false;
	private double gyroAngle = 0.0;

	private static final boolean CONFIGURE_INITIAL_DISTANCE = true;
	private static final boolean DONT_RECONFIGURE_INITIAL_DISTANCE = !CONFIGURE_INITIAL_DISTANCE;
	private static final boolean DRIVING_FORWARDS = true;
	private static final boolean DRIVING_BACKWARDS = false;

	private static final double THRESHOLD_FOR_ADJUSTING_DRIVE_STRAIGHT_WITH_ULTRSONIC_SENSOR = .8;

	private static final double THRESHOLD_GYRO_ANGLE_FOR_ADJUSTING = 1.8;

	private boolean initializeInitialGyroAngle = false;
	private double initialGyroAngleForAdjusting = 0.0;
	private boolean adjustRobotWithGyro = false;

	private boolean getInitialUltrasonicSensorValue = 0.0;
	private double initialUltrasonicSensorValue = 0.0;

	private boolean initializeDistanceToTravel = false;
	private double desiredDistanceToTravelFromLidar = 0.0;

	private double frontUltrasonicSensorValue = 0.0;
	
	  public DriveParallelWithGuardRailWithUltrasonic() {
	        // Use requires() here to declare subsystem dependencies
	        // eg. requires(chassis);
		  	requires(CommandBase.driveTrain);
			requires(CommandBase.ultrasonicSensor);
			requires(CommandBase.lidars);

			this.desiredDistanceToTravel = desiredDistanceToTravel;
			this.distanceAwayFromGuardrailToDrive = distanceAwayFromGuardrailToDrive;
			this.driveTrainMotorPower = driveTrainMotorPower;
			this.rightHopperAndShoot = rightHopperAndShoot;
	    }

	    // Called just before this Command runs the first time
	    protected void initialize() {
	    	initialDistanceTraveled = CommandBase.driveTrain.getDistance(DriveTrain.RIGHT_SIDE_ENCODER);
	    	initializeEncoderDriveDistance = true;

	    	doneWithDrivingWithUltrasonicSensor = false;

	    	CommandBase.driveTrain.is_aiming = true;
	    	CommandBase.ultrasonicSensor.usingUltrasonicSensor = true;
	    	initializeIsAimingBoolean = true;

	    	CommandBase.driveTrain.ZeroGyroAngle();
	    	zeroGyro = true;

	    	initializeInitialGyroAngle = false;

	    	adjustRobotWithGyro = false;

	    	initialUltrasonicSensorValue = CommandBase.ultrasonicSensor.GetSideUltrasonicSensorValueInches(UltrasonicSensor::LEFT_SIDE_ULTRASONIC_SENSOR);
	    	getInitialUltrasonicSensorValue = true;

	    	initializeDistanceToTravel = false;
	    }

	    // Called repeatedly when this Command is scheduled to run
	    protected void execute() {
	    	if (initializeDistanceToTravel == false) {
	    		frontUltrasonicSensorValue = CommandBase.ultrasonicSensor.GetGearUltrasonicSensorValueInches();
	    		desiredDistanceToTravelFromLidar = CommandBase.lidars.GetDistanceToTravelToHopper(frontUltrasonicSensorValue);
	    		initializeDistanceToTravel = true;
	    	}

	    	if (getInitialUltrasonicSensorValue == false) {
	    		initialUltrasonicSensorValue = CommandBase.ultrasonicSensor.GetSideUltrasonicSensorValueInches(UltrasonicSensor::LEFT_SIDE_ULTRASONIC_SENSOR);
	    		getInitialUltrasonicSensorValue = true;
	    	}

	    	if (initializeEncoderDriveDistance == false) {
	    		initialDistanceTraveled = CommandBase.driveTrain.getDistance(DriveTrain.RIGHT_SIDE_ENCODER);
	    		initializeEncoderDriveDistance = true;
	    	}

	    	if (initializeIsAimingBoolean == false) {
	    		CommandBase.driveTrain.is_aiming = true;
	    		CommandBase.ultrasonicSensor.usingUltrasonicSensor = true;
	    		initializeIsAimingBoolean = true;
	    	}

	    	if (zeroGyro == false) {
	    		CommandBase.driveTrain.ZeroGyroAngle();
	    		zeroGyro = true;
	    	}

	    	if (this.rightHopperAndShoot) {
	    		sideUltrasonicSensorValue = CommandBase.ultrasonicSensor.GetSideUltrasonicSensorValueInches(UltrasonicSensor::LEFT_SIDE_ULTRASONIC_SENSOR);
	    	}
	    	else if (this.rightHopperAndShoot == false) {
	    		sideUltrasonicSensorValue = CommandBase.ultrasonicSensor.GetSideUltrasonicSensorValueInches(UltrasonicSensor::RIGHT_SIDE_ULTRASONIC_SENSOR);
	    	}

	    	gyroAngle = CommandBase.driveTrain.getGyroAngle();
	    	acutalDistanceRobotIsFromWall = CommandBase.ultrasonicSensor.GetActualDistanceFromGuardrail(sideUltrasonicSensorValue, gyroAngle);

	    	currentDistanceTraveled = CommandBase.driveTrain.getDistance(DriveTrain.RIGHT_SIDE_ENCODER);
	    	differenceBetweenInitialAndCurrentDistance = Math.abs((currentDistanceTraveled - initialDistanceTraveled));
	    	if (differenceBetweenInitialAndCurrentDistance >= desiredDistanceToTravelFromLidar) {
	    		CommandBase.driveTrain.DriveTank(0.0, 0.0);
	    		doneWithDrivingWithUltrasonicSensor = true;
	    	}
	    	else if (differenceBetweenInitialAndCurrentDistance < desiredDistanceToTravelFromLidar) {
	    		if ((((Math.abs(acutalDistanceRobotIsFromWall - initialUltrasonicSensorValue)) < THRESHOLD_FOR_ADJUSTING_DRIVE_STRAIGHT_WITH_ULTRSONIC_SENSOR) && ((Math.abs(gyroAngle)) > THRESHOLD_GYRO_ANGLE_FOR_ADJUSTING)) || adjustRobotWithGyro) {
	    			System.out.println("Adjusting");
	    			if (initializeInitialGyroAngle == false) {
	    				initialGyroAngleForAdjusting = gyroAngle;
	    				initializeInitialGyroAngle = true;
	    				adjustRobotWithGyro = true;
	    			}

	    			gyroAngle = CommandBase.driveTrain.getGyroAngle();

	    			if (initialGyroAngleForAdjusting < 0) {
	    				//  Move Right Side Backwards
	    				if (gyroAngle >= 0) {
	    					adjustRobotWithGyro = false;
	    				}
	    				else {
	    					CommandBase.driveTrain.DriveTank(0.0, (this.driveTrainMotorPower - .25));
	    				}
	    			}
	    			else if (initialGyroAngleForAdjusting > 0) {
	    				//  Move Left Side Backwards
	    				if (gyroAngle <= 0) {
	    					adjustRobotWithGyro = false;
	    				}
	    				else {
	    					CommandBase.driveTrain.DriveTank((this.driveTrainMotorPower - .25), 0.0);
	    				}
	    			}
	    		}
	    		else {
	    			System.out.println("Tracking");
	    			CommandBase.driveTrain.DriveStraightWithUltrasonicSensor(acutalDistanceRobotIsFromWall, initialUltrasonicSensorValue, this.driveTrainMotorPower, this.rightHopperAndShoot);
	    		}
	    	}

	    	System.out.println("Gyro ANgle: " + gyroAngle);
	    	System.out.println("Ultrasonic Sensor Cos Value: " + acutalDistanceRobotIsFromWall);

	    }

	    // Make this return true when this Command no longer needs to run execute()
	    protected boolean isFinished() {
	    	return doneWithDrivingWithUltrasonicSensor;
	    }

	    // Called once after isFinished returns true
	    protected void end() {
	    	CommandBase.driveTrain.DriveTank(0.0, 0.0);
	    	CommandBase.driveTrain.is_aiming = false;
	    	CommandBase.ultrasonicSensor.usingUltrasonicSensor = false;
	    	zeroGyro = false;
	    	initializeIsAimingBoolean = false;
	    	initializeEncoderDriveDistance = false;
	    	getInitialUltrasonicSensorValue = false;
	    	initializeDistanceToTravel = false;
	    	doneWithDrivingWithUltrasonicSensor = false;

	    }

	    // Called when another command which requires one or more of the same
	    // subsystems is scheduled to run
	    protected void interrupted() {
	    }
  
}
