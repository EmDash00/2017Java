package org.usfirst.frc.team9135.robot.subsystems;

import org.usfirst.frc.team9135.robot.RobotMap;
import edu.wpi.first.wpilibj.command.Subsystem;
import edu.wpi.first.wpilibj.*;
import java.math.*;
import com.ctre.*;
import com.ctre.CANTalon.*;

/**
 *
 */
public class DriveTrain extends Subsystem implements PIDOutput 
{
		// It's desirable that everything possible under private except
		// for methods that implement subsystem capabilities

		private static final int NUM_OF_DRIVE_TRAIN_MOTORS = 4;
		CANTalon[] driveTrainMotors = new CANTalon[NUM_OF_DRIVE_TRAIN_MOTORS];


		//  Drive Train Encoder Variables
		private static final boolean CB_LEFT_ENCODER_SENSOR_DIRECTION = true;
		private static final boolean CB_RIGHT_ENCODER_SENSOR_DIRECTION = false;
		private static final boolean PB_LEFT_ENCODER_SENSOR_DIRECTION = false;
		private static final boolean PB_RIGHT_ENCODER_SENSOR_DIRECTION = false;

		private static final int CB_ENCODER_COUNTS = 256;
		private static final int CB_QUADRATURE_ENCODER_COUNTS = (CB_ENCODER_COUNTS * 4);
		private static final int PB_ENCODER_COUNTS = 256;
		private static final int PB_QUADRATURE_ENCODER_COUNTS = (PB_ENCODER_COUNTS * 4);

		private int quadratureCountOfEncoder = 0;
		private double encoderCountToDistanceConstant = 0;

		private static final double DIAMETER_OF_WHEEL_IN = 4.1;
		private static final double CIRCUMFERENCE_OF_WHEEL = (DIAMETER_OF_WHEEL_IN * Math.PI);

		ADXRS450_Gyro gyro;

		private int encoderValue = 0;
		double distanceTraveled = 0.0;

		private double kToleranceDegrees = .5f;
		private double kP = .14f;
		private double kI = 0;
		private double kD = 0.6f;

		private double kF = 0.0f;

		//  Practice Bot Straight Drive Train
		//  Sensitivity .02
		//  Kp .2

		private double straightDriveTrainSensitivity = 0.0;
		private double straightDriveTrainProportionalConstant = 0.0;

		private static final double PB_STRAIGHT_DRIVE_TRAIN_SENSITIVITY = .05;
		private static final double PB_STRAIGHT_DRIVE_TRAIN_PROPORTIONAL_CONSTANT = .125;
		private static final double CB_STRAIGHT_DRIVE_TRAIN_SENSITIVITY = .07;
		private static final double CB_STRAIGHT_DRIVE_TRAIN_PROPORTIONAL_CONSTANT = .12;

		private double curveValue = 0.0;

		private final FeedbackDeviceStatus UNKNOWN_CONNECTED = FeedbackDeviceStatus.FeedbackStatusUnknown;
		private final FeedbackDeviceStatus RECOGNIZED_CONNECTED = FeedbackDeviceStatus.FeedbackStatusPresent;
		private final FeedbackDeviceStatus DISCONNECTED = FeedbackDeviceStatus.FeedbackStatusNotPresent;

		FeedbackDeviceStatus rightEncoderPluggedIn;
		FeedbackDeviceStatus leftEncoderPluggedIn;

		private boolean rightDriveTrainEncoderPluggedIn = false;
		private boolean leftDriveTrainEncoderPluggedIn = false;

		private static double DRIVE_TRAIN_SPROCKET_RATIO = (15.0/12.0);
		//static  double DRIVE_TRAIN_SPROCKET_RATIO = 1.0;

		//  Variables for Switching the MAX Drive Train Motor Power
		private boolean fastDriveTrainMotorPower = true;
		double desiredMaxMotorPower = 0.0;

		//  Variables for AutoRotateRobot()
		private double currentGyroAngle = 0.0;
		private double initialGyroAngle = 0.0;
		private double differenceBetweenCurrentAndInitialGyroAngle = 0.0;
		private boolean doneAutoRotateRobot = false;

		//  Variables for DriveStraightWithUltrasonicSensor()
		private double differenceBetweenCurrentAndDesiredUltrasonicSensorValue = 0.0;
		private double ultrasonicSensorDriveStraightProportionalConstant = 0.0;
		private double ultrasonicSensorDriveStraightCurveValue = 0.0;
		private double ultrasonicSensorSensitivityValue = 0.0;

		private boolean initializeUltrasonicSensorDriveStraight = false;
		private boolean initializeGyroDriveStraight = false;

		private static final double PB_DRIVE_STRAIGHT_WITH_ULTRASONIC_SENSOR_PROPORTIONAL_CONSTANT = .12;
		private static final double PB_DRIVE_STRAIGHT_WITH_ULTRASONIC_SENSOR_SENSITIVITY_CONSTANT = .17;
		private static final double CB_DRIVE_STRAIGHT_WITH_ULTRASONIC_SENSOR_PROPORTIONAL_CONSTANT = .05;  //  TBD
		private static final double CB_DRIVE_STRAIGHT_WITH_ULTRASONIC_SENSOR_SENSITIVITY_CONSTANT = .17;  //  TBD

		public double initialDistanceTraveled = 0.0;
		public double savedDistanceTraveled = 0.0;
		public double currentDistanceTraveled = 0.0;
		public double differenceBetweenCurrentAndSavedDistanceTraveled = 0.0;
		public double smallPortionOfStraightDistanceTraveled = 0.0;
		public double distanceTraveledStraight = 0.0;
		public double gyroAngleRadians = 0.0;
		public static final double DEGREES_TO_RADIANS_CONSTANT = (Math.PI/180.0);

		public RobotDrive chassis;

		public static final int FRONT_LEFT = 0;
		public static final int REAR_LEFT = 1;
		public static final int FRONT_RIGHT = 2;
		public static final int REAR_RIGHT = 3;

		//  For Competition Bot
		public static final int LEFT_SIDE_ENCODER = REAR_LEFT;
		public static final int RIGHT_SIDE_ENCODER = REAR_RIGHT;

		static final boolean RIGHT_SIDE_ENCODER_booleanEAN = true;
		static final boolean LEFT_SIDE_ENCODER_booleanEAN = !RIGHT_SIDE_ENCODER_booleanEAN;

		public static double FAST_MAX_DRIVE_TRAIN_MOTOR_POWER = 1.0;
		public static double SLOW_MAX_DRIVE_TRAIN_MOTOR_POWER = .75;
		
		public boolean is_aiming = false;
		public double rotateToAngleRate = 0;

		PIDController turnController;


		public void InitializeDriveTrainMotors(boolean competitionBot) {
			if (competitionBot) {
				driveTrainMotors[FRONT_LEFT] = new CANTalon(RobotMap.CB_FRONT_LEFT_TALON_ID);
				driveTrainMotors[REAR_LEFT] = new CANTalon(RobotMap.CB_REAR_LEFT_TALON_ID);
				driveTrainMotors[FRONT_RIGHT] = new CANTalon(RobotMap.CB_FRONT_RIGHT_TALON_ID);
				driveTrainMotors[REAR_RIGHT] = new CANTalon(RobotMap.CB_REAR_RIGHT_TALON_ID);
			}
			else if (competitionBot == false) {
				driveTrainMotors[FRONT_LEFT] = new CANTalon(RobotMap.PB_FRONT_LEFT_TALON_ID);
				driveTrainMotors[REAR_LEFT] = new CANTalon(RobotMap.PB_REAR_LEFT_TALON_ID);
				driveTrainMotors[FRONT_RIGHT] = new CANTalon(RobotMap.PB_FRONT_RIGHT_TALON_ID);
				driveTrainMotors[REAR_RIGHT] = new CANTalon(RobotMap.PB_REAR_RIGHT_TALON_ID);
			}

			for (int i = 0; i < NUM_OF_DRIVE_TRAIN_MOTORS; i++) {
				driveTrainMotors[i].enableBrakeMode(true);
			}

			chassis = new RobotDrive(driveTrainMotors[FRONT_LEFT], driveTrainMotors[REAR_LEFT], driveTrainMotors[FRONT_RIGHT], driveTrainMotors[REAR_RIGHT]);
			chassis.setSafetyEnabled(false);
		}

		public void DriveTank(double leftMotorPower, double rightMotorPower) {
			chassis.tankDrive(leftMotorPower, rightMotorPower);
		}

		public void RotateTank(double motorPower, boolean turnRight) {
			if (turnRight) {
				this.DriveTank(motorPower, -motorPower);
			}
			else if (turnRight == false) {
				this.DriveTank(-motorPower, motorPower);
			}
		}

		public void SwitchBetweenFastAndSlowDriveTrainMotorPower(boolean fastDriveTrainMotorPower) {
			this.fastDriveTrainMotorPower = fastDriveTrainMotorPower;
		}

		public double getDesiredDriveTrainMaxMotorPower() {
			if (this.fastDriveTrainMotorPower) {
				desiredMaxMotorPower = FAST_MAX_DRIVE_TRAIN_MOTOR_POWER;
			}
			else if (this.fastDriveTrainMotorPower == false) {
				desiredMaxMotorPower = SLOW_MAX_DRIVE_TRAIN_MOTOR_POWER;
			}
			return desiredMaxMotorPower;
		}

		void configureDriveTrainEncoders(boolean competitionBot) {
			if (competitionBot) {
				driveTrainMotors[LEFT_SIDE_ENCODER].configEncoderCodesPerRev(CB_ENCODER_COUNTS);
				driveTrainMotors[RIGHT_SIDE_ENCODER].configEncoderCodesPerRev(CB_ENCODER_COUNTS);

				driveTrainMotors[LEFT_SIDE_ENCODER].reverseSensor(CB_LEFT_ENCODER_SENSOR_DIRECTION);
				driveTrainMotors[RIGHT_SIDE_ENCODER].reverseSensor(CB_RIGHT_ENCODER_SENSOR_DIRECTION);

				quadratureCountOfEncoder = (CB_ENCODER_COUNTS * 4);
			}
			else if (competitionBot == false) {
				driveTrainMotors[LEFT_SIDE_ENCODER].configEncoderCodesPerRev(PB_ENCODER_COUNTS);
				driveTrainMotors[RIGHT_SIDE_ENCODER].configEncoderCodesPerRev(PB_ENCODER_COUNTS);

				driveTrainMotors[LEFT_SIDE_ENCODER].reverseSensor(PB_LEFT_ENCODER_SENSOR_DIRECTION);
				driveTrainMotors[RIGHT_SIDE_ENCODER].reverseSensor(PB_RIGHT_ENCODER_SENSOR_DIRECTION);

				quadratureCountOfEncoder = (PB_ENCODER_COUNTS * 4);
			}

			encoderCountToDistanceConstant = (CIRCUMFERENCE_OF_WHEEL/((double)quadratureCountOfEncoder));

			driveTrainMotors[LEFT_SIDE_ENCODER].setFeedbackDevice(FeedbackDevice.QuadEncoder);
			driveTrainMotors[RIGHT_SIDE_ENCODER].setFeedbackDevice(FeedbackDevice.QuadEncoder);

			driveTrainMotors[LEFT_SIDE_ENCODER].setStatusFrameRateMs(StatusFrameRate.QuadEncoder, 10);
			driveTrainMotors[RIGHT_SIDE_ENCODER].setStatusFrameRateMs(StatusFrameRate.QuadEncoder, 10);
		}

		public boolean MakeSureDriveTrainEncoderisPluggedIn(boolean rightLeftSideEncoder) {
			if (RIGHT_SIDE_ENCODER_booleanEAN) {
				rightEncoderPluggedIn = driveTrainMotors[RIGHT_SIDE_ENCODER].isSensorPresent(FeedbackDevice.QuadEncoder);
				if (rightEncoderPluggedIn == UNKNOWN_CONNECTED || rightEncoderPluggedIn == RECOGNIZED_CONNECTED) {
					rightDriveTrainEncoderPluggedIn = true;
				}
				else if (rightEncoderPluggedIn == DISCONNECTED) {
					rightDriveTrainEncoderPluggedIn = false;
				}
				return rightDriveTrainEncoderPluggedIn;
			}
			else if (LEFT_SIDE_ENCODER_booleanEAN) {
				leftEncoderPluggedIn = driveTrainMotors[LEFT_SIDE_ENCODER].isSensorPresent(FeedbackDevice.QuadEncoder);
				if (leftEncoderPluggedIn == UNKNOWN_CONNECTED || leftEncoderPluggedIn == RECOGNIZED_CONNECTED) {
					leftDriveTrainEncoderPluggedIn = true;
				}
				else if (leftEncoderPluggedIn == DISCONNECTED) {
					leftDriveTrainEncoderPluggedIn = false;
				}
				return leftDriveTrainEncoderPluggedIn;
			}
			
			return false;
		}

		public void ZeroDriveTrainEncoder(int motorEncoderPort) {
			driveTrainMotors[motorEncoderPort].setEncPosition(0);
		}

		public int getEncoderPosition(int motorEncoderPort) {
			if (motorEncoderPort == LEFT_SIDE_ENCODER) {
				return (-1 * driveTrainMotors[motorEncoderPort].getEncPosition());
			}
			else {
				return driveTrainMotors[motorEncoderPort].getEncPosition();
			}
		}

		public double getDistance(int motorEncoderPort) {
			encoderValue = this.getEncoderPosition(motorEncoderPort);
			distanceTraveled = (encoderValue * encoderCountToDistanceConstant);
			return (distanceTraveled * DRIVE_TRAIN_SPROCKET_RATIO);
		}

		public double getStraightDistanceTraveled(int motorEncoderPort, double gyroAngleDegrees, boolean configureInitialDistanceTraveled, boolean drivingForwards) {
			if (configureInitialDistanceTraveled) {
				savedDistanceTraveled = this.getDistance(motorEncoderPort);
				distanceTraveledStraight = 0.0;
			}
			else if (configureInitialDistanceTraveled == false) {
				currentDistanceTraveled = this.getDistance(motorEncoderPort);
				if (drivingForwards) {
					differenceBetweenCurrentAndSavedDistanceTraveled = (currentDistanceTraveled - savedDistanceTraveled);
				}
				else if (drivingForwards == false) {
					differenceBetweenCurrentAndSavedDistanceTraveled = (savedDistanceTraveled - currentDistanceTraveled);
				}
				gyroAngleRadians = (gyroAngleDegrees * DEGREES_TO_RADIANS_CONSTANT);
				smallPortionOfStraightDistanceTraveled = (differenceBetweenCurrentAndSavedDistanceTraveled * (Math.cos(gyroAngleRadians)));
				distanceTraveledStraight += smallPortionOfStraightDistanceTraveled;
				savedDistanceTraveled = currentDistanceTraveled;
			}
			return distanceTraveledStraight;
		}

		public int getEncoderRPM(int motorEncoderPort) {
			return (int)driveTrainMotors[motorEncoderPort].getSpeed();
		}

		public double getTalonOutputCurrent(int motorArray) {
			return driveTrainMotors[motorArray].getOutputCurrent();
		}

		public void InitializeDriveStraightWithGyro(boolean competitionBot) {
			if (competitionBot) {
				straightDriveTrainSensitivity = CB_STRAIGHT_DRIVE_TRAIN_SENSITIVITY;
				straightDriveTrainProportionalConstant = CB_STRAIGHT_DRIVE_TRAIN_PROPORTIONAL_CONSTANT;
				ultrasonicSensorDriveStraightProportionalConstant = CB_DRIVE_STRAIGHT_WITH_ULTRASONIC_SENSOR_PROPORTIONAL_CONSTANT;
				ultrasonicSensorSensitivityValue = CB_DRIVE_STRAIGHT_WITH_ULTRASONIC_SENSOR_SENSITIVITY_CONSTANT;
			}
			else if (competitionBot == false) {
				straightDriveTrainSensitivity = .05;  //. 12  Low Battery
				straightDriveTrainProportionalConstant = .125;  //  .04  Low Battery
				ultrasonicSensorDriveStraightProportionalConstant = .15;  //  .12 ///////////////////
				ultrasonicSensorSensitivityValue = .23;  //  .17 /////////////////////////
			}
			chassis.setSensitivity(straightDriveTrainSensitivity);
			initializeGyroDriveStraight = true;
			initializeUltrasonicSensorDriveStraight = false;
		}

		public void ChangeDriveStraightSensitivity(double sensitivity) {
			chassis.setSensitivity(sensitivity);
		}

		public void DriveStraightWithGyro(double motorPower, double gyroAngle) {
			if (initializeGyroDriveStraight == false) {
				this.ChangeDriveStraightSensitivity(straightDriveTrainSensitivity);
				initializeGyroDriveStraight = true;
				initializeUltrasonicSensorDriveStraight = false;
			}
			if (motorPower > 0.0) {
				curveValue = (-1 * gyroAngle * straightDriveTrainProportionalConstant);
			}
			else if (motorPower < 0.0) {
				curveValue = (gyroAngle * straightDriveTrainProportionalConstant);
			}

			if (curveValue > 1.0) {
				curveValue = 1.0;
			}
			else if (curveValue < -1.0) {
				curveValue = -1.0;
			}

			chassis.drive(motorPower, curveValue);
		}

		public void DriveStraightWithUltrasonicSensor(double currentUltrasonicSensorValue, double desiredDistanceFromGuardrail, double motorPower, boolean rightSideHopperAndShoot) {
			if (initializeUltrasonicSensorDriveStraight == false) {
				this.ChangeDriveStraightSensitivity(ultrasonicSensorSensitivityValue);
				initializeUltrasonicSensorDriveStraight = true;
				initializeGyroDriveStraight = false;
			}
			differenceBetweenCurrentAndDesiredUltrasonicSensorValue = (currentUltrasonicSensorValue - desiredDistanceFromGuardrail);
			if (rightSideHopperAndShoot) {
				ultrasonicSensorDriveStraightCurveValue = (-1.0 * ultrasonicSensorDriveStraightProportionalConstant * differenceBetweenCurrentAndDesiredUltrasonicSensorValue);
			}
			else if (rightSideHopperAndShoot == false) {
				ultrasonicSensorDriveStraightCurveValue = (ultrasonicSensorDriveStraightProportionalConstant * differenceBetweenCurrentAndDesiredUltrasonicSensorValue);
			}

			if (ultrasonicSensorDriveStraightCurveValue > 1.0) {
				ultrasonicSensorDriveStraightCurveValue = 1.0;
			}
			else if (ultrasonicSensorDriveStraightCurveValue < -1.0) {
				ultrasonicSensorDriveStraightCurveValue = -1.0;
			}

			chassis.drive(motorPower, ultrasonicSensorDriveStraightCurveValue);

			//System.out.println("Curve Value: " + ultrasonicSensorDriveStraightCurveValue);
		}

		public void InitializeDriveTrainPID() {
			gyro = new ADXRS450_Gyro(); //maybe?
			gyro.calibrate();
			turnController = new PIDController(Preferences.getInstance().getDouble("kP",0.0625), Preferences.getInstance().getDouble("kI",0.0),Preferences.getInstance().getDouble("kD",0.0) , kF, gyro, this);
			//turnController = new PIDController(kP,kI,kD,kF, gyro, this);
			turnController.setInputRange(-90.0f,  90.0f);
			turnController.setOutputRange(-1.0, 1.0);
			turnController.setAbsoluteTolerance(kToleranceDegrees);
			turnController.setContinuous(true);
			turnController.disable();
		}

		public double getGyroAngle() {
			return gyro.getAngle();
		}

		public void ZeroGyroAngle() {
			gyro.reset();
		}

		public void TurnPIDEnable(double angleToTurn)
		{
			ZeroGyroAngle();
			//std.cout +"navx angle: " + gyro.getAngle() + "\n";
			turnController.setSetpoint(angleToTurn);
			turnController.enable();
		}

		public void TurnPIDDisable()
		{
			turnController.disable();
		}

		public void PIDTurning()
		{
			//System.out.println("navx angle: " + gyro.getAngle();
			//System.out.println("rot rate: " + rotateToAngleRate + "\n";
			this.RotateTank(rotateToAngleRate, true);
		}

		public boolean AutoRotateRobot(double motorPower, double desiredGyroAngleToTurn, boolean turnRight, boolean initializeAutoRotateRobot) {
			if (initializeAutoRotateRobot == false) {
				initialGyroAngle = this.getGyroAngle();
				System.out.println("Initial Gyro Angle: " + initialGyroAngle);
				doneAutoRotateRobot = false;
				initializeAutoRotateRobot = true;
			}

			currentGyroAngle = this.getGyroAngle();
			System.out.println("Current Gyro Angle" + currentGyroAngle);
			differenceBetweenCurrentAndInitialGyroAngle = (Math.abs(currentGyroAngle - initialGyroAngle));
			System.out.println("Difference: " + differenceBetweenCurrentAndInitialGyroAngle);
			System.out.println("Desired Gyro ANgleee: " + desiredGyroAngleToTurn);

			if (initializeAutoRotateRobot) {
				if (differenceBetweenCurrentAndInitialGyroAngle >= desiredGyroAngleToTurn) {
					this.RotateTank(0.0, turnRight);
					initializeAutoRotateRobot = false;
					doneAutoRotateRobot = true;
					System.out.println("Done");
				}
				else {
					this.RotateTank(motorPower, turnRight);
				}
			}
			return doneAutoRotateRobot;
		}

    // Put methods for controlling this subsystem
    // here. Call these from Commands.

    public void initDefaultCommand() {
        // set the default command for a subsystem here.
        //setDefaultCommand(new MySpecialCommand());
    }

	@Override
	public void pidWrite(double output) {
		// TODO Auto-generated method stub
		
	}
}

