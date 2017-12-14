package org.usfirst.frc.team9135.robot.subsystems;

import org.usfirst.frc.team9135.robot.commands.ReadUltrasonicSensorValue;

import edu.wpi.first.wpilibj.*;
import edu.wpi.first.wpilibj.command.Subsystem;

/**
 *
 */


public class UltrasonicSensor extends Subsystem {

	private Ultrasonic gearUltrasonicSensor;
	private DigitalOutput gearUltrasonicPingSignal;
	private DigitalInput gearUltrasonicEchoSignal;

	private static final int GEAR_ULTRASONIC_DIO_PING_PORT = 9;
	private static final int GEAR_ULTRASONIC_DIO_ECHO_PORT = 8;

	private double sideUltrasonicSensorValue = 0.0;

	private double gyroAngleRadians = 0.0;
	private double actualDistanceFromGuardrail = 0.0;
	private static final double DEGREES_TO_RADIANS_CONSTANT = (Math.PI/180.0);
	

	public double differenceBetweenFrontAndBackUltrasonicSensorValues = 0.0;
	public static final double DISTANCE_BETWEEN_FRONT_AND_BACK_ULTRASONIC_SENSORS = 28.0;
	public static final double DISTANCE_WIDTH_WISE_FRONT_AND_BACK_ULTRASONIC_SENSORS_ARE_APART = 1.5;
	public double convertedBackUltrasonicSensorValue = 0.0;
	public double desiredAngleToTurnRadians = 0.0;
	public static final double RADIANS_TO_DERGREES_CONSTANT = (180.0/Math.PI);
	public double desiredAngleToTurnDegrees = 0.0;

	public static final boolean  RIGHT_SIDE_ULTRASONIC_SENSOR = true;
	public static final boolean  LEFT_SIDE_ULTRASONIC_SENSOR = !RIGHT_SIDE_ULTRASONIC_SENSOR;

	public boolean  usingRightUltrasonicSensorForGearCamera = false;

	public boolean  usingUltrasonicSensor = false;
    // Put methods for controlling this subsystem
    // here. Call these from Commands.

    protected void initDefaultCommand() {
    	// Set the default command for a subsystem here.
    	// SetDefaultCommand(new MySpecialCommand());
    	setDefaultCommand(new ReadUltrasonicSensorValue());
    }

    void InitializeUltrasonicSensors() {
    	gearUltrasonicPingSignal = new DigitalOutput(GEAR_ULTRASONIC_DIO_PING_PORT);
    	gearUltrasonicEchoSignal = new DigitalInput(GEAR_ULTRASONIC_DIO_ECHO_PORT);
    	gearUltrasonicSensor = new Ultrasonic(gearUltrasonicPingSignal, gearUltrasonicEchoSignal, Ultrasonic.Unit.kInches);
    	gearUltrasonicSensor.setAutomaticMode(true);

    	/*rightSideUltrasonicSensorPingSignal = new DigitalOutput(RIGHT_SIDE_ULTRASONIC_SENSOR_PING_SIGNAL_PORT);
    	rightSideUltrasonicSensorEchoSignal = new DigitalInput(RIGHT_SIDE_ULTRASONIC_SENSOR_ECHO_SIGNAL_PORT);
    	rightSideUltrasonicSensor = new Ultrasonic(rightSideUltrasonicSensorPingSignal, rightSideUltrasonicSensorEchoSignal, Ultrasonic.DistanceUnit.kInches);
    	rightSideSetAutomaticMode(true);

    	leftSideUltrasonicSensorPingSignal = new DigitalOutput(LEFT_SIDE_ULTRASONIC_SENSOR_PING_SIGNAL_PORT);
    	leftSideUltrasonicSensorEchoSignal = new DigitalInput(LEFT_SIDE_ULTRASONIC_SENSOR_ECHO_SIGNAL_PORT);
    	leftSideUltrasonicSensor = new Ultrasonic(leftSideUltrasonicSensorPingSignal, leftSideUltrasonicSensorEchoSignal, Ultrasonic.DistanceUnit.kInches);
    	leftSideSetAutomaticMode(true); */
    }

    void PingSideUltrasonicSensor(boolean rightUltrasonic) {
    	if (rightUltrasonic) {
    		//rightSidePing();
    	}
    	else if (rightUltrasonic == false) {
    		//leftSidePing();
    	}
    }

    void PingGearUltrasonicSensor() {
    	gearUltrasonicSensor.ping();
    }

    double GetGearUltrasonicSensorValueInches() {
    	return gearUltrasonicSensor.getRangeInches();
    }

    double GetSideUltrasonicSensorValueInches(boolean sideUltrasonicSensor) {
    	if (sideUltrasonicSensor == RIGHT_SIDE_ULTRASONIC_SENSOR) {
    		//sideUltrasonicSensorValue = rightSideGetRangeInches();
    	}
    	else if (sideUltrasonicSensor == LEFT_SIDE_ULTRASONIC_SENSOR) {
    		//sideUltrasonicSensorValue = leftSideGetRangeInches();
    	}
    	return sideUltrasonicSensorValue;
    }

    double GetActualDistanceFromGuardrail(double ultrasonicSensorValue, double gyroAngle) {
    	gyroAngleRadians = (gyroAngle * DEGREES_TO_RADIANS_CONSTANT);
    	actualDistanceFromGuardrail = (Math.cos(gyroAngleRadians) * ultrasonicSensorValue);
    	return actualDistanceFromGuardrail;
    }

    double GetAngleToTurnToAlignWithGuardRail(double frontUltrasonicSensorValue, double backUltrasonicSensorValue, boolean rightHopper) {
    	//std.cout << "Front Ultrasonic Sensor Value: " << frontUltrasonicSensorValue << std.endl;
    	//std.cout << "Back Ultrasonic Sensor Value: " << backUltrasonicSensorValue << std.endl;
    	convertedBackUltrasonicSensorValue = (backUltrasonicSensorValue + DISTANCE_WIDTH_WISE_FRONT_AND_BACK_ULTRASONIC_SENSORS_ARE_APART);
    	if (rightHopper) {
    		differenceBetweenFrontAndBackUltrasonicSensorValues = (backUltrasonicSensorValue - frontUltrasonicSensorValue);
    	}
    	else if (rightHopper == false) {
    		differenceBetweenFrontAndBackUltrasonicSensorValues = (frontUltrasonicSensorValue - backUltrasonicSensorValue);
    	}
    	//std.cout << "Difference Between Two Ultrasonic Sensor Values: " << differenceBetweenFrontAndBackUltrasonicSensorValues << std.endl;
    	desiredAngleToTurnRadians = Math.atan2(differenceBetweenFrontAndBackUltrasonicSensorValues, DISTANCE_BETWEEN_FRONT_AND_BACK_ULTRASONIC_SENSORS);
    	//std.cout << "Angle To Turn Radians: " << desiredAngleToTurnRadians << std.endl;
    	desiredAngleToTurnDegrees = (desiredAngleToTurnRadians * RADIANS_TO_DERGREES_CONSTANT);
    	//std.cout << "Desired Angle To Turn Degrees: " << desiredAngleToTurnDegrees << std.endl;
    	return desiredAngleToTurnDegrees;
    }

    // Put methods for controlling this subsystem
    // here. Call these from Commands.

}

