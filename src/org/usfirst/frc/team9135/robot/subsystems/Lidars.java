package org.usfirst.frc.team9135.robot.subsystems;

import org.usfirst.frc.team9135.robot.RobotMap;

import edu.wpi.first.wpilibj.command.Subsystem;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.*;


public class Lidars extends Subsystem 
{
	
		// It's desirable that everything possible under private except
		// for methods that implement subsystem capabilities

		private final I2C.Port MXP_PORT = I2C.Port.kMXP;
		private final I2C.Port I2C_PORT = I2C.Port.kOnboard;

		//  Lidar Pointer and Variables
		private I2C lidar;

		private static final int LIDAR_DEVICE_ADDRESS = 0x62;

		private static final int CONFIGURE_REGISTER_ADDRESS = 0x00;
		private static final int CONFIGURE_VALUE_TO_WRITE = 0x04;

		private static final int UPPER_BYTE_REGISTER_ADDRESS = 0x0f;
		private static final int LOWER_BYTE_REGISTER_ADDRESS = 0x10;

		private static final int BOTH_BYTE_REGISTER_ADDRESS = 0x8f;

		private static final int POWER_CONSUMPTION_MODE_AFTER_READING_VALUES_ADDRESS = 0x04;
		private static final int VALUE_TO_SEND_TO_TURN_OFF_DETECTOR_BIAS_BETWEEN_ACQUISITIONS = 0b00001000;

		private byte[] upperByteDataPointer;
		private byte[] lowerByteDataPointer;

		private int convertedUpperByteData = 0;
		private int convertedLowerByteData = 0;

		private int shiftedUpperByte = 0;

		private int finalValue = 0;
		private int convertedFinalValue = 0;

		private char charPointerConvertedData = 0;
		private char charPointerData = 0;
		private int finalIntConvertedData = 0;


		//  I2C Multiplexer Pointer and Variables
		private I2C i2CMultiplexer;

		private static final int I2C_MULTIPLEXER_DEVICE_ADDRESS = 0x70;

		
		private byte[] convertedValueToSendToI2CMultiplexer;
		private byte[] convertedByte;

		private int differenceBetweenLIDARValues = 0;
		private double angleBetweenLIDARValues = 0.0;
		private double extraDistanceOfLIDAR_M = 0.0;

		private int finalNonZeroLidarValue = 0;

		private double returnLidarValue = 0.0;

		private double savedLidarValue = 0.0;

		private DigitalOutput lidarPowerEnabledDO;
		private static final int LIDAR_POWER_ENABLE_DIGITAL_OUTPUT_PORT = 16;
		boolean lidarTurnedOn = false;

		private double storedLidarValue = 0.0;
		private double distanceFromBumperToGuardrail = 0.0;
		private static double DISTANCE_FROM_FRONT_ULTRASONIC_SENSOR_TO_BUMPER = 2.75;
		private static double START_OF_DESIRED_LIDAR_VALUE = 35.5;  //  37 //  35
		private double desiredLidarValue = 0.0;
		private double desiredDistanceToTravelToHopper = 0.0;

		public static final int NUM_OF_UNITS = 3;
		public static final int CENTIMETERS = 0;
		public static final int INCHES = 1;
		public static final int METERS = 2;

		public static final int[] DISTANCE_UNIT_ARRAY = {CENTIMETERS, INCHES, METERS};

		public static final byte VALUE_TO_OPEN_LIDAR_CHANNEL_6_RIGHT_LIDAR = (byte)0b01000000;
		public static final byte VALUE_TO_OPEN_LIDAR_CHANNEL_7_LEFT_LIDAR = (byte)0b10000000;
		public static final byte VALUE_TO_OPEN_FRONT_LIDAR_CHANNEL_7 = (byte)0b10000000;
		public static final byte VALUE_TO_OPEN_FRONT_LIDAR_CHANNEL_6 = (byte)0b01000000;

		public static final boolean TURN_LIDAR_ON = true;
		public static final boolean TURN_LIDAR_OFF = !TURN_LIDAR_ON;

    // Put methods for controlling this subsystem
    // here. Call these from Commands.


		public void InitializeLidarsAndI2CMultiplexer() {
			lidar = new I2C(MXP_PORT, LIDAR_DEVICE_ADDRESS);

			upperByteDataPointer = new byte[1];
			lowerByteDataPointer = new byte[1];

			char PointerConvertedData;

			i2CMultiplexer = new I2C(MXP_PORT, I2C_MULTIPLEXER_DEVICE_ADDRESS);

			convertedValueToSendToI2CMultiplexer = new byte[1];
		}

		public void InitializeLidarPowerEnablePin() {
			lidarPowerEnabledDO = new DigitalOutput(LIDAR_POWER_ENABLE_DIGITAL_OUTPUT_PORT);
		}

		public void TurnLidarOnOff(boolean turnOn) {
			lidarPowerEnabledDO.set(turnOn);
			lidarTurnedOn = turnOn;
		}

		public void OpenLidarChannelOnMultiplexer(int byteToSend) {
			convertedValueToSendToI2CMultiplexer[0] = (byte) byteToSend;
			i2CMultiplexer.writeBulk(convertedValueToSendToI2CMultiplexer);
		}

		public void ConfigureLidar() {
			lidar.write(CONFIGURE_REGISTER_ADDRESS, CONFIGURE_VALUE_TO_WRITE);
		}

		public void TurnOffDetectorBiasBetweenLidarAcquisitions() {
			lidar.write(POWER_CONSUMPTION_MODE_AFTER_READING_VALUES_ADDRESS, VALUE_TO_SEND_TO_TURN_OFF_DETECTOR_BIAS_BETWEEN_ACQUISITIONS);
		}

		
		
		
		
		
		
		
		
		//Get lidar values		
		
		public int GetUpperByte() {
			lidar.read(UPPER_BYTE_REGISTER_ADDRESS, 1, upperByteDataPointer);
			convertedUpperByteData = upperByteDataPointer[0];
			return convertedUpperByteData;
			
		}

		public int GetLowerByte() {
			lidar.read(LOWER_BYTE_REGISTER_ADDRESS, 1, lowerByteDataPointer);
			convertedLowerByteData = lowerByteDataPointer[0];
			return convertedLowerByteData;
		}

		public double GetLidarValue(int lowerByte, int upperByte, int distanceUnits) {
			shiftedUpperByte = upperByte << 8;
			finalValue = (shiftedUpperByte + lowerByte);

			SmartDashboard.putNumber("Actual LIDAR Value:", finalValue);

			if (distanceUnits == DISTANCE_UNIT_ARRAY[CENTIMETERS]) {
				returnLidarValue = ((double)finalValue);
			}
			else if (distanceUnits == DISTANCE_UNIT_ARRAY[INCHES]) {
				returnLidarValue = this.ConvertCentimetersToInches(finalValue);
				if (returnLidarValue > 200.0) {
					returnLidarValue = 0.0;
				}
			}
			else if (distanceUnits == DISTANCE_UNIT_ARRAY[METERS]) {
				returnLidarValue = this.ConvertCentimetersToMeters(finalValue);
			}

			if (returnLidarValue != 0) {
				savedLidarValue = returnLidarValue;
			}

			return savedLidarValue;
		}

		
		
		
		
		
		
		
		public double ConvertCentimetersToInches(int valueInCM) {
			return (((double)valueInCM)/2.54);
		}

		public double ConvertCentimetersToMeters(int valueinCM) {
			return (valueinCM/100.0);
		}


		public void StoreLidarValueForHopperAndShoot(double lidarValue) {
			storedLidarValue = lidarValue;
		}

		public double GetDistanceToTravelToHopper(double frontUltrasonicSensorValue) {
			distanceFromBumperToGuardrail = (frontUltrasonicSensorValue - DISTANCE_FROM_FRONT_ULTRASONIC_SENSOR_TO_BUMPER);
			desiredLidarValue = (START_OF_DESIRED_LIDAR_VALUE + distanceFromBumperToGuardrail);
			System.out.println("Desired Lidar Value: " + desiredLidarValue);
			desiredDistanceToTravelToHopper = (desiredLidarValue - storedLidarValue);
			System.out.println("Stored Lidar Value: " + storedLidarValue);
			System.out.println("Desired Distance To Travel: " + desiredDistanceToTravelToHopper);
			return desiredDistanceToTravelToHopper;
		}

		
    public void initDefaultCommand() {
        // Set the default command for a subsystem here.
        //setDefaultCommand(new MySpecialCommand());
    }
}

