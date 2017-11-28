package org.usfirst.frc.team9135.robot.subsystems;

import org.usfirst.frc.team9135.robot.RobotMap;

import edu.wpi.first.wpilibj.command.Subsystem;
import edu.wpi.first.wpilibj.*;

/**
 *
 */
public class GearHolder extends Subsystem 
{
		// It's desirable that everything possible under private except
		// for methods that implement subsystem capabilities

		private VictorSP gearHolderMotor;

		private DigitalInput upperLimitSwitch;
		private DigitalInput lowerLimitSwitch;

		private boolean upperLimitSwitchValue = false;
		private boolean lowerLimitSwitchValue = false;

		private double gearHolderMotorPower = 0.0;

		private boolean limitSwitchValue = false;

		private Servo gearHolderServo;

		//  Practice Bot PWM Port 4
		private static final int GEAR_HOLDER_SERVO_PWM_PORT = 0;

		private DigitalInput photoElectricSensor;
		private static final int PHOTO_ELECTRIC_SENSOR_PORT = 3;

		private boolean overrideUpperLimitSwitch = false;

		public static final int UPPER_LIMIT_SWITCH_PORT = 1;
		public static final int LOWER_LIMIT_SWITCH_PORT = 0;

		public static  double SERVO_OUT_POSITION = 0.0;
		//  150 for Competition Bot
		//  180 for Practice Bot
		public static  double SERVO_IN_POSITION = 150.0;

		public boolean gearUsedNotDefaultly = false;
		

		public void InitializeGearHolderMotor(boolean competitionBot) {
			if (competitionBot) {
				gearHolderMotor = new VictorSP(RobotMap.CB_GEAR_VICTOR_PWM_PORT);
				gearHolderMotor.setInverted(RobotMap.CB_GEAR_HOLDER_INVERTED);
				gearHolderServo = new Servo(RobotMap.CB_GEAR_HOLDER_SERVO_PWM_PORT);
			}
			else if (competitionBot == false) {
				gearHolderMotor = new VictorSP(RobotMap.PB_GEAR_VICTOR_PWM_PORT);
				gearHolderMotor.setInverted(RobotMap.PB_GEAR_HOLDER_INVERTED);
				gearHolderServo = new Servo(RobotMap.PB_GEAR_HOLDER_SERVO_PWM_PORT);
			}
			upperLimitSwitch = new DigitalInput(UPPER_LIMIT_SWITCH_PORT);
			lowerLimitSwitch = new DigitalInput(LOWER_LIMIT_SWITCH_PORT);

			photoElectricSensor = new DigitalInput(PHOTO_ELECTRIC_SENSOR_PORT);
		}

		public void DriveGearHolderMotor(double motorPower) {
			upperLimitSwitchValue = this.getLimitSwitchValue(UPPER_LIMIT_SWITCH_PORT);
			lowerLimitSwitchValue = this.getLimitSwitchValue(LOWER_LIMIT_SWITCH_PORT);

			if (upperLimitSwitchValue) {
				if (overrideUpperLimitSwitch == false) {
					gearHolderMotorPower = Math.min(0.0, motorPower);
				}
				else if (overrideUpperLimitSwitch) {
					gearHolderMotorPower = motorPower;
				}
			}
			else if (lowerLimitSwitchValue) {
				if (overrideUpperLimitSwitch == false) {
					gearHolderMotorPower = Math.max(0.0, motorPower);
				}
				else if (overrideUpperLimitSwitch) {
					gearHolderMotorPower = motorPower;
				}
			}
			else {
				gearHolderMotorPower = motorPower;
			}
			gearHolderMotor.set(gearHolderMotorPower);
		}

		public void OverrideUpperLimitSwitch() {
			overrideUpperLimitSwitch = !overrideUpperLimitSwitch;
		}

		public boolean getLimitSwitchValue(int limitSwitchDigitalInputNumber) {
			if (limitSwitchDigitalInputNumber == UPPER_LIMIT_SWITCH_PORT) {
				limitSwitchValue = !upperLimitSwitch.get();
			}
			else if (limitSwitchDigitalInputNumber == LOWER_LIMIT_SWITCH_PORT) {
				limitSwitchValue = !lowerLimitSwitch.get();
			}
			return limitSwitchValue;
		}

		public double getGearHolderServoValue() {
			return gearHolderServo.getAngle();
		}

		public void setGearHolderServoValue(double servoPosition) {
			gearHolderServo.setAngle(servoPosition);
		}

		public boolean getPhotoElectricSensorValue() {
			return !photoElectricSensor.get();
		}

    // Put methods for controlling this subsystem
    // here. Call these from Commands.

    public void initDefaultCommand() 
    {
    	setDefaultCommand(new DriveGearHolder());
        // set the default command for a subsystem here.
        //setDefaultCommand(new MySpecialCommand());
    }
}

