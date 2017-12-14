/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package org.usfirst.frc.team9135.robot;

import java.math.*;
import edu.wpi.first.wpilibj.buttons.*;
import edu.wpi.first.wpilibj.*;
import org.usfirst.frc.team9135.robot.subsystems.*;
import org.usfirst.frc.team9135.robot.commands.*;

/**
 * This class is the glue that binds the controls on the physical operator
 * interface to the commands and command groups that allow control of the robot.
 */
public class OI {
		private static final int NUM_OF_JOYSTICKS = 3;
		private Joystick[] joystick = new Joystick[NUM_OF_JOYSTICKS];

		private  static final int NUM_OF_BUTTONS = 12;
		private JoystickButton[][] joystickButton = new JoystickButton[NUM_OF_JOYSTICKS][NUM_OF_BUTTONS];

		private double joystickYAxisValue = 0.0;
		private double deadbandJoystickValue = 0.0;

		static final double JOYSTICK_DEADBAND_VALUE = .15;

		static final int AGITATOR_FORWARD_BUTTON = 8;
		static final int AGITATOR_BACKWARDS_BUTTON = 7;

		static final int COLLECTION_FORWARD_BUTTON = 10;
		static final int COLLECTION_BACKWARDS_BUTTON = 12;

		static final int SHOOTER_HOOD_INCREASE_ANGLE_BUTTON = 5;
		static final int SHOOTER_HOOD_DECREASE_ANGLE_BUTTON = 3;

		static final int SHOOTER_HOOD_SIDE_GEAR_SHOT_BUTTON = 12;  //  Right Drive Joystick
		static final int SHOOTER_HOOD_MIDDLE_GEAR_BUTTON = 11;  //  Right Drive Joystick
		static final int SHOOTER_HOOD_40_KPA_AUTONOMOUS_BUTTON = 10;  //  Right Drive Joystick
		static final int SHOOTER_HOOD_LEFT_CLOSE_SHOT_BUTTON = 9;

		static final int SHOOTER_HOOD_CLOSE_SHOT_BUTTON = 11;  //  MANIPULATOR JOYSTICK

		static final int GET_LIFT_HANG_INTO_CORRECT_POSITION_BUTTON = 5;  //  Right Drive Joystick

		static final int SWITCH_BETWEEN_MAX_DRIVE_TRAIN_MOTOR_POWER_BUTTON = 6;  //  Right Drive Joystick

		static final int SWITCH_BETWEEN_FAR_AND_CLOSE_SHOT_RPM_BUTTON = 9;  //  Manipulator Joystick

		static final int READ_LIDAR_VALUE_BUTTON = 8;  //  Left Drive Joystick

		private static final int OVERRIDE_GEAR_LIMIT_SWITCHES_BUTTON = 7;

		private static final int CLOSE_SHOT_HOOD_ENCODER_VALUE = 0;
		private static final int RIGHT_CLOSE_SHOT_SHOOTER_AUTONOMOUS_HOOD_ENCODER_VALUE = 0;
		private static final int LEFT_CLOSE_SHOT_SHOOTER_AUTONOMOUS_HOOD_ENCODER_VALUE = 1250;
		private static final int MIDDLE_GEAR_SHOOT_AUTONOMUS_HOOD_ENCODER_VALUE = 6750;  //  15000 Tested on Practice Bot
		private static final int SIDE_GEAR_SHOOT_AUTONOMOUS_HOOD_ENCODER_VALUE = 3750;
		private static final int KPA_AUTONOMOUS_HOOD_ENCODER_VALUE = 5400;

		private static final int POV_NUMBER = 0;

		private double povValue = 0.0;
		private boolean povDirectionPressed = false;

		private double throttleValue = 0.0;
		private boolean throttleUp = false;

		private static final double LIFT_HANG_FULL_POWER = 1.0;
		private static final double LIFT_HANG_HALF_POWER = .45;
		
		public static final int LEFT_DRIVE_JOYSTICK = 0;
		public static final int RIGHT_DRIVE_JOYSTICK = 1;
		public static final int MANIPULATOR_JOYSTICK = 2;

		public static final int TRIGGER_BUTTON = 1;
		public static final int THUMB_BUTTON = 2;

		public static final int TOP_POV = 0;
		public static final int RIGHT_POV = 1;
		public static final int BOTTOM_POV = 2;
		public static final int LEFT_POV = 3;

		public static final int GEAR_HOLDER_UPWARDS_BUTTON = 6;
		public static final int GEAR_HOLDER_DOWNWARDS_BUTTON = 4;
		
		public OI() {
			// Process operator interface input here.

			for (int i = 0; i < NUM_OF_JOYSTICKS; i++) {
				joystick[i] = new Joystick(i);
				for (int j = 1; j <= NUM_OF_BUTTONS; j++) {
					joystickButton[i][j] = new JoystickButton(joystick[i], j);
				}
			}

			this.ConfigureButtonMapping();
		}

		public double GetYAxis(int joystickNumber) {
			joystickYAxisValue = (-1 * joystick[joystickNumber].getAxis(Joystick.AxisType.kY));
			return this.DeadbandJoystickValue(joystickYAxisValue);
		}

		public double DeadbandJoystickValue(double joystickValue) {
			if (joystickValue < JOYSTICK_DEADBAND_VALUE && joystickValue > -JOYSTICK_DEADBAND_VALUE) {
				deadbandJoystickValue = 0.0;
			}
			else {
				deadbandJoystickValue = joystickValue;
			}
			return deadbandJoystickValue;
		}

		public boolean GetButtonPressed(int joystickNumber, int joystickButtonNumber) {
			return joystickButton[joystickNumber][joystickButtonNumber].get();
		}

		public double GetThrottleValue(int joystickNumber) {
			return (-1 * joystick[joystickNumber].getThrottle());
		}

		boolean GetThrottleUp(int joystickNumber) {
			throttleValue = this.GetThrottleValue(joystickNumber);
			if (throttleValue >= 0) {
				throttleUp = true;
			}
			else if (throttleValue < 0) {
				throttleUp = false;
			}
			return throttleUp;
		}

		int GetAngleOfPOV(int joystickNumber) {
			return joystick[joystickNumber].getPOV(POV_NUMBER);
		}

		public boolean POVDirectionPressed(int joystickNumber, int povDirection) {
			povValue = this.GetAngleOfPOV(joystickNumber);

			switch(povDirection) {
			case (TOP_POV):
				if (povValue >= 315.0 || (povValue <= 45.0 && povValue != -1)) {
					povDirectionPressed = true;
				}
				else {
					povDirectionPressed = false;
				}
				break;
			case (RIGHT_POV):
				if (povValue >= 45.0 && povValue <= 135.0) {
					povDirectionPressed = true;
				}
				else {
					povDirectionPressed = false;
				}
				break;
			case (BOTTOM_POV):
				if (povValue >= 135.0 && povValue <= 225.0) {
					povDirectionPressed = true;
				}
				else {
					povDirectionPressed = false;
				}
				break;
			case (LEFT_POV):
				if (povValue >= 225.0 && povValue <= 315.0) {
					povDirectionPressed = true;
				}
				else {
					povDirectionPressed = false;
				}
				break;
			}
			return povDirectionPressed;
		}

		void ConfigureButtonMapping() {
			joystickButton[MANIPULATOR_JOYSTICK][AGITATOR_FORWARD_BUTTON].WhileHeld(new DriveAgitator(true));
			//joystickButton[MANIPULATOR_JOYSTICK][AGITATOR_BACKWARDS_BUTTON].WhileHeld(new DriveAgitator(false));

			joystickButton[MANIPULATOR_JOYSTICK][COLLECTION_FORWARD_BUTTON].WhileHeld(new DriveCollection(true));
			joystickButton[MANIPULATOR_JOYSTICK][COLLECTION_BACKWARDS_BUTTON].WhileHeld(new DriveCollection(false));

			joystickButton[RIGHT_DRIVE_JOYSTICK][TRIGGER_BUTTON].WhileHeld(new DriveLiftHang(LIFT_HANG_FULL_POWER));
			joystickButton[RIGHT_DRIVE_JOYSTICK][THUMB_BUTTON].WhileHeld(new DriveLiftHang(LIFT_HANG_HALF_POWER));
			joystickButton[RIGHT_DRIVE_JOYSTICK][GET_LIFT_HANG_INTO_CORRECT_POSITION_BUTTON].WhenPressed(new GetReadyForLiftHang());

			joystickButton[RIGHT_DRIVE_JOYSTICK][SWITCH_BETWEEN_MAX_DRIVE_TRAIN_MOTOR_POWER_BUTTON].WhenPressed(new SwitchDriveTrainMotorPower());

			joystickButton[MANIPULATOR_JOYSTICK][SHOOTER_HOOD_INCREASE_ANGLE_BUTTON].WhileHeld(new DriveShooterHood(true));
			joystickButton[MANIPULATOR_JOYSTICK][SHOOTER_HOOD_DECREASE_ANGLE_BUTTON].WhileHeld(new DriveShooterHood(false));

			joystickButton[MANIPULATOR_JOYSTICK][SWITCH_BETWEEN_FAR_AND_CLOSE_SHOT_RPM_BUTTON].WhenPressed(new SwitchBetweenHighAndLowShot());

			joystickButton[MANIPULATOR_JOYSTICK][SHOOTER_HOOD_CLOSE_SHOT_BUTTON].whenPressed(new AutoDriveShooterHood(CLOSE_SHOT_HOOD_ENCODER_VALUE));
			joystickButton[RIGHT_DRIVE_JOYSTICK][SHOOTER_HOOD_SIDE_GEAR_SHOT_BUTTON].whenPressed(new AutoDriveShooterHood(SIDE_GEAR_SHOOT_AUTONOMOUS_HOOD_ENCODER_VALUE));
			joystickButton[RIGHT_DRIVE_JOYSTICK][SHOOTER_HOOD_MIDDLE_GEAR_BUTTON].whenPressed(new AutoDriveShooterHood(MIDDLE_GEAR_SHOOT_AUTONOMUS_HOOD_ENCODER_VALUE));
			joystickButton[RIGHT_DRIVE_JOYSTICK][SHOOTER_HOOD_40_KPA_AUTONOMOUS_BUTTON].whenPressed(new AutoDriveShooterHood(KPA_AUTONOMOUS_HOOD_ENCODER_VALUE));
			joystickButton[RIGHT_DRIVE_JOYSTICK][SHOOTER_HOOD_LEFT_CLOSE_SHOT_BUTTON].whenPressed(new AutoDriveShooterHood(LEFT_CLOSE_SHOT_SHOOTER_AUTONOMOUS_HOOD_ENCODER_VALUE));

			joystickButton[MANIPULATOR_JOYSTICK][OVERRIDE_GEAR_LIMIT_SWITCHES_BUTTON].WhenPressed(new OverrideGearHolderLimitSwitches());

			//joystickButton[LEFT_DRIVE_JOYSTICK][10].WhenPressed(new CameraAdjustWhileDrivingToGear());
			//joystickButton[LEFT_DRIVE_JOYSTICK][10].WhenPressed(new TurnOneSideOfRobotAngle(2.5,false,.5));
			//joystickButton[LEFT_DRIVE_JOYSTICK][3].WhenPressed(new AutoRotateRobotForGearPeg(.5, 10.0));

			//joystickButton[LEFT_DRIVE_JOYSTICK][TRIGGER_BUTTON].ToggleWhenPressed(new GearUpAndDown());

			//joystickButton[LEFT_DRIVE_JOYSTICK][READ_LIDAR_VALUE_BUTTON].ToggleWhenPressed(new ReadLidarValues());

			//joystickButton[LEFT_DRIVE_JOYSTICK][12].WhenPressed(new TurnDriveTrainAngle(48.0, .65, false));
			//joystickButton[LEFT_DRIVE_JOYSTICK][11].WhenPressed(new TurnDriveTrainAngle(43.0, .65, false));

			//joystickButton[LEFT_DRIVE_JOYSTICK][12].WhenPressed(new DriveParallelWithGuardrailWithUltrasonicSensor(16.0, 5.0, -.35, true));
			//joystickButton[LEFT_DRIVE_JOYSTICK][11].CancelWhenPressed(new DriveParallelWithGuardrailWithUltrasonicSensor(50.0, 5.0, -.45, true));

			//joystickButton[LEFT_DRIVE_JOYSTICK][9].ToggleWhenPressed(new GearUpAndDown());
		}

		public boolean GetAction(int JoyException, int ButtonException)
		{
			for(int i = 0; i < NUM_OF_JOYSTICKS; i++)
			{
				if(Math.abs(joystick[i].getY()) > .15)
					return true;
				for(int j = 1; j < NUM_OF_BUTTONS + 1; j++)
				{
						if(i == JoyException && j == ButtonException)
							continue;
						if(joystickButton[i][j].get())
							return true;
				}
			}
			return false;
		}
}
