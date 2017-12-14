/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package org.usfirst.frc.team9135.robot;

/**
 * The RobotMap is a mapping from the ports sensors and actuators are wired into
 * to a variable name. This provides flexibility changing wiring, makes checking
 * the wiring easier and significantly reduces the number of magic numbers
 * floating around.
 */
public class RobotMap 
{
	public static final boolean COMPETITION_BOT = true;

	//  Practice Bot Motor Ports
	//  VictorSP motor ports
	public static final int PB_COLLECTION_VICTOR_PWM_PORT = 3;
	public static final int PB_HANG_VICTOR_PWM_PORT = 2;
	public static final int PB_GEAR_VICTOR_PWM_PORT = 0;
	public static final int PB_AGITATOR_VICTOR_PWM_PORT = 1;

	//  CANTalon motor ports
	public static final int PB_FRONT_LEFT_TALON_ID = 3;
	public static final int PB_REAR_LEFT_TALON_ID = 4;
	public static final int PB_FRONT_RIGHT_TALON_ID = 1;
	public static final int PB_REAR_RIGHT_TALON_ID = 2;

	public static final int PB_SHOOTER_MOTOR_TALON_ID = 6;
	public static final int PB_HOOD_MOTOR_TALON_ID = 5;

	public static final int PB_GEAR_HOLDER_SERVO_PWM_PORT = 4;

	//  Motor Inversions
	public static final boolean PB_AGITATOR_INVERTED = true;
	public static final boolean PB_COLLECTION_INVERTED = false;
	public static final boolean PB_DRIVE_TRAIN_FRONT_LEFT_INVERTED = false;
	public static final boolean PB_DRIVE_TRAIN_REAR_LEFT_INVERTED = false;
	public static final boolean PB_DRIVE_TRAIN_FRONT_RIGHT_INVERTED = true;
	public static final boolean PB_DRIVE_TRAIN_REAR_RIGHT_INVERTED = true;
	public static final boolean PB_GEAR_HOLDER_INVERTED = true;
	public static final boolean PB_LIFT_HANG_MOTOR_INVERTED = false;
	public static final boolean PB_SHOOTER_MOTOR_INVERTED = false;
	public static final boolean PB_SHOOTER_HOOD_MOTOR_INVERTED = false;
/////////////////////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////////////////////

	//  Competition Bot Motor Ports
	//  Victor SP Motor Ports
	public static final int CB_COLLECTION_VICTOR_PWM_PORT = 11;
	public static final int CB_HANG_VICTOR_PWM_PORT = 10;
	public static final int CB_GEAR_VICTOR_PWM_PORT = 17;
	public static final int CB_AGITATOR_VICTOR_PWM_PORT = 18;
	public static final int CB_AGITATOR_2_VICTOR_PWM_PORT = 12;

	//  CANTalon Motor Ports
	public static final int CB_FRONT_LEFT_TALON_ID = 1;
	public static final int CB_REAR_LEFT_TALON_ID = 2;
	public static final int CB_FRONT_RIGHT_TALON_ID = 3;
	public static final int CB_REAR_RIGHT_TALON_ID = 4;

	public static final int CB_SHOOTER_MOTOR_TALON_ID = 6;
	public static final int CB_HOOD_MOTOR_TALON_ID = 5;

	public static final int CB_GEAR_HOLDER_SERVO_PWM_PORT = 0;

	//  Motor Inversions
	//  To Be Determined
	public static final boolean CB_AGITATOR_INVERTED = true;
	public static final boolean CB_AGITATOR_2_INVERTED = false;
	public static final boolean CB_COLLECTION_INVERTED = true;
	public static final boolean CB_DRIVE_TRAIN_FRONT_LEFT_INVERTED = false;
	public static final boolean CB_DRIVE_TRAIN_REAR_RIGHT_INVERTED = true;
	public static final boolean CB_DRIVE_TRAIN_FRONT_RIGHT_INVERTED = true;
	public static final boolean CB_DRIVE_TRAIN_REAR_LEFT_INVERTED = false;
	public static final boolean CB_GEAR_HOLDER_INVERTED = false;
	public static final boolean CB_LIFT_HANG_MOTOR_INVERTED = false;
	public static final boolean CB_SHOOTER_MOTOR_INVERTED = false;
	public static final boolean CB_SHOOTER_HOOD_MOTOR_INVERTED = true;

	/*enum AutonomousSelection {
		MiddleGear = 0,
		RightGear = 1,
		LeftGear = 2,
		CloseShotShooterLeft = 3,
		CloseShotShooterRight = 4,
		RightKPaAutonomous = 5,
		LeftKPaAutonomous = 6,
		BaseLine = 7
	};

	enum SecondTask {
		MiddleGearShootRight = 0,
		MiddleGearShootLeft = 1,
		SideGearShoot = 2,
		CloseShotBaseLine = 3,
		None
	}; */

	public enum ShooterPIDSelection {
		PID_CompetitionBot,
		PID_Ramp_Up,
		Voltage
	};

	public static final ShooterPIDSelection SHOOTER_PID_SELECTION = ShooterPIDSelection.PID_CompetitionBot;
	// For example to map the left and right motors, you could define the
	// following variables to use with your drivetrain subsystem.
	// public static int leftMotor = 1;
	// public static int rightMotor = 2;

	// If you are using multiple modules, make sure to define both the port
	// number and the module. For example you with a rangefinder:
	// public static int rangefinderPort = 1;
	// public static int rangefinderModule = 1;
}
