package org.usfirst.frc.team9135.robot.subsystems;

import org.usfirst.frc.team9135.robot.RobotMap;
import org.usfirst.frc.team9135.robot.commands.DriveShooter;

import com.ctre.CANTalon;

import edu.wpi.first.wpilibj.command.Subsystem;

/**
 *
 */
public class Shooter extends Subsystem {

    // Put methods for controlling this subsystem
    // here. Call these from Commands.
	
	public static final double DESIRED_VOLTAGE_CLOSE_SHOT = 7.7;
	public static final double DESIRED_VOLTAGE_FAR_SHOT = 9.125;

	public static final double SHOOTER_setPOINT_MIN_RPM_CLOSE_SHOT = 2800.0; //  2600
	public static final double SHOOTER_setPOINT_MAX_RPM_CLOSE_SHOT = 3300.0;  //  2900
	public static final double RANGE_OF_CLOSE_SHOT_SHOOTER_RPM = (SHOOTER_setPOINT_MAX_RPM_CLOSE_SHOT - SHOOTER_setPOINT_MIN_RPM_CLOSE_SHOT);

	public static final double SHOOTER_setPOINT_RPM_CLOSE_SHOT = 2715.0;
	public static final double SHOOTER_setPOINT_RPM_FAR_SHOT = 3040.0;

	public static final int CLOSE_SHOT_PID_VALUES = 0;
	public static final int FAR_SHOT_PID_VALUES = 1;

	public boolean stopShooterFromDriving = false;

	public static final double SHOOTER_setPOINT_MINIMUM_PART_1 = 2650.0;
	public static final double SHOOTER_setPOINT_MAXIMUM_PART_1 = 3000.0;
	public static final double SHOOTER_setPOINT_MINIMUM_PART_2 = 3000.0;
	public static final double SHOOTER_setPOINT_MAXIMUM_PART_2 = 3300.0;

	public static final double SHOOTER_CLOSE_SHOT_setPOINT = 2850.0;

	public static final double SHOOTER_RANGE_OF_RPM_PART_1 = (SHOOTER_setPOINT_MAXIMUM_PART_1 - SHOOTER_setPOINT_MINIMUM_PART_1);
	public static final double SHOOTER_RANGE_OF_RPM_PART_2 = (SHOOTER_setPOINT_MAXIMUM_PART_2 - SHOOTER_setPOINT_MINIMUM_PART_2);

	CANTalon shooterMotor;

	private static final int SHOOTER_ENCODER_COUNT = 1024;
	private static final int SHOOTER_ENCODER_QUADRATURE_COUNT = (SHOOTER_ENCODER_COUNT * 4);

	private static final boolean REVERSE_SHOOTER_ENCODER_DIRECTION = true;

	private static final double SHOOTER_GEAR_RATIO = (30.0/24.0);

	private static final double SHOOTER_MAX_VOLTAGE = 12.0;
	private static final double MAX_MOTOR_OUTPUT_VALUE = 1023.0;

	private static final double SHOOTER_setPOINT_NU_PER_100MS_CLOSE_SHOT = 17476.0;
	private static final double PERCENT_VOLTAGE_CLOSE_SHOT = (DESIRED_VOLTAGE_CLOSE_SHOT/SHOOTER_MAX_VOLTAGE);
	private static final double MOTOR_OUTPUT_VALUE_FEEDFORWARD_CLOSE_SHOT = (PERCENT_VOLTAGE_CLOSE_SHOT * MAX_MOTOR_OUTPUT_VALUE);
	private static final double FEEDFORWARD_TERM_CLOSE_SHOT = (MOTOR_OUTPUT_VALUE_FEEDFORWARD_CLOSE_SHOT/SHOOTER_setPOINT_NU_PER_100MS_CLOSE_SHOT);

	private static final double SHOOTER_setPOINT_NU_PER_100MS_FAR_SHOT = 20800.0;
	private static final double PERCENT_VOLTAGE_FAR_SHOT = (DESIRED_VOLTAGE_FAR_SHOT/SHOOTER_MAX_VOLTAGE);
	private static final double MOTOR_OUTPUT_VALUE_FEEDFORWARD_FAR_SHOT = (PERCENT_VOLTAGE_FAR_SHOT * MAX_MOTOR_OUTPUT_VALUE);
	private static final double FEEDFORWARD_TERM_FAR_SHOT = (MOTOR_OUTPUT_VALUE_FEEDFORWARD_FAR_SHOT/SHOOTER_setPOINT_NU_PER_100MS_FAR_SHOT);

	double closeShotKp = 0.0;
	double closeShotKi = 0.0;
	double closeShotKd = 0.0;
	double closeShotKf = 0.0;
	double closeShotPositivePeakVoltage = 0.0;
	double closeShotNegativePeakVoltage = 0.0;

	double farShotKp = 0.0;
	double farShotKi = 0.0;
	double farShotKd = 0.0;
	double farShotKf = 0.0;
	double farShotPositivePeakVoltage = 0.0;
	double farShotNegativePeakVoltage = 0.0;

	private static final double PB_CLOSE_SHOT_Kp = .25;  //  1.3
	private static final double PB_CLOSE_SHOT_Ki = 0.0;
	private static final double PB_CLOSE_SHOT_Kd = .5;  //  8.0
	private static final double PB_CLOSE_SHOT_Kf = .0352;  //  .0361
	private static final double PB_CLOSE_SHOT_POSITIVE_PEAK_VOLTAGE = 9.0;
	private static final double PB_CLOSE_SHOT_NEGATIVE_PEAK_VOLTAGE = -6.0;

	private static final double PB_FAR_SHOT_Kp = 1.0;
	private static final double PB_FAR_SHOT_Ki = 0.0;
	private static final double PB_FAR_SHOT_Kd = 0.0;
	private static final double PB_FAR_SHOT_Kf = .0359;
	private static final double PB_FAR_SHOT_POSITIVE_PEAK_VOLTAGE = 10.2;
	private static final double PB_FAR_SHOT_NEGATIVE_PEAK_VOLTAGE = -6.0;

	private static final double PB_NEW_PID_CLOSE_SHOT_Kp = .4;
	private static final double PB_NEW_PID_CLOSE_SHOT_Ki = .00015;
	private static final double PB_NEW_PID_CLOSE_SHOT_Kd = 2.0;
	private static final double PB_NEW_PID_CLOSE_SHOT_Kf = .032;
	private static final double PB_NEW_PID_CLOSE_SHOT_IZone = 8196.0;

	private static final double CB_CLOSE_SHOT_Kp = .25;  //  1.3
	private static final double CB_CLOSE_SHOT_Ki = 0.0;
	private static final double CB_CLOSE_SHOT_Kd = .5;  //  8.0
	private static final double CB_CLOSE_SHOT_Kf = .0352;  //  .0361
	private static final double CB_CLOSE_SHOT_POSITIVE_PEAK_VOLTAGE = 9.0;
	private static final double CB_CLOSE_SHOT_NEGATIVE_PEAK_VOLTAGE = -6.0;

	private static final double CB_FAR_SHOT_Kp = 1.0;  //  1.0
	private static final double CB_FAR_SHOT_Ki = 0.0;  //  0.0
	private static final double CB_FAR_SHOT_Kd = 0.0;  //  0.0
	private static final double CB_FAR_SHOT_Kf = .0359;  //  /.0359
	private static final double CB_FAR_SHOT_POSITIVE_PEAK_VOLTAGE = 10.2;  //
	private static final double CB_FAR_SHOT_NEGATIVE_PEAK_VOLTAGE = -6.0;

	private static final double CB_NEW_PID_CLOSE_SHOT_Kp = .5;
	private static final double CB_NEW_PID_CLOSE_SHOT_Ki = .00015;
	private static final double CB_NEW_PID_CLOSE_SHOT_Kd = 2.0;
	private static final double CB_NEW_PID_CLOSE_SHOT_Kf = .032;
	private static final double CB_NEW_PID_CLOSE_SHOT_IZONE = 8196.0;

	//  Variable for ShooterUpToSpeed() and getShooterUpToSpeed()
	public boolean shooterUpToSpeed = false;

	public double convertedThrottleValue = 0.0;
	public double desiredCloseShotShootersetpoint = 0.0;

	public boolean closeShot = true;


	public void InitDefaultCommand() {
		// set the default command for a subsystem here.
		// setDefaultCommand(new MySpecialCommand());
		setDefaultCommand(new DriveShooter(RobotMap.SHOOTER_PID_SELECTION));
	}

	public void InitializeShooterMotor(boolean competitionBot) {
		if (competitionBot) {
			shooterMotor = new CANTalon(RobotMap.CB_SHOOTER_MOTOR_TALON_ID);
			shooterMotor.setInverted(RobotMap.CB_SHOOTER_MOTOR_INVERTED);
		}
		else if (competitionBot == false) {
			shooterMotor = new CANTalon(RobotMap.PB_SHOOTER_MOTOR_TALON_ID);
			shooterMotor.setInverted(RobotMap.PB_SHOOTER_MOTOR_INVERTED);
		}
	}

	public void DriveShooterMotor(double motorPower) {
		shooterMotor.set(motorPower);
	}

	public void ConfigureShooterMotorEncoder(boolean competitionBot) {
		shooterMotor.configEncoderCodesPerRev(SHOOTER_ENCODER_COUNT);
		shooterMotor.setFeedbackDevice(CANTalon.FeedbackDevice.CtreMagEncoder_Relative);
		shooterMotor.reverseSensor(REVERSE_SHOOTER_ENCODER_DIRECTION);
		shooterMotor.setPosition(0.0);
		shooterMotor.setNominalClosedLoopVoltage(SHOOTER_MAX_VOLTAGE);

		if (competitionBot) {
			closeShotKp = CB_NEW_PID_CLOSE_SHOT_Kp;
			closeShotKi = CB_NEW_PID_CLOSE_SHOT_Ki;
			closeShotKd = CB_NEW_PID_CLOSE_SHOT_Kd;
			closeShotKf = CB_NEW_PID_CLOSE_SHOT_Kf;
			closeShotPositivePeakVoltage = CB_CLOSE_SHOT_POSITIVE_PEAK_VOLTAGE;
			closeShotNegativePeakVoltage = CB_CLOSE_SHOT_NEGATIVE_PEAK_VOLTAGE;

			farShotKp = CB_FAR_SHOT_Kp;
			farShotKi = CB_FAR_SHOT_Ki;
			farShotKd = CB_FAR_SHOT_Kd;
			farShotKf = CB_FAR_SHOT_Kf;
			farShotPositivePeakVoltage = CB_FAR_SHOT_POSITIVE_PEAK_VOLTAGE;
			farShotNegativePeakVoltage = CB_FAR_SHOT_NEGATIVE_PEAK_VOLTAGE;
		}
		else if (competitionBot == false) {
			closeShotKp = PB_CLOSE_SHOT_Kp;
			closeShotKi = PB_CLOSE_SHOT_Ki;
			closeShotKd = PB_CLOSE_SHOT_Kd;
			closeShotKf = PB_CLOSE_SHOT_Kf;
			closeShotPositivePeakVoltage = PB_CLOSE_SHOT_POSITIVE_PEAK_VOLTAGE;
			closeShotNegativePeakVoltage = PB_CLOSE_SHOT_NEGATIVE_PEAK_VOLTAGE;

			farShotKp = PB_FAR_SHOT_Kp;
			farShotKi = PB_FAR_SHOT_Ki;
			farShotKd = PB_FAR_SHOT_Kd;
			farShotKf = PB_FAR_SHOT_Kf;
			farShotPositivePeakVoltage = PB_FAR_SHOT_POSITIVE_PEAK_VOLTAGE;
			farShotNegativePeakVoltage = PB_FAR_SHOT_NEGATIVE_PEAK_VOLTAGE;
		}

		this.SelectPIDProfileSlot(CLOSE_SHOT_PID_VALUES);
		shooterMotor.configNominalOutputVoltage(12.0, -12.0);
		shooterMotor.configNominalOutputVoltage(0.0, -0.0);
		shooterMotor.setP(closeShotKp);
		shooterMotor.setI(closeShotKi);
		shooterMotor.setD(closeShotKd);
		shooterMotor.setF(closeShotKf);

		this.SelectPIDProfileSlot(FAR_SHOT_PID_VALUES);
		shooterMotor.configNominalOutputVoltage(12.0, -12.0);
		shooterMotor.configNominalOutputVoltage(0.0, -0.0);
		shooterMotor.setP(farShotKp);
		shooterMotor.setI(farShotKi);
		shooterMotor.setD(farShotKd);
		shooterMotor.setF(farShotKf);
	}

	public void ConfigureShooterVoltageMode() {
		shooterMotor.changeControlMode(CANTalon.TalonControlMode.Voltage);
		shooterMotor.set(0.0);
	}

	public void ConfigureShooterPID() {
		shooterMotor.changeControlMode(CANTalon.TalonControlMode.Speed);
		shooterMotor.set(0.0);
	}

	public int getShooterWheelRPM() {
		return (int) shooterMotor.getSpeed();
	}

	public int getShooterWheelNUPer100Ms() {
		return shooterMotor.getEncVelocity();
	}

	public double getShooterVoltage() {
		return shooterMotor.getOutputVoltage();
	}

	public double getShooterMotorOutputCurrent() {
		return shooterMotor.getOutputCurrent();
	}

	public void ZeroAccumulatedError() {
		shooterMotor.ClearIaccum();
	}

	public void SelectPIDProfileSlot(int profileSlot) {
		shooterMotor.setProfile(profileSlot);
		
	}

	public void ShooterUpToSpeed(boolean shooterUpToSpeed) {
		this.shooterUpToSpeed = shooterUpToSpeed;
	}

	public boolean getShooterUpToSpeed() {
		return (this.shooterUpToSpeed);
	}

	public double getCloseShotShooterRPMGivenThrottleValue(double throttleValue, double rangeOfShooterRPM, double minimumShootersetpoint) {
		convertedThrottleValue = (throttleValue + 1.0);
		desiredCloseShotShootersetpoint = (((convertedThrottleValue / 2.0) * rangeOfShooterRPM) + minimumShootersetpoint);
		return desiredCloseShotShootersetpoint;
	}

	void setSwitchFarAndCloseShotShooterRPM(boolean closeShot) {
		this.closeShot = closeShot;
	}

	public boolean getSwitchBetweenFarAndCloseShotShooterRPM() {
		return this.closeShot;
	}

    public void initDefaultCommand() {
        // set the default command for a subsystem here.
        //setDefaultCommand(new MySpecialCommand());
    }
}

