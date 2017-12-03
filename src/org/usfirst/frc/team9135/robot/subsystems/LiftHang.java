package org.usfirst.frc.team9135.robot.subsystems;

import org.usfirst.frc.team9135.robot.RobotMap;

import edu.wpi.first.wpilibj.*;
import edu.wpi.first.wpilibj.command.Subsystem;

/**
 *
 */
public class LiftHang extends Subsystem {
	private VictorSP liftHangMotor;

	private Encoder liftHangEncoder;

	private static final int LIFT_HANG_ENCODER_A_CHANNEL = 6;
	private static final int LIFT_HANG_ENCODER_B_CHANNEL = 7;
	private static final boolean REVERSE_ENCODER_DIRECTION = true;
	private final Encoder.EncodingType QUADRATURE_ENCODER = Encoder.EncodingType.k4X;

	private int liftHangEncoderValue = 0;
	private double numberOfRotationsOfLiftHang = 0.0;
	
	public static final int ENCODER_COUNT = 1024;
	public static final int QUADRATURE_ENCODER_COUNT = (ENCODER_COUNT * 4);
	public static final int LIFT_HANG_READY_ENCODER_POSITION = (((3 * QUADRATURE_ENCODER_COUNT)/ 4) - 1181);

    // Put methods for controlling this subsystem
    // here. Call these from Commands.
	
	public LiftHang()
	{

	}

	@Override
	public void initDefaultCommand() {
		// Set the default command for a subsystem here.
		// SetDefaultCommand(new MySpecialCommand());
		SetDefaultCommand(new ReadLiftHangEncoderValue());
	}

	public void InitializeLiftHang(boolean competitionBot) {
		if (competitionBot) {
			liftHangMotor = new VictorSP(RobotMap.CB_HANG_VICTOR_PWM_PORT);
			liftHangMotor.setInverted(RobotMap.CB_LIFT_HANG_MOTOR_INVERTED);
		}
		else if (competitionBot == false) {
			liftHangMotor = new VictorSP(RobotMap.PB_HANG_VICTOR_PWM_PORT);
			liftHangMotor.setInverted(RobotMap.PB_LIFT_HANG_MOTOR_INVERTED);
		}
	}

	public void DriveLiftHang(double motorPower) {
		liftHangMotor.set(motorPower);
	}

	public void InitializeLiftHangEncoder() {
		liftHangEncoder = new Encoder(LIFT_HANG_ENCODER_A_CHANNEL, LIFT_HANG_ENCODER_B_CHANNEL, REVERSE_ENCODER_DIRECTION, QUADRATURE_ENCODER);
	}

	public void ZeroLiftHangEncoder() {
		liftHangEncoder.reset();
	}

	public int GetLiftHangEncoderRawValue() {
		return liftHangEncoder.getRaw();
	}

	public int GetLiftHangEncoderValue() {
		return liftHangEncoder.get();
	}

	public double GetNumberOfRotationsOfLiftHang() {
		liftHangEncoderValue = this.GetLiftHangEncoderRawValue();
		numberOfRotationsOfLiftHang = (((double)liftHangEncoderValue)/QUADRATURE_ENCODER_COUNT);
		return numberOfRotationsOfLiftHang;
	}




}

