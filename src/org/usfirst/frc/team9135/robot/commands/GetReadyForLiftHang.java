package org.usfirst.frc.team9135.robot.commands;

import org.usfirst.frc.team9135.robot.CommandBase;
import org.usfirst.frc.team9135.robot.subsystems.LiftHang;

import edu.wpi.first.wpilibj.command.Command;

/**
 *
 */
public class GetReadyForLiftHang extends Command {
	
	private int liftHangCurrentEncoderPosition = 0;
	private double currentLiftHangNumberOfRotations = 0.0;

	private double lowNumberOfCurrentRotations = 0.0;
	private double highNumberOfCurrentRotations = 0.0;
	private int liftHangDesiredEncoderValueForLowNumberOfRotations = 0;

	private boolean calculateDesiredEncoderPosition = false;
	private int desiredEncoderPosition = 0;

	private int differenceBetweenCurrentAndDesiredLiftHangEncoderValue = 0;

	private boolean liftHangInDesiredPosition = false;

	private static final double LIFT_HANG_MOTOR_POWER = .2;

	private static final int VARIABILITY_IN_DESIRED_LIFT_HANG_ENCODER_POSITION = 135;

    public GetReadyForLiftHang() {
        // Use requires() here to declare subsystem dependencies
        // eg. requires(chassis);
    	requires(CommandBase.liftHang);
    }

    // Called just before this Command runs the first time
    protected void initialize() {
    	calculateDesiredEncoderPosition = false;
    }

    // Called repeatedly when this Command is scheduled to run
    protected void execute() {
    	liftHangCurrentEncoderPosition = CommandBase.liftHang.GetLiftHangEncoderRawValue();
    	currentLiftHangNumberOfRotations = CommandBase.liftHang.GetNumberOfRotationsOfLiftHang();
    	lowNumberOfCurrentRotations = Math.floor(currentLiftHangNumberOfRotations);
    	liftHangDesiredEncoderValueForLowNumberOfRotations = ((((int)lowNumberOfCurrentRotations) * LiftHang.QUADRATURE_ENCODER_COUNT) + LiftHang.LIFT_HANG_READY_ENCODER_POSITION);

    	differenceBetweenCurrentAndDesiredLiftHangEncoderValue = (Math.abs(liftHangDesiredEncoderValueForLowNumberOfRotations - liftHangCurrentEncoderPosition));
    	//std::cout << "Difference Between CUrrent and Deisred: " << differenceBetweenCurrentAndDesiredLiftHangEncoderValue << std::endl;

    	if (calculateDesiredEncoderPosition == false) {
    		if (differenceBetweenCurrentAndDesiredLiftHangEncoderValue < VARIABILITY_IN_DESIRED_LIFT_HANG_ENCODER_POSITION) {
    			liftHangInDesiredPosition = true;
    		}
    		else if (liftHangCurrentEncoderPosition < liftHangDesiredEncoderValueForLowNumberOfRotations) {
    			desiredEncoderPosition = liftHangDesiredEncoderValueForLowNumberOfRotations;
    		}
    		else if (liftHangCurrentEncoderPosition > liftHangDesiredEncoderValueForLowNumberOfRotations) {
    			highNumberOfCurrentRotations = (Math.ceil(currentLiftHangNumberOfRotations));
    			desiredEncoderPosition = ((((int)highNumberOfCurrentRotations) * LiftHang.QUADRATURE_ENCODER_COUNT) + LiftHang.LIFT_HANG_READY_ENCODER_POSITION);
    		}

    		//std::cout << "Desired Lift Hang Encoder Position: " << desiredEncoderPosition << std::endl;

    		calculateDesiredEncoderPosition = true;
    	}

    	if (calculateDesiredEncoderPosition) {
    		if (liftHangCurrentEncoderPosition > desiredEncoderPosition) {
    			CommandBase.liftHang.DriveLiftHang(0.0);
    			liftHangInDesiredPosition = true;
    			calculateDesiredEncoderPosition = false;
    		}
    		else {
    			CommandBase.liftHang.DriveLiftHang(LIFT_HANG_MOTOR_POWER);
    			liftHangInDesiredPosition= false;
    		}
    	}
    }

    // Make this return true when this Command no longer needs to run execute()
    protected boolean isFinished() {
    	return liftHangInDesiredPosition;
    }

    // Called once after isFinished returns true
    protected void end() {
    	CommandBase.liftHang.DriveLiftHang(0.0);
    	calculateDesiredEncoderPosition = false;
    	liftHangInDesiredPosition= false;
    }

    // Called when another command which requires one or more of the same
    // subsystems is scheduled to run
    protected void interrupted() {
    	end();
    }
}
