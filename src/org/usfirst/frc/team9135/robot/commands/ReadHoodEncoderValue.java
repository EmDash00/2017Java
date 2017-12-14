package org.usfirst.frc.team9135.robot.commands;

import org.usfirst.frc.team9135.robot.CommandBase;

import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/**
 *
 */
public class ReadHoodEncoderValue extends Command {
	private int shooterHoodEncoderValue = 0;
	
	private int maxAngleLimitSwitchValue = 0;
	private int minAngleLimitSwitchValue = 0;

	private boolean hoodEncoderPluggedIn = false;

    public ReadHoodEncoderValue() {
        // Use requires() here to declare subsystem dependencies
        // eg. requires(chassis);
    	requires(CommandBase.shooterHood);
    }

    // Called just before this Command runs the first time
    protected void initialize() {
    }

    // Called repeatedly when this Command is scheduled to run
    protected void execute() {
    	shooterHoodEncoderValue = CommandBase.shooterHood.GetShooterHoodEncoderPosition();
    	//std.cout << "Shooter Hood Encoder Value: " << shooterHoodEncoderValue << std.endl;

    	SmartDashboard.putNumber("Shooter Hood Encoder Value", (shooterHoodEncoderValue != 0));

    	maxAngleLimitSwitchValue = CommandBase.shooterHood.GetMaxAngleLimitSwitch();
    	minAngleLimitSwitchValue = CommandBase.shooterHood.GetMinAngleLimitSwitch();

    	SmartDashboard.putBoolean("Shooter Hood MAX Angle Limit Switch Pressedd", (maxAngleLimitSwitchValue != 0));
    	SmartDashboard.putBoolean("Shooter Hood MIN Angle Limit Switch Pressedd", (minAngleLimitSwitchValue != 0));

    	//std.cout << "Max Angle Limit Switch Value: " << maxAngleLimitSwitchValue << std.endl;
    	//std.cout << "Min Angle Limit Switch Value: " << minAngleLimitSwitchValue << std.endl;

    	/*hoodEncoderPluggedIn = CommandBase.shooterHood.HoodEncoderPluggedIn();
    	if (hoodEncoderPluggedIn == false) {
    		std.cout << "Shooter Hood Encoder Disconnected" << std.endl;
    	} */
    }

    // Make this return true when this Command no longer needs to run execute()
    protected boolean isFinished() {
        return false;
    }

    // Called once after isFinished returns true
    protected void end() {
    }

    // Called when another command which requires one or more of the same
    // subsystems is scheduled to run
    protected void interrupted() {
    }
}
