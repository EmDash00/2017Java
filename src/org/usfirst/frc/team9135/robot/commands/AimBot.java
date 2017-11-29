package org.usfirst.frc.team9135.robot.commands;

import org.usfirst.frc.team9135.robot.CommandBase;

import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.*;

/**
 *
 */
public class AimBot extends Command {

	int cameraNumber = 0;
	boolean isbad = false;
	public Timer timer = new Timer();
	public double sonar_value = 0.0;
	public final double CAMERA_TO_GEAR_IN = 11;
	public final double SPRING_IN = 14.5;
	
    public AimBot(int camNumber) 
    {
        // Use requires() here to declare subsystem dependencies
        // eg. requires(chassis);
    	requires(CommandBase.driveTrain);
    	requires(CommandBase.ultrasonicSensor);
    	this.cameraNumber = camNumber;
    }

    // Called just before this Command runs the first timer
    protected void initialize() {
    	System.out.println("running \n\n\n:");
    	double angleToTurn = -CommandBase.server.get_angle(cameraNumber);
    	/*if(angleToTurn == 0) isbad = true;
    	else isbad = false;*/
    	if(cameraNumber == 1)
    	{
    		System.out.println("\n\n\n\n\n\n\n\n\n\nCAMERA NUMBER IS ONE \n\n\n\n\n\n\n\n");
    		if (CommandBase.ultrasonicSensor.usingRightUltrasonicSensorForGearCamera) {
    			sonar_value = CommandBase.ultrasonicSensor.GetGearUltrasonicSensorValueInches();
    		}
    		else {
    			//sonar_value = CommandBase.ultrasonicSensor.GetUltrasonicSensorValueInches(UltrasonicSensor.LEFT_ULTRASONIC_SENSOR);
    		}
    		double dangleToTurn = 90 - atan( (sonar_value - SPRING_IN) / (CAMERA_TO_GEAR_IN - sonar_value * tan((angleToTurn) * M_PI / 180))) * 180 / M_PI;
    		System.out.println("\n\n\n\n\nangle: " + dangleToTurn + "\nSonar: " + sonar_value + "\n\n\n\n" + "\ncam angle " + angleToTurn;
    		SmartDashboard.putNumber("Trig Angle: ", dangleToTurn);
    		CommandBase.driveTrain.TurnPIDEnable(-dangleToTurn);
    	}
    	else
    	{
    		System.out.println("\n\n\n\nCAMERA !\n\n\n\n");
    		CommandBase.driveTrain.TurnPIDEnable(2.5);//- 18);
    	}
    	SmartDashboard.putNumber("Angle to Turn Starting: ", angleToTurn);
    	timer.start();
    	timer.reset();
    	CommandBase.driveTrain.ZeroGyroAngle();
    	CommandBase.driveTrain.is_aiming = true;
    	System.out.println("initialized \n\n");
    }

    // Called repeatedly when this Command is scheduled to run
    protected void execute() {
    	SmartDashboard.PutNumber("Angle to Turn: ", CommandBase.driveTrain.GetGyroAngle());
    	CommandBase.driveTrain.PIDTurning();
    	System.out.println("Time: " + timer.Get());
    	//System.out.println("testing\n");
    }

    // Make this return true when this Command no longer needs to run execute()
    protected boolean isFinished() {
    	//return false;
    	return timer.get() > 3  || isbad == true;
    }

    // Called once after isFinished returns true
    protected void end() {
    	timer.stop();
    	CommandBase.driveTrain.DriveTank(0,0);
    }

    // Called when another command which requires one or more of the same
    // subsystems is scheduled to run
    protected void interrupted() {
    	timer.stop();
    	CommandBase.driveTrain.is_aiming = false;
    }
}
