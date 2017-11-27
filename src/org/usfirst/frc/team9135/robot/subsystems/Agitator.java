package org.usfirst.frc.team9135.robot.subsystems;

import edu.wpi.first.wpilibj.*;
import org.usfirst.frc.team9135.robot.RobotMap;
import edu.wpi.first.wpilibj.command.Subsystem;


/**
 *
 */
public class Agitator extends Subsystem 
{
	
	VictorSP agitatorMotor;
	VictorSP agitatorMotorPart2;
	
	void InitializeAgitatorMotor(boolean competitionBot) {
		if (competitionBot) {
			agitatorMotor = new VictorSP(RobotMap.CB_AGITATOR_VICTOR_PWM_PORT);
			agitatorMotor.setInverted(RobotMap.CB_AGITATOR_INVERTED);
		}
		else if (competitionBot == false) {
			agitatorMotor = new VictorSP(RobotMap.PB_AGITATOR_VICTOR_PWM_PORT);
			agitatorMotor.setInverted(RobotMap.PB_AGITATOR_INVERTED);
		}
		agitatorMotorPart2 = new VictorSP(RobotMap.CB_AGITATOR_2_VICTOR_PWM_PORT);
		agitatorMotorPart2.setInverted(RobotMap.CB_AGITATOR_2_INVERTED);
	}

	void DriveAgitator(double agitator1MotorPower, double agitator2MotorPower) {
		agitatorMotor.set(agitator1MotorPower);
		agitatorMotorPart2.set(agitator2MotorPower);
	}
    // Put methods for controlling this subsystem
    // here. Call these from Commands.

    public void initDefaultCommand() 
    {
        // Set the default command for a subsystem here.
        //setDefaultCommand(new MySpecialCommand());
    }
}

