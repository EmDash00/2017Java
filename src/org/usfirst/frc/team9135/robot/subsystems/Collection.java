package org.usfirst.frc.team9135.robot.subsystems;

import edu.wpi.first.wpilibj.command.Subsystem;
import edu.wpi.first.wpilibj.*;
import org.usfirst.frc.team9135.robot.RobotMap;

/**
 *
 */

	// It's desirable that everything possible under private except
	// for methods that implement subsystem capabilities
public class Collection extends Subsystem 
{
	
	private VictorSP collectionMotor;
	boolean autoDriveCollection = false;

    // Put methods for controlling this subsystem
    // here. Call these from Commands.

	void InitDefaultCommand() {
		// Set the default command for a subsystem here.
		// SetDefaultCommand(new MySpecialCommand());
	}

	protected void InitializeCollectionMotor(boolean competitionBot) {
		if (competitionBot) {
			collectionMotor = new VictorSP(RobotMap.CB_COLLECTION_VICTOR_PWM_PORT);
			collectionMotor.setInverted(RobotMap.CB_COLLECTION_INVERTED);
		}
		else if (competitionBot == false) {
			collectionMotor = new VictorSP(RobotMap.PB_COLLECTION_VICTOR_PWM_PORT);
			collectionMotor.setInverted(RobotMap.PB_COLLECTION_INVERTED);
		}
	}

	public void DriveCollection(double motorPower) {
		collectionMotor.set(motorPower);
	}

	protected void SetAutoDriveCollection(boolean autoDriveCollection) {
		this.autoDriveCollection = autoDriveCollection;
	}

	protected boolean GetAutoDriveCollection() {
		return (this.autoDriveCollection);
	}
	
    protected void initDefaultCommand() {
        // Set the default command for a subsystem here.
        //setDefaultCommand(new MySpecialCommand());
    }
}

