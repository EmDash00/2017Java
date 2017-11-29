package org.usfirst.frc.team9135.command_groups;

import org.usfirst.frc.team9135.robot.commands.*;
import edu.wpi.first.wpilibj.command.Scheduler;

import edu.wpi.first.wpilibj.command.*;



/**
 *
 */
public class AimToGear extends CommandGroup {

	public Boolean sonarGood = true;
	
    public AimToGear() {
    	addSequential(new AimBotWithUltrasonicSensors(sonarGood));
    	addSequential(new WaitTime(.1));
        // Use requires() here to declare subsystem dependencies
        // eg. requires(chassis);
    }


}
