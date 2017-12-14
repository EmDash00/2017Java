package org.usfirst.frc.team9135.robot.subsystems;

import edu.wpi.first.wpilibj.PowerDistributionPanel;
import edu.wpi.first.wpilibj.command.Subsystem;

/**
 *
 */
public class PDP extends Subsystem {
	PowerDistributionPanel pdp;
	static final int PDP_ID = 0;

    // Put methods for controlling this subsystem
    // here. Call these from Commands.

    public void initDefaultCommand() {
        // Set the default command for a subsystem here.
        //setDefaultCommand(new MySpecialCommand());
    	
    }
    
    void InitializePDP() {
    	pdp = new PowerDistributionPanel(PDP_ID);
    }

    double GetCurrentOfPDPPort(int pdpPort) {
    	return pdp.getCurrent(pdpPort);
    }
}

