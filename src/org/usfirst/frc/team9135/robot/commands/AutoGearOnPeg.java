package org.usfirst.frc.team9135.robot.commands;

import org.usfirst.frc.team9135.robot.CommandBase;
import org.usfirst.frc.team9135.robot.subsystems.*;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.command.Command;

/**
 *
 */
public class AutoGearOnPeg extends Command {
	private double ultrasonicValue = 0.0;

	private boolean startMovingTowardsGear = false;
	private boolean startPuttingGearOnPeg = false;
	private boolean gearHolderDown = false;
	private boolean gearOnPeg = false;

	private static final double DISTANCE_AWAY_FROM_AIRSHIP_TO_DROP_GEAR_IN = 8.5;

	private static final double DRIVE_TRAIN_MOTOR_POWER = .45;

	private double gearHolderMotorPower = 0.0;
	private static final double GEAR_HOLDER_MOTOR_POWER = .75;

	private boolean lowerLimitSwitchValue = false;

	private int rightDriveTrainEncoderRPM = 0;

	private static final int DRIVE_TRAIN_ENCODER_RPM_THRESHOLD = 6;

	private double initialRightEncoderDistance = 0.0;
	private double currentRightEncoderDistance = 0.0;
	private double desiredRightEncoderDistance = 0.0;
	private static final double DISTANCE_TO_MOVE_AWAY_FROM_GEAR = 4.0;  //  In Inches
	private boolean retryGearLineUp = false;
	private boolean moveGearHolderDown = false;
	private boolean stopGearHolderDown = false;
	private boolean initializeTimerLimitSwitch = false;
	static final double WAIT_TIME_FOR_LIMIT_SWITCH_TO_LOWER = 2.0;

	private static final double TIME_TO_LOWER_GEAR_HOLDER = .2;
	private boolean startMovingRobot = true;

	private static final double TIME_ROBOT_IS_ABOVE_DRIVE_TRAIN_RPM_THRESHOLD = .25;
	private boolean startTimerForDriveTrainRPMThreshold = false;
	private boolean waitingForDriveTrainRPMConfigure = false;

	private Timer timer;
	private double timerValue = 0.0;

	private boolean initializeTimerForRamIntoAirship = false;
	private boolean initializeTimerToUnRamFromAirship = false;
	private static final double TIME_TO_RAM_INTO_AIRSHIP = .35;
	private static final double TIME_TO_WAIT_BEFORE_UNRAMMING = .1;
	private static final double TIME_TO_REVERSE_FROM_RAMMING_INTO_AIRSHIP = .35;
	private static final double RAMMING_MOTOR_POWER = -.55;
	private static final double UNRAMMING_MOTOR_POWER = .65;
	private boolean rammedIntoAirship = false;
	private boolean initializeTimeWaitBeforeUnRamming = false;
	private boolean waitTimeForUnRammming = false;

	private int moveGearHolderUpDownCounter = 1;

	private static final boolean RAM_INTO_GEAR_PEG = false;
	
    public AutoGearOnPeg() {
        // Use requires() here to declare subsystem dependencies
        // eg. requires(chassis);
    	requires(CommandBase.driveTrain);
    	requires(CommandBase.ultrasonicSensor);
    	requires(CommandBase.gearHolder);

    	timer = new Timer();
    
    }

    // Called just before this Command runs the first time
    protected void initialize() {
    	startMovingTowardsGear = true;
    	startPuttingGearOnPeg = false;
    	initializeTimerLimitSwitch = false;
    	gearHolderDown = false;
    	gearOnPeg = false;
    	timer.reset();
    	timer.start();

    	startMovingRobot = false;
    	retryGearLineUp = false;
    	moveGearHolderDown = false;
    	stopGearHolderDown = false;
    	startTimerForDriveTrainRPMThreshold = false;
    	waitingForDriveTrainRPMConfigure = false;

    	initializeTimerForRamIntoAirship = false;
    	rammedIntoAirship = false;
    	initializeTimeWaitBeforeUnRamming = false;
    	waitTimeForUnRammming = false;
    	initializeTimerToUnRamFromAirship = false;

    	moveGearHolderUpDownCounter = 1;
    }

    // Called repeatedly when this Command is scheduled to run
    protected void execute() {
    	ultrasonicValue = CommandBase.ultrasonicSensor.GetGearUltrasonicSensorValueInches();
    	timerValue = timer.get();

    	if (ultrasonicValue > 0.0 && ultrasonicValue < 5000.0) {
    		startMovingTowardsGear = true;
    	}

    	if (startMovingTowardsGear) {
    		if (ultrasonicValue > DISTANCE_AWAY_FROM_AIRSHIP_TO_DROP_GEAR_IN) {
    			rightDriveTrainEncoderRPM = (Math.abs(CommandBase.driveTrain.getEncoderRPM(DriveTrain.RIGHT_SIDE_ENCODER)));
    			if ((rightDriveTrainEncoderRPM < DRIVE_TRAIN_ENCODER_RPM_THRESHOLD && startMovingRobot == false) || retryGearLineUp) {
    				if (retryGearLineUp == false) {
    					initialRightEncoderDistance = CommandBase.driveTrain.getDistance(DriveTrain.RIGHT_SIDE_ENCODER);
    					desiredRightEncoderDistance = (initialRightEncoderDistance + DISTANCE_TO_MOVE_AWAY_FROM_GEAR);
    					CommandBase.driveTrain.DriveTank(DRIVE_TRAIN_MOTOR_POWER, DRIVE_TRAIN_MOTOR_POWER);
    					retryGearLineUp = true;
    				}
    				else if (retryGearLineUp) {
    					currentRightEncoderDistance = CommandBase.driveTrain.getDistance(DriveTrain.RIGHT_SIDE_ENCODER);
    					if (currentRightEncoderDistance >= desiredRightEncoderDistance) {
    						CommandBase.driveTrain.DriveTank(0.0, 0.0);
    						moveGearHolderDown = true;
    						retryGearLineUp = false;
    						startMovingRobot = true;
    						startTimerForDriveTrainRPMThreshold = false;
    					}
    					else {
    						CommandBase.driveTrain.DriveTank(DRIVE_TRAIN_MOTOR_POWER, DRIVE_TRAIN_MOTOR_POWER);
    					}
    				}
    			}
    			else {
    				if (moveGearHolderDown) {
    					if (stopGearHolderDown == false) {
    						timer.reset();
    						timer.start();
    						if ((moveGearHolderUpDownCounter % 2) == 1) {
    							gearHolderMotorPower = (-1 * GEAR_HOLDER_MOTOR_POWER);
    						}
    						else if ((moveGearHolderUpDownCounter % 2) == 0) {
    							gearHolderMotorPower = (GEAR_HOLDER_MOTOR_POWER + .25);
    						}
    						CommandBase.gearHolder.DriveGearHolderMotor(gearHolderMotorPower);
    						moveGearHolderUpDownCounter++;
    						stopGearHolderDown = true;
    					}
    					else if (stopGearHolderDown && timerValue < TIME_TO_LOWER_GEAR_HOLDER) {
    						CommandBase.gearHolder.DriveGearHolderMotor(gearHolderMotorPower);
    					}
    					else if (stopGearHolderDown && timerValue >= TIME_TO_LOWER_GEAR_HOLDER) {
    						CommandBase.gearHolder.DriveGearHolderMotor(0.0);
    						timer.reset();
    						timer.stop();
    						stopGearHolderDown = false;
    						moveGearHolderDown = false;
    					}
    				}
    				else {
    					if (rightDriveTrainEncoderRPM >= DRIVE_TRAIN_ENCODER_RPM_THRESHOLD || waitingForDriveTrainRPMConfigure) {
    						if (startTimerForDriveTrainRPMThreshold == false) {
    							timer.reset();
    							timer.start();
    							startMovingRobot = true;
    							waitingForDriveTrainRPMConfigure = true;
    							startTimerForDriveTrainRPMThreshold = true;
    						}
    						else if (startTimerForDriveTrainRPMThreshold) {
    							if (timerValue >= TIME_ROBOT_IS_ABOVE_DRIVE_TRAIN_RPM_THRESHOLD) {
    								startMovingRobot = false;
    								waitingForDriveTrainRPMConfigure = false;
    							}
    							else {
    								startMovingRobot = true;
    								waitingForDriveTrainRPMConfigure = true;
    							}
    						}
    					}
    					else {
    						startMovingRobot = true;
    					}
    					CommandBase.driveTrain.DriveTank(-DRIVE_TRAIN_MOTOR_POWER, -DRIVE_TRAIN_MOTOR_POWER);
    				}
    			}
    		}
    		else if (ultrasonicValue <= DISTANCE_AWAY_FROM_AIRSHIP_TO_DROP_GEAR_IN) {
    			CommandBase.driveTrain.DriveTank(0.0, 0.0);
    			startMovingTowardsGear = false;
    			startPuttingGearOnPeg = true;
    		}
    	}

    	/*if (startPuttingGearOnPeg) {
    		if (initializeTimerLimitSwitch == false) {
    			timer.stop();
    			timer.reset();
    			timer.start();
    			initializeTimerLimitSwitch = true;
    		}

    		timerValue = timer.get();

    		lowerLimitSwitchValue = CommandBase.gearHolder.GetLimitSwitchValue(GearHolder.LOWER_LIMIT_SWITCH_PORT);
    		if (lowerLimitSwitchValue || (timerValue >= WAIT_TIME_FOR_LIMIT_SWITCH_TO_LOWER)) {
    			CommandBase.gearHolder.DriveGearHolderMotor(0.0);
    			startPuttingGearOnPeg = false;
    			gearHolderDown = true;
    			if (RAM_INTO_GEAR_PEG == false) {
    				gearOnPeg = true;
    			}
    		}
    		else {
    			CommandBase.gearHolder.DriveGearHolderMotor(-GEAR_HOLDER_MOTOR_POWER);
    		}
    	} */

    	if (RAM_INTO_GEAR_PEG) {
    		if (gearHolderDown && rammedIntoAirship == false) {
    			if (initializeTimerForRamIntoAirship == false) {
    				timer.stop();
    				timer.reset();
    				timer.start();
    				initializeTimerForRamIntoAirship = true;
    			}

    			timerValue = timer.get();

    			if (timerValue >= TIME_TO_RAM_INTO_AIRSHIP) {
    				CommandBase.driveTrain.DriveTank(0.0, 0.0);
    				timer.stop();
    				timer.reset();
    				rammedIntoAirship = true;
    			}
    			else {
    				CommandBase.driveTrain.DriveTank(RAMMING_MOTOR_POWER, RAMMING_MOTOR_POWER);
    				rammedIntoAirship = false;
    			}
    		}
    		else if (rammedIntoAirship) {
    			if (initializeTimeWaitBeforeUnRamming == false) {
    				timer.stop();
    				timer.reset();
    				timer.start();
    				initializeTimeWaitBeforeUnRamming = true;
    			}
    			else if (initializeTimeWaitBeforeUnRamming && waitTimeForUnRammming == false) {
    				timerValue = timer.get();
    				if (timerValue >= TIME_TO_WAIT_BEFORE_UNRAMMING) {
    					waitTimeForUnRammming = true;
    				}
    				CommandBase.driveTrain.DriveTank(0.0, 0.0);
    			}

    			if (waitTimeForUnRammming) {
    				if (initializeTimerToUnRamFromAirship == false) {
    					timer.stop();
    					timer.reset();
    					timer.start();
    					initializeTimerToUnRamFromAirship = true;
    				}

    				timerValue = timer.get();
    				if (timerValue >= TIME_TO_REVERSE_FROM_RAMMING_INTO_AIRSHIP) {
    					CommandBase.driveTrain.DriveTank(0.0, 0.0);
    					timer.stop();
    					timer.reset();
    					gearOnPeg = true;
    				}
    				else {
    					CommandBase.driveTrain.DriveTank(UNRAMMING_MOTOR_POWER, UNRAMMING_MOTOR_POWER);
    					gearOnPeg = false;
    				}
    			}
    		}
    	}
    }

    // Make this return true when this Command no longer needs to run execute()
    protected boolean isFinished() {
        return false;
    }

    // Called once after isFinished returns true
    protected void end() {
    	System.out.println("Done with Auto Gear On Peg");
    	startMovingTowardsGear = true;
    	startPuttingGearOnPeg = false;
    	initializeTimerLimitSwitch = false;
    	gearHolderDown = false;
    	gearOnPeg = false;
    	CommandBase.driveTrain.DriveTank(0.0, 0.0);
    	CommandBase.gearHolder.DriveGearHolderMotor(0.0);
    	startMovingRobot = false;
    	retryGearLineUp = false;
    	moveGearHolderDown = false;
    	stopGearHolderDown = false;
    	startTimerForDriveTrainRPMThreshold = false;
    	waitingForDriveTrainRPMConfigure = false;
    	initializeTimerForRamIntoAirship = false;
    	rammedIntoAirship = false;
    	initializeTimeWaitBeforeUnRamming = false;
    	waitTimeForUnRammming = false;
    	initializeTimerToUnRamFromAirship = false;

    	moveGearHolderUpDownCounter = 1;
    }

    // Called when another command which requires one or more of the same
    // subsystems is scheduled to run
    protected void interrupted() {
    	end();
    }
}
