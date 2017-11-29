package org.usfirst.frc.team9135.command_groups;

import edu.wpi.first.wpilibj.Preferences;
import edu.wpi.first.wpilibj.command.CommandGroup;
import org.usfirst.frc.team9135.robot.commands.*;
import org.usfirst.frc.team9135.robot.CommandBase;

/**
 *
 */
public class AutonomousCommand extends CommandGroup {
	//  Robot Dimensions with Bumpers
	private static final double ROBOT_WITH_BUMPERS_LENGTH = 39.0;
	private static final double ROBOT_WITH_BUMPERS_WIDTH = 36.0;

	//  Simple Base Line Autonomous Path
	private static final double BASE_LINE_PATH_DISTANCE = 85.0;

	private static final double RIGHT_SIDE_GEAR_DISTANCE_PART_1 = 37.0;
	private static final double LEFT_SIDE_GEAR_DISTANCE_PART_1 = 36.0;
	private static final double SIDE_GEAR_DISTANCE_PART_2 = 20.0;
	private static final double SIDE_GEAR_DISTANCE_PART_3 = 20.0;

	//  Gear Autonomous Programs Distances
	private static final double DISTANCE_FROM_ALLIANCE_WALL_TO_MIDDLE_GEAR = (80.0 - ROBOT_WITH_BUMPERS_LENGTH);
	private static final double DISTANCE_FROM_ALLIANCE_WALL_TO_RIGHT_GEAR_PART_1 = (93.0 - ROBOT_WITH_BUMPERS_LENGTH);
	private static final double DISTANCE_FROM_ALLIANCE_WALL_TO_LEFT_GEAR_PART_1 = (91.0 - ROBOT_WITH_BUMPERS_LENGTH);
	private static final double DISTANCE_FROM_ALLIANCE_WALL_TO_SIDE_GEAR_PART_2 = 20.0;

	private static final double ANGLE_TO_TURN_TO_FACE_SIDE_GEAR = 42.0;
	private static final double ANGLE_FROM_SIDE_GEAR_TO_BASE_LINE_PATH = 75.0;

	private static final double DISTANCE_TO_MOVE_AWAY_FROM_GEAR_AFTER_PLACING_PART_1 = 11.0;
	private static final double DISTANCE_TO_MOVE_AWAY_FROM_GEAR_AFTER_PLACING_PART_2 = 6.0;
	private static final double DISTANCE_TO_MOVE_AWAY_FROM_GEAR_AFTER_PLACING_PART_3 = 8.0;

	private static final double DISTANCE_TO_MOVE_AWAY_FROM_SIDE_GEAR_TO_PURSUE_BASELINE = 30.0;
	private static final double DISTANCE_OF_SIDE_GEAR_BASE_LINE_PATH = 36.0;

	private static final double DISTANCE_AFTER_TURNING_ONTO_SIDE_GEAR = 13.5;

	//  Gear Autonomous Program and Neutral Zone Variables
	private static final double ANGLE_TO_TURN_TO_FACE_NEUTRAL_ZONE = 42.0;
	private static final double DISTANCE_TO_TRAVEL_TOWARDS_AND_INTO_NEUTRAL_ZONE = 220.0;

	//  Gear Autonomous Program With Lidar Variables
	private static final double DISTANCE_TO_TRAVEL_UNITL_TURNING_PARALLEL_WITH_AIRSHP = 45.0;
	private static final double ANGLE_TO_TURN_TO_BE_PARALLEL_WITH_AIRSHIP = 30.0;
	private static final double DISTANCE_TO_TRAVEL_TO_START_LIDAR_DETECTING = 10.0;

	//  Shooter Autonomous Programs
	//  Both Close Shot Right and Close Shot Left
	private static final double DISTANCE_TO_TRAVEL_TO_BASELINE_AFTER_CLOSE_SHOT = 70.0;
	//  Close Shot Right
	private static final double DISTANCE_TO_TRAVEL_BACKWARDS_AFTER_CLOSE_SHOT = 15.0;
	private static final double ANGLE_TO_TURN_AFTER_DRIVING_OFF_RIGHT_ALLIANCE_WALL = 38.0;
	//  Close Shot Left
	private static final double ANGLE_TO_TURN_AFTER_DRIVING_OFF_LEFT_ALLIANCE_WALL = 15.0;
	private static final double DISTANCE_TO_TRAVEL_OFF_LEFT_ALLIANCE_WALL = 12.0;
	private static final double ANGLE_TO_TURN_AFTER_TURNING_OFF_LEFT_ALLIANCE_WALL = 45.0;
	private static final double DISTANCE_TO_TRAVEL_ABOUT_PARALLEL_WITH_LEFT_ALLIANCE_WALL = 20.0;
	private static final double ANGLE_TO_TURN_TO_DRIVE_TO_BASELINE_AFTER_LEFT_ALLIANCE_WALL = 50.0;

	//  Close Shot Right With Gear Variables
	private static final double DISTANCE_TO_TRAVEL_OFF_OF_BOILER_FOR_RIGHT_CLOSE_SHOT_AND_GEAR = 5.0;
	private static final double ANGLE_TO_TURN_TOWARDS_RIGHT_GEAR = 15.0;
	private static final double DISTANCE_TO_DRIVE_BEFORE_TURNING_ONTO_RIGHT_GEAR = 54.0;
	private static final double ANGLE_TO_TURN_TOWARDS_RIGHT_GEAR_SECOND_TURN = 18.0;
	private static final double DISTANCE_TO_TRAVEL_TOWARDS_RIGHT_GEAR_PEG_FOR_CLOSE_SHOT = 12.0;

	//  Close Shot Left With Gear Variables
	private static final double DISTANCE_TO_TRAVEL_OFF_OF_LEFT_ALLIANCE_WALL = 28.0;
	private static final double ANGLE_TO_TURN_FOR_ROBOT_TO_BE_PERPENDICULAR_WITH_BOILER = 40.0;
	private static final double TIME_FOR_DRIVE_TRAIN_TO_DRIVE_INTO_BOILER = 1.0;
	private static final double TIME_TO_RUN_AGITATOR_FOR_LEFT_CLOSE_SHOT_AND_GEAR = 5.5;
	private static final double DISTANCE_TO_DRIVE_AWAY_FROM_BOILER = 6.0;
	private static final double ANGLE_TO_TURN_AFTER_DRIVING_AWAY_FROM_BOILER = 15.0;
	private static final double DISTANCE_TO_DRIVE_TOWARDS_LEFT_GEAR_AIRSHIP = 50.0;
	private static final double ANGLE_TO_TURN_TOWARDS_LEFT_GEAR_PEG = 18.0;
	private static final double DISTANCE_TO_TRAVEL_TOWARDS_LEFT_GEAR_PEG_FOR_CLOSE_SHOT = 12.0;

	//  Both Shoot After Right Gear and Shoot After Left Gear
	private static final double DISTANCE_TO_TRAVEL_AFTER_PLACING_SIDE_GEAR = 21.0;
	private static final double DISTANCE_TO_TRAVEL_AFTER_TURNING_TO_DRIVE_TOWARDS_BOILER = 35.0;
	//  Shoot After Right Gear
	private static final double ANGLE_TO_TURN_AFTER_DRIVING_AWAY_FROM_RIGHT_GEAR = 18.0;//18.0;
	//  Shoot After Left Gear
	private static final double ANGLE_TO_TURN_AFTER_DRIVING_AWAY_FROM_LEFT_GEAR = 14.0;

	//  Both Shoot Right After Placing Middle Gear and Shoot Left After Placing Middle Gear
	private static final double DISTANCE_TO_MOVE_AWAY_FROM_GEAR_AFTER_PLACING_TO_SHOOT = 25.0;
	private static final double DISTANCE_LIDAR_AWAY_FROM_SIDE_RAIL = 80.0;
	//  Shoot Right After Placing Middle Gear
	private static final double ANGLE_TO_TURN_TO_FACE_BOILDER_RIGHT_SHOOT_GEAR = 25.0;
	//  Shoot Left After Placing Middle Gear
	private static final double ANGLE_TO_TURN_TO_FACE_BOILDER_LEFT_SHOOT_GEAR = 20.0;

	//  40 KPa Autonomous Variables Option 1
	private static final double DISTANCE_FROM_ALLIANCE_WALL_TO_MIDDLE_OF_HOPPER_PANEL_PART_1 = 12.0;  //  TBD
	private static final double DISTANCE_FROM_ALLIANCE_WALL_TO_MIDDLE_OF_HOPPER_PANEL_PART_2 = 55.0;  //  TBD
	private static final double DISTANCE_FROM_ALLIANCE_WALL_TO_MIDDLE_OF_HOPPER_PANEL_PART_3 = 14.0;
	private static final double ANGLE_TO_TURN_INTO_HOPPER_PANEL = 45.0;  //  TBD
	private static final double ANGLE_TO_TURN_OUT_OF_HOPPER_PANEL = 20.0;  //  TBD

	//  40 KPa Autonomous Variables Option 2
	private static final double DISTANCE_TO_DRIVE_FROM_ALLIANCE_WALL_TO_GUARDRAIL = 20.0;
	private static final double ANGLE_TO_TURN_TO_BE_PARALLEL_WITH_GUARDRAIL = 40.0;
	private static final double DISTANCE_TO_DRIVE_TOWARDS_HOPPER = 12.0;
	private static final double DISTANCE_AWAY_FROM_GUARDRAIL_TO_DRIVE = 6.0;
	private static final double ANGLE_TO_TURN_TO_LINE_UP_TO_HIT_PANEL = 8.0;
	private static final double ANGLE_TO_TURN_AWAY_FROM_HOPPER_PANEL = 40.0;
	private static final double ANGLE_TO_TURN_TOWARDS_HOPPER = 32.0;

	private static final double TIME_TO_RUN_AGITATOR_GEAR_AND_SHOOT = 12.0;
	private static final double TIME_TO_RUN_AGITATOR_HOPPER_AND_SHOOT = 12.0;

	private static final double RIGHT_HOPPER_AND_SHOOT = 1;
	private static final double LEFT_HOPPER_AND_SHOOT = -RIGHT_HOPPER_AND_SHOOT;

	//  General Variables
	private static final boolean TURN_RIGHT = true;
	private static final boolean TURN_LEFT = !TURN_RIGHT;

	private static final boolean DRIVE_RIGHT_SIDE_DRIVE_TRAIN = true;
	private static final boolean DRIVE_LEFT_SIDE_DRIVE_TRAIN = !DRIVE_RIGHT_SIDE_DRIVE_TRAIN;

	private static final boolean RIGHT_GEAR = true;
	private static final boolean LEFT_GEAR = !RIGHT_GEAR;

	private static final int GEAR_CAMERA = 1;
	private static final int SHOOTER_CAMERA = 0;

	private static final double RIGHT_ANGLE_DEGREES = 90.0;

	//  Booleans for Autonomous Operation
	private static final boolean USING_AUTO_ROTATE_FOR_GEAR_PEG = true;

	private static final boolean USING_GEAR_CAMERA = true;

	private static final double ANGLE_TO_ROTATE_FOR_GEAR_PEG = 2.0;
	private static final double AUTO_ROTATE_FOR_GEAR_PEG_MOTOR_POWER = .5;

	private static final double MIDDLE_GEAR_LIDAR_VALUE_TO_TRAVEL_UNITL = 42.0;
	private static final double SIDE_GEAR_LIDAR_VALUE_TO_TRAVEL_UNTIL = 87.0;  //  85.0

	private static final boolean AUTO_MOVE_GEAR_HOLDER_UPWARDS = true;
	private static final boolean AUTO_MOVE_GEAR_HOLDER_DOWNWARDS = !AUTO_MOVE_GEAR_HOLDER_UPWARDS;

	private static final boolean CLOSE_SHOT_PID_SLOT = true;
	private static final boolean FAR_SHOT_PID_SLOT = !CLOSE_SHOT_PID_SLOT;

	private boolean baseLine = false;
	private boolean middleGear = false;
	private boolean middleGearShootRight = false;
	private boolean middleGearShootLeft = false;
	private boolean rightGear = false;
	private boolean leftGear = false;
	private boolean rightGearAndShoot = false;
	private boolean leftGearAndShoot = false;
	private boolean rightGearAndNeutralZone = false;
	private boolean leftGearAndNeutralZone = false;
	private boolean closeShotRightWithBaseline = false;
	private boolean closeShotRightWithGear = false;
	private boolean closeShotLeftWithBaseline = false;
	private boolean closeShotLeftWithGear = false;
	private boolean right40KPa = false;
	private boolean left40KPa = false;
	
	
	AutonomousCommand() {
		// Add Commands here:
		// e.g. addSequential(new Command1());
		//      addSequential(new Command2());
		// these will run in order.

		// To run multiple commands at the same time,
		// use addParallel()
		// e.g. addParallel(new Command1());
		//      addSequential(new Command2());
		// Command1 and Command2 will run in parallel.

		// A command group will require all of the subsystems that each member
		// would require.
		// e.g. if Command1 requires chassis, and Command2 requires arm,
		// a CommandGroup containing them would require both the chassis and the
		// arm.

		requires(CommandBase.driveTrain);
		requires(CommandBase.gearHolder);
		requires(CommandBase.ultrasonicSensor);
		requires(CommandBase.shooter);
		requires(CommandBase.agitator);
		requires(CommandBase.lidars);
		requires(CommandBase.liftHang);
		requires(CommandBase.collection);

		baseLine = Preferences.getInstance().getBoolean("BaseLine", false);
		middleGear = Preferences.getInstance().getBoolean("Middle Gear", false);
		middleGearShootRight = Preferences.getInstance().getBoolean("Middle Gear Shoot Right", false);
		middleGearShootLeft = Preferences.getInstance().getBoolean("Middle Gear Shoot Left", false);
		rightGear = Preferences.getInstance().getBoolean("Right Gear", false);
		leftGear = Preferences.getInstance().getBoolean("Left Gear", false);
		rightGearAndShoot = Preferences.getInstance().getBoolean("Right Gear and Shoot", false);
		leftGearAndShoot = Preferences.getInstance().getBoolean("Left Gear and Shoot", false);
		rightGearAndNeutralZone = Preferences.getInstance().getBoolean("Right Gear and Neutral Zone", false);
		leftGearAndNeutralZone = Preferences.getInstance().getBoolean("Left Gear and Neutral Zone", false);
		closeShotRightWithBaseline = Preferences.getInstance().getBoolean("Close Shot Right With Baseline", false);
		closeShotRightWithGear = Preferences.getInstance().getBoolean("Close Shot Right With Gear", false);
		closeShotLeftWithBaseline = Preferences.getInstance().getBoolean("Close Shot Left With Baseline", false);
		closeShotLeftWithGear = Preferences.getInstance().getBoolean("Close Shot Left With Gear", false);
		right40KPa = Preferences.getInstance().getBoolean("Right 40KPa", false);
		left40KPa = Preferences.getInstance().getBoolean("Left 40KPa", false);

		if (baseLine) {
			addSequential(new DriveDistance(BASE_LINE_PATH_DISTANCE, .7));
			addParallel(new GetReadyForLiftHang());
		}
		else if (middleGear) {
			addSequential(new DriveDistance(DISTANCE_FROM_ALLIANCE_WALL_TO_MIDDLE_GEAR, -.55));
			addParallel(new GetReadyForLiftHang());
			addSequential(new WaitTime(.25));
			addSequential(new AutoGearOnPeg());
			addSequential(new WaitTime(.001));
			addParallel(new AutoMoveGearHolder(AUTO_MOVE_GEAR_HOLDER_DOWNWARDS));
			addSequential(new WaitTime(.6));
			addSequential(new AutoRotateRobotForGearPeg(AUTO_ROTATE_FOR_GEAR_PEG_MOTOR_POWER, ANGLE_TO_ROTATE_FOR_GEAR_PEG));
			addSequential(new WaitTime(.25));
			addSequential(new DriveDistance(DISTANCE_TO_MOVE_AWAY_FROM_GEAR_AFTER_PLACING_PART_1, .175));
			addSequential(new DriveDistance(DISTANCE_TO_MOVE_AWAY_FROM_GEAR_AFTER_PLACING_PART_2, .175));
			addParallel(new AutoMoveGearHolder(AUTO_MOVE_GEAR_HOLDER_UPWARDS));
			addSequential(new DriveDistance(DISTANCE_TO_MOVE_AWAY_FROM_GEAR_AFTER_PLACING_PART_3, .4));
		}
		else if (middleGearShootRight) {
			addSequential(new DriveDistance(DISTANCE_FROM_ALLIANCE_WALL_TO_MIDDLE_GEAR, -.55));
			addParallel(new GetReadyForLiftHang());
			addSequential(new WaitTime(.25));
			addSequential(new AutoGearOnPeg());
			addSequential(new WaitTime(.001));
			addParallel(new AutoMoveGearHolder(AUTO_MOVE_GEAR_HOLDER_DOWNWARDS));
			addSequential(new WaitTime(.6));
			addSequential(new AutoRotateRobotForGearPeg(AUTO_ROTATE_FOR_GEAR_PEG_MOTOR_POWER, ANGLE_TO_ROTATE_FOR_GEAR_PEG));
			addSequential(new WaitTime(.25));
			addSequential(new DriveDistance(DISTANCE_TO_MOVE_AWAY_FROM_GEAR_AFTER_PLACING_PART_1, .175));
			addSequential(new DriveDistance(DISTANCE_TO_MOVE_AWAY_FROM_GEAR_AFTER_PLACING_PART_2, .175));
			addParallel(new AutoMoveGearHolder(AUTO_MOVE_GEAR_HOLDER_UPWARDS));
			addSequential(new DriveDistance(DISTANCE_TO_MOVE_AWAY_FROM_GEAR_AFTER_PLACING_PART_3, .4));
			addSequential(new DriveDistance(DISTANCE_TO_MOVE_AWAY_FROM_GEAR_AFTER_PLACING_TO_SHOOT, .65));
			addSequential(new WaitTime(.2));
			addSequential(new TurnDriveTrainAngle(RIGHT_ANGLE_DEGREES, .65, TURN_LEFT));
			addSequential(new WaitTime(.15));
			addSequential(new DriveDistance(DISTANCE_TO_TRAVEL_AFTER_TURNING_TO_DRIVE_TOWARDS_BOILER, .7));
			addParallel(new AutoGetShooterUpToSpeed(Shooter.SHOOTER_SETPOINT_RPM_FAR_SHOT, FAR_SHOT_PID_SLOT));
			addSequential(new WaitTime(.2));
			addSequential(new TurnDriveTrainAngle(ANGLE_TO_TURN_TO_FACE_BOILDER_RIGHT_SHOOT_GEAR, .65, TURN_RIGHT));
			addSequential(new WaitTime(.2));
			//addSequential(new AimBot(SHOOTER_CAMERA));
			/*double angle = -CommandBase.server.get_angle(SHOOTER_CAMERA);
			if(angle > 0)
				addSequential(new TurnOneSideOfRobotAngle(angle,false,.5));
			else
				addSequential(new TurnOneSideOfRobotAngle(angle,true,.5)); */
			addSequential(new AutoDriveAgitator(TIME_TO_RUN_AGITATOR_GEAR_AND_SHOOT));
		}
		else if (middleGearShootLeft) {
			addSequential(new DriveDistance(DISTANCE_FROM_ALLIANCE_WALL_TO_MIDDLE_GEAR, -.55));
			addParallel(new GetReadyForLiftHang());
			addSequential(new WaitTime(.25));
			addSequential(new AutoGearOnPeg());
			addSequential(new WaitTime(.001));
			addParallel(new AutoMoveGearHolder(AUTO_MOVE_GEAR_HOLDER_DOWNWARDS));
			addSequential(new WaitTime(.6));
			addSequential(new AutoRotateRobotForGearPeg(AUTO_ROTATE_FOR_GEAR_PEG_MOTOR_POWER, ANGLE_TO_ROTATE_FOR_GEAR_PEG));
			addSequential(new WaitTime(.25));
			addSequential(new DriveDistance(DISTANCE_TO_MOVE_AWAY_FROM_GEAR_AFTER_PLACING_PART_1, .175));
			addSequential(new DriveDistance(DISTANCE_TO_MOVE_AWAY_FROM_GEAR_AFTER_PLACING_PART_2, .175));
			addParallel(new AutoMoveGearHolder(AUTO_MOVE_GEAR_HOLDER_UPWARDS));
			addSequential(new DriveDistance(DISTANCE_TO_MOVE_AWAY_FROM_GEAR_AFTER_PLACING_PART_3, .4));
			addSequential(new DriveDistance(DISTANCE_TO_MOVE_AWAY_FROM_GEAR_AFTER_PLACING_TO_SHOOT, .65));
			addSequential(new WaitTime(.2));
			addSequential(new TurnDriveTrainAngle(RIGHT_ANGLE_DEGREES, .65, TURN_RIGHT));
			addSequential(new WaitTime(.15));
			addSequential(new DriveDistance(DISTANCE_TO_TRAVEL_AFTER_TURNING_TO_DRIVE_TOWARDS_BOILER, .7));
			addParallel(new AutoGetShooterUpToSpeed(Shooter.SHOOTER_SETPOINT_RPM_FAR_SHOT, FAR_SHOT_PID_SLOT));
			addSequential(new WaitTime(.2));
			addSequential(new TurnDriveTrainAngle(ANGLE_TO_TURN_TO_FACE_BOILDER_LEFT_SHOOT_GEAR, .65, TURN_LEFT));
			addSequential(new WaitTime(.2));
			//addSequential(new AimBot(SHOOTER_CAMERA));
			/*double angle = -CommandBase.server.get_angle(SHOOTER_CAMERA);
			if(angle > 0)
				addSequential(new TurnOneSideOfRobotAngle(angle,false,.5));
			else
				addSequential(new TurnOneSideOfRobotAngle(angle,true,.5)); */
			addSequential(new AutoDriveAgitator(TIME_TO_RUN_AGITATOR_GEAR_AND_SHOOT));
		}
		else if (rightGear) {
			addSequential(new DriveDistance(RIGHT_SIDE_GEAR_DISTANCE_PART_1, -.65));
			addSequential(new DriveDistance(SIDE_GEAR_DISTANCE_PART_2, -.25));
			addSequential(new DriveDistance(SIDE_GEAR_DISTANCE_PART_3, -.15));
			addParallel(new GetReadyForLiftHang());
			addSequential(new WaitTime(.2));
			addSequential(new TurnDriveTrainAngle(ANGLE_TO_TURN_TO_FACE_SIDE_GEAR, .65, TURN_LEFT));
			addSequential(new WaitTime(.25));
			addSequential(new CameraAdjustWhileDrivingToGear());
			/*if (USING_GEAR_CAMERA) {
				addSequential(new WaitTime(.25));
				addSequential(new CameraAdjustWhileDrivingToGear());
			}
			else if (USING_GEAR_CAMERA == false) {
				addSequential(new WaitTime(.1));
				addSequential(new DriveDistance(DISTANCE_AFTER_TURNING_ONTO_SIDE_GEAR, -.4));
				addSequential(new WaitTime(.15));
			}*/
			addSequential(new AutoGearOnPeg());
			addSequential(new WaitTime(.001));
			addParallel(new AutoMoveGearHolder(AUTO_MOVE_GEAR_HOLDER_DOWNWARDS));
			addSequential(new WaitTime(.6));
			addSequential(new AutoRotateRobotForGearPeg(AUTO_ROTATE_FOR_GEAR_PEG_MOTOR_POWER, ANGLE_TO_ROTATE_FOR_GEAR_PEG));
			//if (USING_AUTO_ROTATE_FOR_GEAR_PEG) { addSequential(new WaitTime(.1)); addSequential(new AutoRotateRobotForGearPeg(AUTO_ROTATE_FOR_GEAR_PEG_MOTOR_POWER, ANGLE_TO_ROTATE_FOR_GEAR_PEG)); }
			addSequential(new WaitTime(.25));
			addSequential(new DriveDistance(DISTANCE_TO_MOVE_AWAY_FROM_GEAR_AFTER_PLACING_PART_1, .175));
			addSequential(new DriveDistance(DISTANCE_TO_MOVE_AWAY_FROM_GEAR_AFTER_PLACING_PART_2, .175));
			addParallel(new AutoMoveGearHolder(AUTO_MOVE_GEAR_HOLDER_UPWARDS));
			addSequential(new DriveDistance(DISTANCE_TO_MOVE_AWAY_FROM_GEAR_AFTER_PLACING_PART_3, .4));
		}
		else if (rightGearAndShoot) {
			addSequential(new DriveDistance(RIGHT_SIDE_GEAR_DISTANCE_PART_1, -.65));
			addParallel(new GetReadyForLiftHang());
			addSequential(new DriveDistance(SIDE_GEAR_DISTANCE_PART_2, -.25));
			addSequential(new DriveDistance(SIDE_GEAR_DISTANCE_PART_3, -.15));
			addSequential(new WaitTime(.2));
			addSequential(new TurnDriveTrainAngle(ANGLE_TO_TURN_TO_FACE_SIDE_GEAR, .65, TURN_LEFT));
			addSequential(new WaitTime(.25));
			addSequential(new CameraAdjustWhileDrivingToGear());
			/*if (USING_GEAR_CAMERA) {
				addSequential(new WaitTime(.25));
				addSequential(new CameraAdjustWhileDrivingToGear());
			}
			else if (USING_GEAR_CAMERA == false) {
				addSequential(new WaitTime(.1));
				addSequential(new DriveDistance(DISTANCE_AFTER_TURNING_ONTO_SIDE_GEAR, -.4));
				addSequential(new WaitTime(.15));
			}*/
			addSequential(new AutoGearOnPeg());
			addSequential(new WaitTime(.001));
			addParallel(new AutoMoveGearHolder(AUTO_MOVE_GEAR_HOLDER_DOWNWARDS));
			addSequential(new WaitTime(.6));
			addSequential(new AutoRotateRobotForGearPeg(AUTO_ROTATE_FOR_GEAR_PEG_MOTOR_POWER, ANGLE_TO_ROTATE_FOR_GEAR_PEG));
			//if (USING_AUTO_ROTATE_FOR_GEAR_PEG) { addSequential(new WaitTime(.1)); addSequential(new AutoRotateRobotForGearPeg(AUTO_ROTATE_FOR_GEAR_PEG_MOTOR_POWER, ANGLE_TO_ROTATE_FOR_GEAR_PEG)); }
			addSequential(new WaitTime(.25));
			addSequential(new DriveDistance(DISTANCE_TO_MOVE_AWAY_FROM_GEAR_AFTER_PLACING_PART_1, .175));
			addSequential(new DriveDistance(DISTANCE_TO_MOVE_AWAY_FROM_GEAR_AFTER_PLACING_PART_2, .175));
			addParallel(new AutoMoveGearHolder(AUTO_MOVE_GEAR_HOLDER_UPWARDS));
			addSequential(new DriveDistance(DISTANCE_TO_MOVE_AWAY_FROM_GEAR_AFTER_PLACING_PART_3, .4));
			addSequential(new TurnDriveTrainAngle(ANGLE_TO_TURN_AFTER_DRIVING_AWAY_FROM_RIGHT_GEAR, .55, TURN_RIGHT));
			addParallel(new AutoGetShooterUpToSpeed(Shooter.SHOOTER_SETPOINT_RPM_FAR_SHOT, FAR_SHOT_PID_SLOT));
			addSequential(new WaitTime(.2));
			addSequential(new DriveDistance(DISTANCE_TO_TRAVEL_AFTER_PLACING_SIDE_GEAR, .6));
			addSequential(new WaitTime(.15));
			//addSequential(new AimBot(SHOOTER_CAMERA));
			/*double angle = -CommandBase.server.get_angle(SHOOTER_CAMERA);
			if(angle > 0)
				addSequential(new TurnOneSideOfRobotAngle(angle,false,.5));
			else
				addSequential(new TurnOneSideOfRobotAngle(angle,true,.5)); */
			addSequential(new AutoDriveAgitator(TIME_TO_RUN_AGITATOR_GEAR_AND_SHOOT));
		}
		else if (rightGearAndNeutralZone) {
			addSequential(new DriveDistance(RIGHT_SIDE_GEAR_DISTANCE_PART_1, -.65));
			addParallel(new GetReadyForLiftHang());
			addSequential(new DriveDistance(SIDE_GEAR_DISTANCE_PART_2, -.25));
			addSequential(new DriveDistance(SIDE_GEAR_DISTANCE_PART_3, -.15));
			addSequential(new WaitTime(.2));
			addSequential(new TurnDriveTrainAngle(ANGLE_TO_TURN_TO_FACE_SIDE_GEAR, .65, TURN_LEFT));
			addSequential(new WaitTime(.1));
			addSequential(new DriveDistance(DISTANCE_AFTER_TURNING_ONTO_SIDE_GEAR, -.4));
			addSequential(new WaitTime(.15));
			/*if (USING_GEAR_CAMERA) {
				addSequential(new WaitTime(.25));
				addSequential(new CameraAdjustWhileDrivingToGear());
			}
			else if (USING_GEAR_CAMERA == false) {
				addSequential(new WaitTime(.1));
				addSequential(new DriveDistance(DISTANCE_AFTER_TURNING_ONTO_SIDE_GEAR, -.4));
				addSequential(new WaitTime(.15));
			}*/
			addSequential(new AutoGearOnPeg());
			addSequential(new WaitTime(.001));
			addParallel(new AutoMoveGearHolder(AUTO_MOVE_GEAR_HOLDER_DOWNWARDS));
			addSequential(new WaitTime(.6));
			addSequential(new AutoRotateRobotForGearPeg(AUTO_ROTATE_FOR_GEAR_PEG_MOTOR_POWER, ANGLE_TO_ROTATE_FOR_GEAR_PEG));
			addSequential(new WaitTime(.25));
			addSequential(new DriveDistance(DISTANCE_TO_MOVE_AWAY_FROM_GEAR_AFTER_PLACING_PART_1, .175));
			addSequential(new DriveDistance(DISTANCE_TO_MOVE_AWAY_FROM_GEAR_AFTER_PLACING_PART_2, .175));
			addParallel(new AutoMoveGearHolder(AUTO_MOVE_GEAR_HOLDER_UPWARDS));
			addSequential(new DriveDistance((DISTANCE_TO_MOVE_AWAY_FROM_GEAR_AFTER_PLACING_PART_3 + 8.0), .4));
			addSequential(new WaitTime(.2));
			addSequential(new TurnDriveTrainAngle(ANGLE_TO_TURN_TO_FACE_NEUTRAL_ZONE, .65, TURN_RIGHT));
			addSequential(new WaitTime(.15));
			addSequential(new DriveDistance(DISTANCE_TO_TRAVEL_TOWARDS_AND_INTO_NEUTRAL_ZONE, -.65));
		}
		else if (leftGear) {
			addSequential(new DriveDistance(LEFT_SIDE_GEAR_DISTANCE_PART_1, -.65));
			addParallel(new GetReadyForLiftHang());
			addSequential(new DriveDistance(SIDE_GEAR_DISTANCE_PART_2, -.25));
			addSequential(new DriveDistance(SIDE_GEAR_DISTANCE_PART_3, -.15));
			addSequential(new WaitTime(.2));
			addSequential(new TurnDriveTrainAngle((ANGLE_TO_TURN_TO_FACE_SIDE_GEAR), .65, TURN_RIGHT));
			if (USING_GEAR_CAMERA) {
				addSequential(new WaitTime(.25));
				addSequential(new CameraAdjustWhileDrivingToGear());
			}
			else if (USING_GEAR_CAMERA == false) {
				addSequential(new WaitTime(.1));
				addSequential(new DriveDistance(DISTANCE_AFTER_TURNING_ONTO_SIDE_GEAR, -.4));
				addSequential(new WaitTime(.15));
			}
			addSequential(new AutoGearOnPeg());
			addSequential(new WaitTime(.001));
			addParallel(new AutoMoveGearHolder(AUTO_MOVE_GEAR_HOLDER_DOWNWARDS));
			addSequential(new WaitTime(.6));
			addSequential(new AutoRotateRobotForGearPeg(AUTO_ROTATE_FOR_GEAR_PEG_MOTOR_POWER, ANGLE_TO_ROTATE_FOR_GEAR_PEG));
			addSequential(new WaitTime(.25));
			addSequential(new DriveDistance(DISTANCE_TO_MOVE_AWAY_FROM_GEAR_AFTER_PLACING_PART_1, .175));
			addSequential(new DriveDistance(DISTANCE_TO_MOVE_AWAY_FROM_GEAR_AFTER_PLACING_PART_2, .175));
			addParallel(new AutoMoveGearHolder(AUTO_MOVE_GEAR_HOLDER_UPWARDS));
			addSequential(new DriveDistance(DISTANCE_TO_MOVE_AWAY_FROM_GEAR_AFTER_PLACING_PART_3, .4));
		}
		else if (leftGearAndShoot) {
			addSequential(new DriveDistance(LEFT_SIDE_GEAR_DISTANCE_PART_1, -.65));
			addParallel(new GetReadyForLiftHang());
			addSequential(new DriveDistance(SIDE_GEAR_DISTANCE_PART_2, -.25));
			addSequential(new DriveDistance(SIDE_GEAR_DISTANCE_PART_3, -.15));
			addSequential(new WaitTime(.2));
			addSequential(new TurnDriveTrainAngle((ANGLE_TO_TURN_TO_FACE_SIDE_GEAR), .65, TURN_RIGHT));
			if (USING_GEAR_CAMERA) {
				addSequential(new WaitTime(.25));
				addSequential(new CameraAdjustWhileDrivingToGear());
			}
			else if (USING_GEAR_CAMERA == false) {
				addSequential(new WaitTime(.1));
				addSequential(new DriveDistance(DISTANCE_AFTER_TURNING_ONTO_SIDE_GEAR, -.4));
				addSequential(new WaitTime(.15));
			}
			addSequential(new AutoGearOnPeg());
			addSequential(new WaitTime(.001));
			addParallel(new AutoMoveGearHolder(AUTO_MOVE_GEAR_HOLDER_DOWNWARDS));
			addSequential(new WaitTime(.6));
			addSequential(new AutoRotateRobotForGearPeg(AUTO_ROTATE_FOR_GEAR_PEG_MOTOR_POWER, ANGLE_TO_ROTATE_FOR_GEAR_PEG));
			addSequential(new WaitTime(.25));
			addSequential(new DriveDistance(DISTANCE_TO_MOVE_AWAY_FROM_GEAR_AFTER_PLACING_PART_1, .175));
			addSequential(new DriveDistance(DISTANCE_TO_MOVE_AWAY_FROM_GEAR_AFTER_PLACING_PART_2, .175));
			addParallel(new AutoMoveGearHolder(AUTO_MOVE_GEAR_HOLDER_UPWARDS));
			addSequential(new DriveDistance(DISTANCE_TO_MOVE_AWAY_FROM_GEAR_AFTER_PLACING_PART_3, .4));
			addSequential(new TurnDriveTrainAngle(ANGLE_TO_TURN_AFTER_DRIVING_AWAY_FROM_LEFT_GEAR, .6, TURN_LEFT));
			addParallel(new AutoGetShooterUpToSpeed(Shooter.SHOOTER_SETPOINT_RPM_FAR_SHOT, FAR_SHOT_PID_SLOT));
			addSequential(new WaitTime(.15));
			addSequential(new DriveDistance(DISTANCE_TO_TRAVEL_AFTER_PLACING_SIDE_GEAR, .6));
			addSequential(new WaitTime(.15));
			//addSequential(new AimBot(SHOOTER_CAMERA));
			/*double angle = -CommandBase.server.get_angle(SHOOTER_CAMERA);
			if(angle > 0)
				addSequential(new TurnOneSideOfRobotAngle(angle,false,.5));
			else
				addSequential(new TurnOneSideOfRobotAngle(angle,true,.5)); */
			addSequential(new AutoDriveAgitator(TIME_TO_RUN_AGITATOR_GEAR_AND_SHOOT));
		}
		else if (leftGearAndNeutralZone) {
			addSequential(new DriveDistance(LEFT_SIDE_GEAR_DISTANCE_PART_1, -.65));
			addParallel(new GetReadyForLiftHang());
			addSequential(new DriveDistance(SIDE_GEAR_DISTANCE_PART_2, -.25));
			addSequential(new DriveDistance(SIDE_GEAR_DISTANCE_PART_3, -.15));
			addSequential(new WaitTime(.2));
			addSequential(new TurnDriveTrainAngle((ANGLE_TO_TURN_TO_FACE_SIDE_GEAR), .65, TURN_RIGHT));
			addSequential(new WaitTime(.25));
			addSequential(new CameraAdjustWhileDrivingToGear());
			/*if (USING_GEAR_CAMERA) {
				addSequential(new WaitTime(.25));
				addSequential(new CameraAdjustWhileDrivingToGear());
			}
			else if (USING_GEAR_CAMERA == false) {
				addSequential(new WaitTime(.1));
				addSequential(new DriveDistance(DISTANCE_AFTER_TURNING_ONTO_SIDE_GEAR, -.4));
				addSequential(new WaitTime(.15));
			} */
			addSequential(new AutoGearOnPeg());
			addSequential(new WaitTime(.001));
			addParallel(new AutoMoveGearHolder(AUTO_MOVE_GEAR_HOLDER_DOWNWARDS));
			addSequential(new WaitTime(.6));
			addSequential(new AutoRotateRobotForGearPeg(AUTO_ROTATE_FOR_GEAR_PEG_MOTOR_POWER, ANGLE_TO_ROTATE_FOR_GEAR_PEG));
			addSequential(new WaitTime(.25));
			addSequential(new DriveDistance(DISTANCE_TO_MOVE_AWAY_FROM_GEAR_AFTER_PLACING_PART_1, .175));
			addSequential(new DriveDistance(DISTANCE_TO_MOVE_AWAY_FROM_GEAR_AFTER_PLACING_PART_2, .175));
			addParallel(new AutoMoveGearHolder(AUTO_MOVE_GEAR_HOLDER_UPWARDS));
			addSequential(new DriveDistance((DISTANCE_TO_MOVE_AWAY_FROM_GEAR_AFTER_PLACING_PART_3 + 8.0), .4));
			addSequential(new WaitTime(.2));
			addSequential(new TurnDriveTrainAngle(ANGLE_TO_TURN_TO_FACE_NEUTRAL_ZONE, .65, TURN_LEFT));
			addSequential(new WaitTime(.1));
			addSequential(new DriveDistance(DISTANCE_TO_TRAVEL_TOWARDS_AND_INTO_NEUTRAL_ZONE, -.65));
		}
		else if (closeShotRightWithBaseline) {
			//  Make Sure Shooter Hood Encoder is at Proper Value - Encoder Value at 5000
			addSequential(new AutoDriveShooter(Shooter.SHOOTER_SETPOINT_RPM_CLOSE_SHOT, CLOSE_SHOT_PID_SLOT));
			addSequential(new DriveDistance(DISTANCE_TO_TRAVEL_BACKWARDS_AFTER_CLOSE_SHOT, -.5));
			addParallel(new GetReadyForLiftHang());
			addSequential(new WaitTime(.2));
			addSequential(new TurnDriveTrainAngle(ANGLE_TO_TURN_AFTER_DRIVING_OFF_RIGHT_ALLIANCE_WALL, .6, TURN_RIGHT));
			addSequential(new WaitTime(.2));
			addSequential(new DriveDistance(DISTANCE_TO_TRAVEL_TO_BASELINE_AFTER_CLOSE_SHOT, -.65));
		}
		else if (closeShotRightWithGear) {
			addSequential(new AutoDriveShooter(Shooter.SHOOTER_SETPOINT_RPM_CLOSE_SHOT, CLOSE_SHOT_PID_SLOT));
			addSequential(new DriveDistance(DISTANCE_TO_TRAVEL_OFF_OF_BOILER_FOR_RIGHT_CLOSE_SHOT_AND_GEAR, -.6));
			addParallel(new GetReadyForLiftHang());
			addSequential(new TurnOneSideOfRobotAngle(ANGLE_TO_TURN_TOWARDS_RIGHT_GEAR, DRIVE_RIGHT_SIDE_DRIVE_TRAIN, -.6));
			addSequential(new WaitTime(.05));
			addSequential(new DriveDistance(DISTANCE_TO_DRIVE_BEFORE_TURNING_ONTO_RIGHT_GEAR, -.65));
			addSequential(new WaitTime(.15));
			addSequential(new TurnDriveTrainAngle(ANGLE_TO_TURN_TOWARDS_RIGHT_GEAR_SECOND_TURN, .55, TURN_LEFT));
			addSequential(new WaitTime(.25));
			if (USING_GEAR_CAMERA) {
				addSequential(new CameraAdjustWhileDrivingToGear());
			}
			else if (USING_GEAR_CAMERA == false) {
				addSequential(new DriveDistance(DISTANCE_TO_TRAVEL_TOWARDS_RIGHT_GEAR_PEG_FOR_CLOSE_SHOT, -.55));
				addSequential(new WaitTime(.05));
			}
			addSequential(new AutoGearOnPeg());
			addSequential(new WaitTime(.001));
			addParallel(new AutoMoveGearHolder(AUTO_MOVE_GEAR_HOLDER_DOWNWARDS));
			addSequential(new WaitTime(.6));
			addSequential(new AutoRotateRobotForGearPeg(AUTO_ROTATE_FOR_GEAR_PEG_MOTOR_POWER, ANGLE_TO_ROTATE_FOR_GEAR_PEG));
			addSequential(new WaitTime(.25));
			addSequential(new DriveDistance(DISTANCE_TO_MOVE_AWAY_FROM_GEAR_AFTER_PLACING_PART_1, .175));
			addSequential(new DriveDistance(DISTANCE_TO_MOVE_AWAY_FROM_GEAR_AFTER_PLACING_PART_2, .175));
			addParallel(new AutoMoveGearHolder(AUTO_MOVE_GEAR_HOLDER_UPWARDS));
			addSequential(new DriveDistance((DISTANCE_TO_MOVE_AWAY_FROM_GEAR_AFTER_PLACING_PART_3 + 8.0), .4));
			addSequential(new WaitTime(.2));
		}
		else if (closeShotLeftWithBaseline) {
			//  Make Sure Shooter Hood Encoder is at Proper Value
			addSequential(new AutoDriveShooter(Shooter.SHOOTER_SETPOINT_RPM_CLOSE_SHOT, CLOSE_SHOT_PID_SLOT));
			addSequential(new TurnOneSideOfRobotAngle(ANGLE_TO_TURN_AFTER_DRIVING_OFF_LEFT_ALLIANCE_WALL, DRIVE_LEFT_SIDE_DRIVE_TRAIN, -.65));
			addSequential(new WaitTime(.1));
			addSequential(new DriveDistance(DISTANCE_TO_TRAVEL_OFF_LEFT_ALLIANCE_WALL, -.6));
			addParallel(new GetReadyForLiftHang());
			addSequential(new WaitTime(.2));
			addSequential(new TurnDriveTrainAngle(ANGLE_TO_TURN_AFTER_TURNING_OFF_LEFT_ALLIANCE_WALL, .55, TURN_RIGHT));
			addSequential(new WaitTime(.2));
			addSequential(new DriveDistance(DISTANCE_TO_TRAVEL_ABOUT_PARALLEL_WITH_LEFT_ALLIANCE_WALL, .65));
			addSequential(new WaitTime(.1));
			addSequential(new TurnDriveTrainAngle(ANGLE_TO_TURN_TO_DRIVE_TO_BASELINE_AFTER_LEFT_ALLIANCE_WALL, .55, TURN_RIGHT));
			addSequential(new WaitTime(.1));
			addSequential(new DriveDistance(DISTANCE_TO_TRAVEL_TO_BASELINE_AFTER_CLOSE_SHOT, .7));
		}
		else if (closeShotLeftWithGear) {
			addSequential(new DriveDistance(DISTANCE_TO_TRAVEL_OFF_OF_LEFT_ALLIANCE_WALL, -.7));
			addSequential(new WaitTime(.1));
			addSequential(new TurnDriveTrainAngle(ANGLE_TO_TURN_FOR_ROBOT_TO_BE_PERPENDICULAR_WITH_BOILER, .55, TURN_RIGHT));
			addParallel(new AutoGetShooterUpToSpeed(Shooter.SHOOTER_SETPOINT_RPM_CLOSE_SHOT, CLOSE_SHOT_PID_SLOT));
			addSequential(new WaitTime(.1));
			addSequential(new DriveDriveTrainCertainTime(TIME_FOR_DRIVE_TRAIN_TO_DRIVE_INTO_BOILER, .75));
			addSequential(new AutoDriveAgitator(TIME_TO_RUN_AGITATOR_FOR_LEFT_CLOSE_SHOT_AND_GEAR));
			addSequential(new DriveDistance(DISTANCE_TO_DRIVE_AWAY_FROM_BOILER, -.6));
			addSequential(new TurnOneSideOfRobotAngle(ANGLE_TO_TURN_AFTER_DRIVING_AWAY_FROM_BOILER, DRIVE_LEFT_SIDE_DRIVE_TRAIN, -.6));
			addSequential(new WaitTime(.05));
			addSequential(new DriveDistance(DISTANCE_TO_DRIVE_TOWARDS_LEFT_GEAR_AIRSHIP, -.65));
			addSequential(new WaitTime(.1));
			addSequential(new TurnDriveTrainAngle(ANGLE_TO_TURN_TOWARDS_LEFT_GEAR_PEG, .6, TURN_RIGHT));
			addSequential(new WaitTime(.25));
			addSequential(new CameraAdjustWhileDrivingToGear());
			/*if (USING_GEAR_CAMERA) {
				addSequential(new CameraAdjustWhileDrivingToGear());
			}
			else if (USING_GEAR_CAMERA == false) {
				addSequential(new DriveDistance(DISTANCE_TO_TRAVEL_TOWARDS_LEFT_GEAR_PEG_FOR_CLOSE_SHOT, -.55));
				addSequential(new WaitTime(.05));
			} */
			addSequential(new AutoGearOnPeg());
			addSequential(new WaitTime(.001));
			addParallel(new AutoMoveGearHolder(AUTO_MOVE_GEAR_HOLDER_DOWNWARDS));
			addSequential(new WaitTime(.6));
			addSequential(new AutoRotateRobotForGearPeg(AUTO_ROTATE_FOR_GEAR_PEG_MOTOR_POWER, ANGLE_TO_ROTATE_FOR_GEAR_PEG));
			addSequential(new WaitTime(.25));
			addSequential(new DriveDistance(DISTANCE_TO_MOVE_AWAY_FROM_GEAR_AFTER_PLACING_PART_1, .175));
			addSequential(new DriveDistance(DISTANCE_TO_MOVE_AWAY_FROM_GEAR_AFTER_PLACING_PART_2, .175));
			addParallel(new AutoMoveGearHolder(AUTO_MOVE_GEAR_HOLDER_UPWARDS));
			addSequential(new DriveDistance((DISTANCE_TO_MOVE_AWAY_FROM_GEAR_AFTER_PLACING_PART_3 + 8.0), .4));
			addSequential(new WaitTime(.2));
		}
		/*else if (right40KPa) {
			addSequential(new DriveDistance(DISTANCE_TO_DRIVE_FROM_ALLIANCE_WALL_TO_GUARDRAIL, -.6));
			addSequential(new WaitTime(.05));
			addSequential(new TurnOneSideOfRobotAngle(ANGLE_TO_TURN_TO_BE_PARALLEL_WITH_GUARDRAIL, DRIVE_LEFT_SIDE_DRIVE_TRAIN, -.65));
			addSequential(new WaitTime(.2));
			addSequential(new AlignRobotWithGuardrail(.55, RIGHT_HOPPER_AND_SHOOT));
			addSequential(new WaitTime(.01));
			addParallel(new GetLidarValueForHopperAndShoot());
			addSequential(new WaitTime(.15));
			addSequential(new DriveStraightGivenLidarValue(-.4));
			addSequential(new WaitTime(.1));
			addSequential(new TurnOneSideOfRobotAngle(ANGLE_TO_TURN_TO_LINE_UP_TO_HIT_PANEL, DRIVE_LEFT_SIDE_DRIVE_TRAIN, -.6));
			addSequential(new WaitTime(.05));
			addParallel(new AutoDriveCollection());
			addSequential(new TurnDriveTrainAngle(ANGLE_TO_TURN_AWAY_FROM_HOPPER_PANEL, .7, TURN_RIGHT));
			addParallel(new AutoGetShooterUpToSpeed(Shooter.SHOOTER_SETPOINT_RPM_FAR_SHOT, FAR_SHOT_PID_SLOT));
			addSequential(new WaitTime(.1));
			addSequential(new TurnDriveTrainAngle((ANGLE_TO_TURN_TOWARDS_HOPPER - 5.0), .8, TURN_LEFT));
			addSequential(new WaitTime(.2));
			//addSequential(new AimBot(SHOOTER_CAMERA));
			addSequential(new AutoDriveAgitator(TIME_TO_RUN_AGITATOR_HOPPER_AND_SHOOT));
		}
		else if (left40KPa) {
			addSequential(new DriveDistance(DISTANCE_TO_DRIVE_FROM_ALLIANCE_WALL_TO_GUARDRAIL, -.6));
			addSequential(new WaitTime(.05));
			addSequential(new TurnOneSideOfRobotAngle(ANGLE_TO_TURN_TO_BE_PARALLEL_WITH_GUARDRAIL, DRIVE_RIGHT_SIDE_DRIVE_TRAIN, -.65));
			addSequential(new WaitTime(.2));
			addSequential(new AlignRobotWithGuardrail(.55, LEFT_HOPPER_AND_SHOOT));
			addSequential(new WaitTime(.01));
			addParallel(new GetLidarValueForHopperAndShoot());
			addSequential(new WaitTime(.15));
			addSequential(new DriveStraightGivenLidarValue(-.4));
			addSequential(new WaitTime(.1));
			addSequential(new TurnOneSideOfRobotAngle(ANGLE_TO_TURN_TO_LINE_UP_TO_HIT_PANEL, DRIVE_RIGHT_SIDE_DRIVE_TRAIN, -.6));
			addSequential(new WaitTime(.05));
			addParallel(new AutoDriveCollection());
			addSequential(new TurnDriveTrainAngle(ANGLE_TO_TURN_AWAY_FROM_HOPPER_PANEL, .7, TURN_LEFT));
			addParallel(new AutoGetShooterUpToSpeed(Shooter.SHOOTER_SETPOINT_RPM_FAR_SHOT, FAR_SHOT_PID_SLOT));
			addSequential(new WaitTime(.1));
			addSequential(new TurnDriveTrainAngle((ANGLE_TO_TURN_TOWARDS_HOPPER - 5.0), .8, TURN_RIGHT));
			addSequential(new WaitTime(.2));
			//addSequential(new AimBot(SHOOTER_CAMERA));
			addSequential(new AutoDriveAgitator(TIME_TO_RUN_AGITATOR_HOPPER_AND_SHOOT));
		} */
	}
}
