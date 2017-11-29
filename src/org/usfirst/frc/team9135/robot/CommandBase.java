package org.usfirst.frc.team9135.robot;

import org.usfirst.frc.team9135.robot.subsystems.*;

public class CommandBase 
{
	public static OI oi = new OI();
	public static DriveTrain driveTrain = new DriveTrain();
	public static Agitator agitator = new Agitator();
	public static CameraServo cameraServo = new CameraServo();
	public static Collection collection = new Collection();
	public static GearHolder gearHolder = new GearHolder();
	public static Lidars lidars = new Lidars();
	public static LiftHang liftHang = new LiftHang();
	public static PDP pdp = new PDP();
	public static Shooter shooter = new Shooter();
	public static ShooterHood shooterHood = new ShooterHood();
	public static UltrasonicSensor ultrasonicSensor  = new UltrasonicSensor();
	public static Server server = new Server();
}
