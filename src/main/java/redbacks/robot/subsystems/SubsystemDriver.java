package redbacks.robot.subsystems;

import redbacks.arachne.core.SubsystemBase;
import redbacks.arachne.ext.motion.MotionSettings;
import redbacks.arachne.lib.actions.drive.DriveSettings;
import redbacks.arachne.lib.motors.CtrlDrive;
import redbacks.arachne.lib.motors.CtrlDrivetrain;
import redbacks.robot.OI;
import redbacks.robot.Robot;
import redbacks.robot.RobotMap;

import static redbacks.robot.RobotMap.*;

/**
 * @author Mitchell Barker, Darin Huang, Lucas Parker, Gabriela Ribeiro, Jon Reilly, Ben Schwarz, Sean Zammit
 */
public class SubsystemDriver extends SubsystemBase {

	public CtrlDrive leftMotor = new CtrlDrive(RobotMap.idMotDriveL1);
	public CtrlDrive rightMotor = new CtrlDrive(RobotMap.idMotDriveR1);	
	public CtrlDrivetrain drivetrain = new CtrlDrivetrain(leftMotor, rightMotor);

	public MotionSettings holdingSettings = new MotionSettings();
	
	public SubsystemDriver(SubsystemBase... childSystems) {
		super(childSystems);

		idMotDriveL2.follow(idMotDriveL1);
		idMotDriveL3.follow(idMotDriveL1);

		idMotDriveR2.follow(idMotDriveR1);
		idMotDriveR3.follow(idMotDriveR1);

		DriveSettings.drivetrain = this.drivetrain;
		DriveSettings.gyro = Robot.sensors.yaw;
		DriveSettings.gyroCorrection = 0.03;

		holdingSettings.trajectoryMaxPosSpeed = 0.8;
		holdingSettings.trajectoryMaxNegSpeed = -0.8;

		holdingSettings.drivePIDLinearKP = 0.2e-1;
		holdingSettings.drivePIDRotKP = 0.03;
		holdingSettings.drivePIDLinearKD = 0.2e-1;
	}
	
	private final double minR = 0.7D, difR = 0.3D;
	
	public void arcadeDrive(boolean invertDrive) {
		double sp = OI.axis_r_Y.get() * (invertDrive ? -1 : 1), rotation = -OI.axis_l_X.get();

		double mod = minR + difR * Math.pow(1 - Math.abs(sp), 2);
		double r = Math.pow(rotation, 3) * mod;
		
		Robot.driver.drivetrain.tankDrive(- sp - r, - sp + r);
	}
}
