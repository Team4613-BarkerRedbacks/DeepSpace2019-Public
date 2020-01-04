package redbacks.robot.actions;

import redbacks.arachne.core.ArachneRobot;
import redbacks.arachne.ext.motion.MotionSettings;
import redbacks.arachne.ext.motion.pid.Tolerances;
import redbacks.arachne.ext.motion.trajectories.AcPath;
import redbacks.arachne.ext.motion.trajectories.Path;
import redbacks.arachne.lib.checks.ChFalse;
import redbacks.arachne.lib.logic.GettableNumber;

import redbacks.arachne.lib.sensors.NumericSensor;
import redbacks.robot.Robot;
import redbacks.robot.RobotMap;

/**
 * Action for robot to travel straight.
 * @author Mitchell Barker, Darin Huang, Lucas Parker, Gabriela Ribeiro, Jon Reilly, Ben Schwarz, Sean Zammit
 */
public class AcStraight extends AcPath {
	public boolean shouldReset;
	
	public AcStraight(MotionSettings settings, double distance, double angle, NumericSensor encoder, boolean shouldReset) {
		super(new ChFalse(), true, settings, new Path(settings, new double[]{distance, angle, 0}), Robot.driver.drivetrain, 1, 1, Robot.sensors.yaw, encoder, false, new Tolerances.Absolute(0.15 * RobotMap.encoderTicksPerMetre));
		this.shouldReset = shouldReset;
	}
	
	public AcStraight(MotionSettings settings, double distance, double angle, NumericSensor encoder, boolean shouldReset, GettableNumber minOut, GettableNumber maxOut) {
		this(settings, distance, angle, encoder, shouldReset);
		this.minOut = minOut;
		this.maxOut = maxOut;
	}
	
	public void onStart() {
		path.reset();
		if(shouldReset) encoder.set(0);
		ArachneRobot.isIndivDriveControl = false;
		acLinear.initialise(command);
		acRotation.initialise(command);
	}
	
	public void onRun() {
		acLinear.execute();
		acRotation.execute();
		
		double linearOutput = linearOut.output;
		if(minOut != null && maxOut != null) linearOutput = Math.max(Math.min(linearOutput, maxOut.get()), minOut.get());
		
		if(Math.abs(linearOutput) < RobotMap.driveMinVoltage) {
			if(linearOutput >= 0) linearOutput = RobotMap.driveMinVoltage;
			else linearOutput = -RobotMap.driveMinVoltage;
		}
		
		drivetrain.tankDrive(linearOutput + rotationOut.output, linearOutput - rotationOut.output);
	}
	
	public void onFinish() {
		super.onFinish();
		
		System.out.println(encoder.get());
	}
}