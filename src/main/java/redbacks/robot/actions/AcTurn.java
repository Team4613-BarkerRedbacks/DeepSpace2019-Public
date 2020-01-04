package redbacks.robot.actions;

import redbacks.arachne.core.ArachneRobot;
import redbacks.arachne.ext.motion.MotionSettings;
import redbacks.arachne.ext.motion.pid.Tolerances;
import redbacks.arachne.lib.checks.ChMulti;
import redbacks.arachne.lib.checks.analog.ChGettableNumber;
import redbacks.arachne.lib.checks.analog.ChNumSen;
import redbacks.arachne.lib.logic.ListOperators;
import redbacks.arachne.lib.logic.LogicOperators;
import redbacks.arachne.lib.sensors.SenTimer;
import redbacks.arachne.ext.motion.trajectories.AcPath;
import redbacks.arachne.ext.motion.trajectories.Path;
import redbacks.robot.Robot;
import redbacks.robot.RobotMap;

/**
 * Action to turn on the spot.
 * @author Mitchell Barker, Darin Huang, Lucas Parker, Gabriela Ribeiro, Jon Reilly, Ben Schwarz, Sean Zammit
 */
public class AcTurn extends AcPath
{
	public AcTurn(MotionSettings settings, double angleTarget) {
		super(new ChMulti(
				ListOperators.ORDER,
				new ChMulti(LogicOperators.OR,
						new ChGettableNumber(RobotMap.stoppedTurnThreshold * 10, Robot.sensors.rateYaw, true, true),
						new ChNumSen(0.5, new SenTimer())
				),
				new ChNumSen(0.5, new SenTimer()),
				new ChGettableNumber(RobotMap.stoppedTurnThreshold, Robot.sensors.rateYaw, false, true)
		), false, settings, new Path(settings, new double[]{0, angleTarget, 0}), Robot.driver.drivetrain, 1, 1, Robot.sensors.yaw, Robot.sensors.averageEncoder, false, new Tolerances.Absolute(0));
	}
	
	public void onStart() {
		path.reset();
		ArachneRobot.isIndivDriveControl = false;
		acLinear.initialise(command);
		acRotation.initialise(command);
	}
	
	public void onRun() {
		acRotation.execute();
		
		drivetrain.tankDrive(rotationOut.output, - rotationOut.output);
	}
}