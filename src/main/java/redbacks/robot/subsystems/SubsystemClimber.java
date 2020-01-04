package redbacks.robot.subsystems;

import redbacks.arachne.core.SubsystemBase;
import redbacks.arachne.lib.motors.CtrlMotor;
import redbacks.arachne.lib.solenoids.SolDouble;
import redbacks.arachne.lib.solenoids.SolSingle;

import static redbacks.robot.RobotMap.*;

/**
 * @author Mitchell Barker, Darin Huang, Lucas Parker, Gabriela Ribeiro, Jon Reilly, Ben Schwarz, Sean Zammit
 */
public class SubsystemClimber extends SubsystemBase {

	public CtrlMotor climberVerticalMotor = new CtrlMotor(idMotClimbVertical1);
	public CtrlMotor climberHorizontalMotor	 = new CtrlMotor(idMotClimberHorizontal);

	public SolDouble climberHorizontalSol = new SolDouble(idSolClimberHorizontalF, idSolClimberHorizontalR);
	public SolSingle skiSol = new SolSingle(idSolSki);
	public SolSingle climberLockSol = new SolSingle(idSolClimberLock);

	public SubsystemClimber(SubsystemBase... childSystems) {
		super(childSystems);

		idMotClimbVertical2.follow(idMotClimbVertical1);
	}
}
