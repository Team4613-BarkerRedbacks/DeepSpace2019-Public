package redbacks.robot.subsystems;

import redbacks.arachne.core.SubsystemBase;
import redbacks.arachne.lib.motors.CtrlMotor;
import redbacks.arachne.lib.solenoids.SolSingle;
import redbacks.arachne.lib.solenoids.SolSingle2;

import static redbacks.robot.RobotMap.*;

import static redbacks.robot.Robot.*;

/**
 * @author Mitchell Barker, Darin Huang, Lucas Parker, Gabriela Ribeiro, Jon Reilly, Ben Schwarz, Sean Zammit
 */
public class SubsystemIntake extends SubsystemBase {

	public CtrlMotor intakeMotor = new CtrlMotor(idMotIntake);
	public CtrlMotor outtakeMotor = new CtrlMotor(idMotOuttake);
	public SolSingle2 intakeHatchSol = new SolSingle2(idSolIntakeHatch);
	public SolSingle intakeCargoSol = new SolSingle(idSolIntakeCargo);
	public SolSingle intakeCargoExtensionSol = new SolSingle(idSolIntakeCargoExtension);

	public boolean shouldExtend() {
		return elevator.setpoint >= elevatorPosBumper && sensors.elevatorEncoder.get() >= elevatorPosBumper && elevator.isCargo;
	}
}
