package redbacks.robot.subsystems;

import redbacks.arachne.core.SubsystemBase;
import redbacks.arachne.lib.solenoids.SolSingle;

import static redbacks.robot.RobotMap.*;

/**
 * @author Mitchell Barker, Darin Huang, Lucas Parker, Gabriela Ribeiro, Jon Reilly, Ben Schwarz, Sean Zammit
 */
public class SubsystemVacuum extends SubsystemBase {

	public SolSingle vacuumSol = new SolSingle(idSolVacuum);
}
