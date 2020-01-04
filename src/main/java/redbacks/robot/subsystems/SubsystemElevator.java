package redbacks.robot.subsystems;

import redbacks.arachne.core.SubsystemBase;
import redbacks.arachne.lib.actions.AcLambda;
import redbacks.arachne.lib.actions.Action;
import redbacks.arachne.lib.checks.ChTrue;
import redbacks.arachne.lib.motors.CtrlMotor;
import redbacks.robot.Robot;

import static redbacks.robot.RobotMap.*;

import edu.wpi.first.wpilibj.RobotController;

/**
 * @author Mitchell Barker, Darin Huang, Lucas Parker, Gabriela Ribeiro, Jon Reilly, Ben Schwarz, Sean Zammit
 */
public class SubsystemElevator extends SubsystemBase {

    public CtrlMotor elevatorMotor = new CtrlMotor(idMotElevator1);
	
	public double setpoint = 0;
	public boolean isCargo = false;
	
	public SubsystemElevator(SubsystemBase... childSystems) {
		super(childSystems);
		idMotElevator2.follow(idMotElevator1);
	}

	public Action setElevator(double pos, boolean isCargo) {
		return new AcLambda(new ChTrue(), a -> {
			this.setpoint = pos;
			this.isCargo = isCargo;
		});
	}

	public double addCompensation(double d) {
		if(setpoint == 0 && Robot.sensors.elevatorSwitch.get()) return 0;

		double compMin, compMax;

		if(Robot.sensors.elevatorEncoder.get() < elevatorPos2ndStage) {
			compMin = elevatorCompMin1st;
			compMax = elevatorCompMax1st;
		}
		else {
			compMin = elevatorCompMin2nd;
			compMax = elevatorCompMax2nd;
		}

		double voltage = Math.max(Math.min(RobotController.getBatteryVoltage(), voltageMax), voltageMin);
		double comp = (compMax - compMin) * (voltage - voltageMin) / (voltageMax - voltageMin) + compMin;

		return d + comp;
	}
}
