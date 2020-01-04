package redbacks.robot.subsystems;

import redbacks.arachne.core.SubsystemBase;
import redbacks.arachne.lib.signalling.LEDBlinkin;
import redbacks.arachne.lib.signalling.LEDBlinkin.Pattern;
import redbacks.robot.Robot;

import static redbacks.robot.RobotMap.*;
import static redbacks.robot.Robot.sensors;

/**
 * @author Mitchell Barker, Darin Huang, Lucas Parker, Gabriela Ribeiro, Jon Reilly, Ben Schwarz, Sean Zammit
 */
public class SubsystemSignalling extends SubsystemBase {

	public LEDBlinkin leds = new LEDBlinkin(idPWMLEDs);

	public SubsystemSignalling() {
        super();
        
        leds.set(Pattern.RED);
	}

	public void manageLEDs() {
		if(sensors.hatchSwitch.get()) leds.set(Pattern.BLUE);
		else if(Robot.elevator.setpoint == 0 && !sensors.elevatorSwitch.get()) {
			if(System.currentTimeMillis() / 125 % 2 == 0) leds.set(Pattern.PURPLE);
			else leds.set(Pattern.BLACK);
		}
		else if(Robot.vision.isValidTarget()) leds.set(Pattern.TURQUOISE);
		else leds.set(Pattern.RED);
	}
}
