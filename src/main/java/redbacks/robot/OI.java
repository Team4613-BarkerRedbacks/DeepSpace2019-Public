package redbacks.robot;

import static redbacks.arachne.lib.input.ButtonGettableWrapper.wrap;
import static redbacks.robot.CommandList.*;
import static redbacks.robot.RobotMap.*;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.buttons.Button;
import edu.wpi.first.wpilibj.buttons.JoystickButton;
import redbacks.arachne.core.OIBase;
import redbacks.arachne.lib.actions.AcInterrupt;
import redbacks.arachne.lib.commands.CommandBase;
import redbacks.arachne.lib.commands.CommandSetup;
import redbacks.arachne.lib.input.BtnAxis;
import redbacks.arachne.lib.input.BtnPOV;
import redbacks.arachne.lib.input.ButtonGettableWrapper;
import redbacks.arachne.lib.input.JoystickAxis;
import redbacks.arachne.lib.logic.GettableBoolean;

/**
 * Mitchell Barker, Darin Huang, Lucas Parker, Gabriela Ribeiro, Jon Reilly, Ben Schwarz, Sean Zammit
 */
public class OI extends OIBase {
    
    public void mapOperations() {
		// Cargo
		whenPressed(new DualButton(p_CargoMode, o_RStick_D), elevatorToGround.c());
		whenPressed(new DualButton(p_CargoMode, o_RStick_U), elevatorToCargoShip.c());
		whenPressed(new DualButton(p_CargoMode, o_A), elevatorToCargo1.c());
		whenPressed(new DualButton(p_CargoMode, o_B), elevatorToCargo2.c());
		whenPressed(new DualButton(p_CargoMode, o_Y), elevatorToCargo3.c());
		
		whenHeld(new DualButton(p_CargoMode, o_LT), intakeCargo.c());
		whenReleased(new DualButton(p_CargoMode, o_LT), intakeCargoSolRetract.c());

		whenHeld(new DualButton(p_CargoMode, o_RT), outtakeCargo.c());

		// Hatch Panels
		whenPressed(new DualButton(p_HatchMode, o_RStick_D), elevatorToGround.c());
		whenPressed(new DualButton(p_HatchMode, o_RStick_U), elevatorToHatchShip.c());
		whenPressed(new DualButton(p_HatchMode, o_A), elevatorToHatch1.c());
		whenPressed(new DualButton(p_HatchMode, o_B), elevatorToHatch2.c());
		whenPressed(new DualButton(p_HatchMode, o_Y), elevatorToHatch3.c());
 
		Button b = new DualButton(p_HatchMode, o_LT);
		whenPressed(b, intakeHatchSolOpen.c(), intakeCargoSolRetract.c());
		whenReleased(b, intakeHatchSolClose.c());

		b = new DualButton(p_HatchMode, o_RT);
		whenPressed(b, placeHatch.c());
		whenReleased(b, intakeCargoSolRetract.c(), intakeHatchSolClose.c());
		
		// whenHeld(o_X, collectHatch.c());
		// whenReleased(o_X, driverSetGrouped.c());
		// whenReleased(o_X, new CommandSetup(null, new AcInterrupt.KillSubsystem(Robot.driver)).c());

		// Climber
		whenHeld(p_Climb, climberAuto.c());
		whenReleased(p_Climb, climberLock.c());

		whenHeld(p_Vacuum, vacuumActivate.c());

		// Sensors
		whenHeld(p_Vision, trackTarget.c());
		whenReleased(p_Vision, driverSetGrouped.c());
		whenReleased(p_Vision, limelightOff.c());

		whenPressed(o_LB, limelightSetLeftmost.c());
		whenReleased(o_LB, limelightSetLowest.c());

		whenPressed(o_RB, limelightSetRightmost.c());
		whenReleased(o_RB, limelightSetLowest.c());

		whenPressedReleased(p_Camera, cameraActivate.c(), cameraDisable.c());

		// Extra
		whenHeld(o_Start, climberAnalog.c());
		whenPressedReleased(o_Start, climberRelease.c(), climberLock.c());
		whenPressed(o_RStick, climberHorizontalSolToggle.c());

		whenHeld(o_Back, elevatorAnalog.c());

		whenPressed(o_LStick, killAll.c());
	}

    public static final Joystick stickLeft = new Joystick(0);
	
	public static final JoystickAxis
		axis_l_X = new JoystickAxis(stickLeft, 0),
		axis_l_Y = new JoystickAxis(stickLeft, 1),
		axis_l_Z = new JoystickAxis(stickLeft, 2);
	
	public static final Joystick stickRight = new Joystick(1);
	
	public static final JoystickAxis
		axis_r_X = new JoystickAxis(stickRight, 0),
		axis_r_Y = new JoystickAxis(stickRight, 1),
		axis_r_Z = new JoystickAxis(stickRight, 2);
	
	public static final Joystick stickOperator = new Joystick(2);
	
	public static final JoystickAxis
		axis_o_LX = new JoystickAxis(stickOperator, 0),
		axis_o_LY = new JoystickAxis(stickOperator, 1),
		axis_o_LT = new JoystickAxis(stickOperator, 2),
		axis_o_RT = new JoystickAxis(stickOperator, 3),
		axis_o_RX = new JoystickAxis(stickOperator, 4),
		axis_o_RY = new JoystickAxis(stickOperator, 5);
	
	public static final ButtonGettableWrapper
		o_A = wrap(new JoystickButton(stickOperator, 1)),
		o_B = wrap(new JoystickButton(stickOperator, 2)),
		o_X = wrap(new JoystickButton(stickOperator, 3)),
		o_Y = wrap(new JoystickButton(stickOperator, 4)),
		o_LB = wrap(new JoystickButton(stickOperator, 5)),
		o_RB = wrap(new JoystickButton(stickOperator, 6)),
		o_Back = wrap(new JoystickButton(stickOperator, 7)),
		o_Start = wrap(new JoystickButton(stickOperator, 8)),
		o_LStick = wrap(new JoystickButton(stickOperator, 9)),
		o_RStick = wrap(new JoystickButton(stickOperator, 10)),
	
		o_POV_U = wrap(new BtnPOV(stickOperator, 0)),
		o_POV_R = wrap(new BtnPOV(stickOperator, 90)),
		o_POV_D = wrap(new BtnPOV(stickOperator, 180)),
		o_POV_L = wrap(new BtnPOV(stickOperator, 270)),
	
		o_LT = wrap(new BtnAxis(axis_o_LT, false, 0.5D)),
		o_RT = wrap(new BtnAxis(axis_o_RT, false, 0.5D)),
	
		o_RStick_U = wrap(new DualButton(
			wrap(new BtnAxis(axis_o_RY, true, 0.8D)),
			new InvertButton(o_Start)
		)),
		o_RStick_D = wrap(new DualButton(
			wrap(new BtnAxis(axis_o_RY, false, 0.8D)),
			new InvertButton(o_Start)
		));
        
	public static final Joystick panel = new Joystick(3);

	public static final ButtonGettableWrapper
		p_Vision = wrap(new JoystickButton(panel, idBtnVision)),
		p_Vacuum = wrap(new JoystickButton(panel, idBtnVacuum)),
		p_Climb = wrap(new JoystickButton(panel, idBtnClimb)),
		p_Camera = wrap(new JoystickButton(panel, idBtnCamera)),

		p_CargoMode = wrap(new JoystickButton(panel, idBtnPieceMode)),
		p_HatchMode = wrap(new Button() {
			public boolean get() {
				return !p_CargoMode.get();
			}
		});
		
	/**
	 * Class to allow two buttons to be used to trigger one command.
	 * Used in this season primarily to allow buttons on controller to do different tasks based off toggle switches (E.g. cargo mode vs hatch mode)
	 */
	public static class DualButton extends Button implements GettableBoolean {

		final GettableBoolean b1, b2;

		public DualButton(GettableBoolean b1, GettableBoolean b2) {
			this.b1 = b1;
			this.b2 = b2;
		}

		public boolean get() {
			return b1.get() && b2.get();
		}
	}

	public static class InvertButton extends Button implements GettableBoolean {

		final GettableBoolean btn;

		public InvertButton(GettableBoolean btn) {
			this.btn = btn;
		}

		public boolean get() {
			return !btn.get();
		}
	}
	
	public void whenPressedReleased(Button button, CommandBase onPressed, CommandBase onReleased) {
        button.whenPressed(onPressed);
        button.whenReleased(onReleased);
    }
}