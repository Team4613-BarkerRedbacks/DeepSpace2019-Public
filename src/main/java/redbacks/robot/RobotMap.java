package redbacks.robot;

import redbacks.arachne.ext.ctre.controllers.CtrlCANTalon;
import redbacks.arachne.ext.ctre.controllers.CtrlCANVictor;

/**
 * Contains robot constants.
 * @author Mitchell Barker, Darin Huang, Lucas Parker, Gabriela Ribeiro, Jon Reilly, Ben Schwarz, Sean Zammit
 */
public class RobotMap {

	public static final double
		elevatorSpeedMax = 0.7,
		elevatorSpeedMin = -0.5,

		elevatorKP = 0.78e-4, elevatorKI = 0, elevatorKD = 0,
		elevatorCompMax1st = 0.06, elevatorCompMin1st = 0.05,
		elevatorCompMax2nd = 0.09, elevatorCompMin2nd = 0.065,

		elevatorPosMin = 0, elevatorPosMax = 80000, elevatorPosBumper = 8000, elevatorPos2ndStage = 40000,
		elevatorPosHatch1 = elevatorPosMin, elevatorPosHatch2 = 35800, elevatorPosHatch3 = 68200, elevatorPosHatchShip = 5000,
		elevatorPosCargo1 = 11600, elevatorPosCargo2 = 45600, elevatorPosCargo3 = 77800, elevatorPosCargoShip = 35900,
		elevatorPosStartingConfiguraton = 15650, elevatorPosDeploy = elevatorPosStartingConfiguraton + 5000,
		elevatorTolerancePos = 1000, elevatorLimelightPosMax = 13000;

	public static final double
		driveMinVoltage = 0.4,
		stoppedMoveThreshold = 0.2,
		stoppedTurnThreshold = 10;

	public static final double
		voltageMax = 12.7, voltageMin = 11.5;

	public static final double
		elevatorResetSpeedMax = -0.9, elevatorResetSpeedMin = -0.35;

	public static final double
		climberMaxSpeed = 0.9,
		climberVerticalSpeed = 0.9,
		climberHorizontalSpeed = 0.8,
		climberDriveSpeed = 0.4;

	public static final double
		intakeSpeed = 0.7, // Intake
		intakeDropSpeed = -1; // Outtake

	public static final double
		trackingRotKP = 1.25e-2, trackingRotKI = 0, trackingRotKD = 0,
		trackingDisKP = 0, trackingDisKI = 0, trackingDisKD = 0,
		trackingRotMax = 0.5,
		trackingCloseSpeed = 0.2, trackingFarSpeed = 0.4, trackingPlaceSpeed = 0.3, trackingPlaceSpeedSlow = 0.2,
		trackingCloseDis = 2500;

	public static final double
		encoderTicksPerMetre = 8815.9;

    public static final CtrlCANTalon
		talon0 = new CtrlCANTalon(0),
		talon1 = new CtrlCANTalon(1),
		talon2 = new CtrlCANTalon(2),
		talon3 = new CtrlCANTalon(3),
		talon4 = new CtrlCANTalon(4),
		talon5 = new CtrlCANTalon(5),
		talon6 = new CtrlCANTalon(6),
		talon7 = new CtrlCANTalon(7);

	public static final CtrlCANVictor
		victor0 = new CtrlCANVictor(0),
		victor1 = new CtrlCANVictor(1),
		victor2 = new CtrlCANVictor(2),
		victor3 = new CtrlCANVictor(3),
		victor4 = new CtrlCANVictor(4);
        
    public static final CtrlCANTalon
		idMotDriveL1 = talon0,
		idMotDriveR1 = talon1,
		idMotDriveL2 = talon2,
		idMotDriveR2 = talon3,
		idMotDriveL3 = talon4,
        idMotDriveR3 = talon5,
		idMotClimbVertical1 = talon6,
		idMotClimbVertical2 = talon7;

	public static final CtrlCANVictor
		idMotClimberHorizontal = victor4,
		idMotIntake = victor1,
		idMotOuttake = victor3,
		idMotElevator1 = victor0,
		idMotElevator2 = victor2;

    public static final int
		idSolClimberHorizontalF = 4,
		idSolClimberHorizontalR = 3,
		idSolIntakeHatch = 5,
		idSolIntakeCargoExtension = 0,
		idSolIntakeCargo = 6,
		idSolClimberLock = 2,
		idSolVacuum = 1,
		idSolSki = 7;

	public static final int
		idSenClimbSwitchV = 0,
		idSenClimbSwitchH = 1,
		idSenLineL = 2,
		idSenLineC = 3,
		idSenLineR = 4,
		idSenElevatorSwitch = 5,
		idSenHatchSwitch = 6;

	public static final int
		idPWMLEDs = 0;

	public static final int
		idBtnVision = 1,
		idBtnVacuum = 2,
		idBtnCamera = 4,
		idBtnPieceMode = 6,
		idBtnClimb = 8;
}
