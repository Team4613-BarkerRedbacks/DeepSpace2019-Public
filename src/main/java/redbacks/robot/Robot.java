package redbacks.robot;

import redbacks.arachne.core.ArachneRobot;
import redbacks.arachne.core.SubsystemBase;
import redbacks.arachne.ext.motion.MotionSettings;
import redbacks.arachne.lib.actions.Action;
import redbacks.arachne.lib.commands.CommandBase;
import redbacks.robot.subsystems.*;
import redbacks.robot.subsystems.SubsystemVision.Pipeline;

import static redbacks.robot.CommandList.*;

import edu.wpi.cscore.UsbCamera;
import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/**
 * Barker Redbacks 2019 Robot Code for our FRC robot Huntsman.
 * Programmed using the Arachne 2.2.0 Library.
 * @author Mitchell Barker, Darin Huang, Lucas Parker, Gabriela Ribeiro, Jon Reilly, Ben Schwarz, Sean Zammit
 */
public class Robot extends ArachneRobot {

    public static void main(String[] args) {
        MotionSettings.standardSettings.trajectoryMaxPosSpeed = 0.8;
        MotionSettings.standardSettings.trajectoryMaxNegSpeed = -0.8;

        MotionSettings.standardSettings.drivePIDLinearKP = 0.00006;
        MotionSettings.standardSettings.drivePIDLinearKD = 0.000008;

        MotionSettings.standardSettings.drivePIDRotKP = 0.028;
        MotionSettings.standardSettings.drivePIDRotKI = 0.0002;
        MotionSettings.standardSettings.drivePIDRotKD = 0.067;

        MotionSettings.standardSettings.setEncoderTicksPerMetre(8816);

        Robot.startRobot(Robot::new);
    }
    
    public static OI oi = new OI();

    public static UsbCamera camera = null;
    
    // Subsystems
	public static SubsystemSensors sensors = new SubsystemSensors();
	public static SubsystemSignalling signalling = new SubsystemSignalling();
    public static SubsystemDriver driver = new SubsystemDriver();
    public static SubsystemIntake intake = new SubsystemIntake();
    public static SubsystemClimber climber = new SubsystemClimber();
    public static SubsystemElevator elevator = new SubsystemElevator();
    public static SubsystemVacuum vacuum = new SubsystemVacuum();
    public static SubsystemVision vision = new SubsystemVision();

    /**
     * Subsystem to primarily control the positon of the intake to ensure it doesn't colide with other parts of the robot.
     * Also calls function to handle LEDs.
     */
    public static SubsystemBase monitor = new SubsystemBase();

    public static void doMonitor(Action action) {
        if(sensors.elevatorSwitch.get()) sensors.elevatorEncoder.set(0);

        if(intake.shouldExtend()) intake.intakeCargoExtensionSol.set(true);
        else intake.intakeCargoExtensionSol.set(false);

        signalling.manageLEDs();
    }

	@Override
    public void initDefaultCommands() {
        driver.setDefaultCommand(drive.c());
        elevator.setDefaultCommand(elevatorLoop.c());
        sensors.setDefaultCommand(readSensors.c());
        vision.setDefaultCommand(readVision.c());

        monitor.setDefaultCommand(doMonitor.c());

        oi.mapOperations();
    }

    @Override
    public CommandBase getAutonomous(int autoID) {
        return Auto.getAutonomous(autoID);
    }

    public void initialiseRobot() {
        vision.ledMode.setNumber(vision.lightsOff);
        activateCamera(true);
    }
    
    public static void activateCamera(boolean shouldActivate) {
        try {
            if(camera == null && shouldActivate) camera = CameraServer.getInstance().startAutomaticCapture();

            if(shouldActivate) CameraServer.getInstance().getServer().setSource(camera);
            else CameraServer.getInstance().getServer().setSource(null);
        }
        catch(Exception e) {}
    }
    
    public void initialiseAuto() {
        sensors.resetDriveEncoders();
        sensors.elevatorEncoder.set(RobotMap.elevatorPosStartingConfiguraton);
        elevator.setpoint = RobotMap.elevatorPosStartingConfiguraton;
        sensors.yaw.set(0);
        SubsystemVision.pipeline.setNumber(Pipeline.LOW.id);
    }

    public void initialiseTeleop() {
        SubsystemVision.pipeline.setNumber(Pipeline.LOW.id);
    }

    public void executeDisabled() {
		int av = (int) SmartDashboard.getNumber("Auto Version", 0);
        SmartDashboard.putNumber("Auto Version", av);
        
        isIndivDriveControl = false;
    }
}