package redbacks.robot.subsystems;

import static redbacks.robot.RobotMap.*;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import redbacks.arachne.core.SubsystemBase;
import redbacks.arachne.ext.ctre.sensors.SenCANEncoder;
import redbacks.arachne.ext.navx.sensors.NavX;
import redbacks.arachne.ext.navx.sensors.NavXReading;
import redbacks.arachne.lib.logic.RedbacksMath;
import redbacks.arachne.lib.sensors.BinarySensor;
import redbacks.arachne.lib.sensors.ModdedSensor;
import redbacks.arachne.lib.sensors.NumericSensor;
import redbacks.arachne.lib.sensors.SenDI;
import redbacks.robot.OI;
import redbacks.robot.Robot;

/**
 * @author Mitchell Barker, Darin Huang, Lucas Parker, Gabriela Ribeiro, Jon Reilly, Ben Schwarz, Sean Zammit
 */
public class SubsystemSensors extends SubsystemBase {

    public RobotPosition robotPosition = RobotPosition.NO_LINE;

    public SenCANEncoder.Displacement driveLeftEncoder = new SenCANEncoder.Displacement(idMotDriveL2);
    public SenCANEncoder.Displacement driveRightEncoder = new SenCANEncoder.Displacement(idMotDriveR1);

    public SenCANEncoder.Displacement droppedLeftEncoder = new SenCANEncoder.Displacement(idMotDriveL3);
    public SenCANEncoder.Displacement droppedRightEncoder = new SenCANEncoder.Displacement(idMotDriveR2);
    
    public SenCANEncoder.Displacement elevatorEncoder = new SenCANEncoder.Displacement(talon5);

    public SenDI climberVerticalSwitch = new SenDI(new DigitalInput(idSenClimbSwitchV));
    public SenDI climberHorizontalSwitch = new SenDI(new DigitalInput(idSenClimbSwitchH));

    // public SenDI hatchSwitch = new SenDI(new DigitalInput(idSenHatchSwitch));
    public BinarySensor hatchSwitch = new BinarySensor() {
        private DigitalInput di = new DigitalInput(idSenHatchSwitch);

        protected boolean getSenVal() {
            return !OI.o_X.get() && di.get();
        }
    };

    public SenDI lineSensorL = new SenDI(new DigitalInput(idSenLineL));
    public SenDI lineSensorC = new SenDI(new DigitalInput(idSenLineC));
    public SenDI lineSensorR = new SenDI(new DigitalInput(idSenLineR));

    public SenDI elevatorSwitch = new SenDI(new DigitalInput(idSenElevatorSwitch));

    public NumericSensor averageEncoder = new NumericSensor() {
        protected double getSenVal() {
            return (driveLeftEncoder.get() + driveRightEncoder.get()) / 2;
        }
    };

    public SenCANEncoder.Rate rateEncoderLeft = new SenCANEncoder.Rate(idMotDriveL1);
    public SenCANEncoder.Rate rateEncoderRight = new SenCANEncoder.Rate(idMotDriveR1);
    public NumericSensor rateEncoderAverage = new NumericSensor() {
        protected double getSenVal() {
            return(-rateEncoderLeft.get() + rateEncoderRight.get()) / 2;
        }
    };

    public ModdedSensor holdingEncoder = new ModdedSensor(averageEncoder, RedbacksMath::signedSqrt);
    public ModdedSensor holdingEncoderL = new ModdedSensor(driveLeftEncoder, RedbacksMath::signedSqrt);
    public ModdedSensor holdingEncoderR = new ModdedSensor(driveRightEncoder, RedbacksMath::signedSqrt);

    public NavX.Sensor pitch = new NavX.Sensor(NavXReading.ANGLE_PITCH);
	public NavX.Sensor roll = new NavX.Sensor(NavXReading.ANGLE_ROLL);
	public NavX.Yaw yaw = new NavX.Yaw();

	public NavX.Sensor ratePitch = new NavX.Sensor(NavXReading.RATE_PITCH);
	public NavX.Sensor rateRoll = new NavX.Sensor(NavXReading.RATE_ROLL);
	public NavX.Sensor rateYaw = new NavX.Sensor(NavXReading.RATE_YAW);

	public NavX.Sensor accelForward = new NavX.Sensor(NavXReading.ACCEL_FORWARD);
	public NavX.Sensor accelRight = new NavX.Sensor(NavXReading.ACCEL_RIGHT);
	public NavX.Sensor accelUp = new NavX.Sensor(NavXReading.ACCEL_UP);

	public NavX.Sensor speedForward = new NavX.Sensor(NavXReading.SPEED_FORWARD);
	public NavX.Sensor speedRight = new NavX.Sensor(NavXReading.SPEED_RIGHT);
    public NavX.Sensor speedUp = new NavX.Sensor(NavXReading.SPEED_UP);

    public SubsystemSensors() {
        super();

        resetSensors();
        
        climberVerticalSwitch.setInverted(true);
        climberHorizontalSwitch.setInverted(true);

        hatchSwitch.setInverted(true);

        lineSensorL.setInverted(true);
        lineSensorR.setInverted(true);

        elevatorSwitch.setInverted(true);

        driveLeftEncoder.setScaleFactor(-1);

        yaw.set(0);
    }

    public void readSensors() {
        SmartDashboard.putNumber("Elevator Encoder", elevatorEncoder.get());
        SmartDashboard.putNumber("Elevator Setpoint", Robot.elevator.setpoint);
        SmartDashboard.putNumber("Elevator Speed", Robot.elevator.elevatorMotor.get());
        SmartDashboard.putBoolean("Elevator LS", elevatorSwitch.get());

        SmartDashboard.putBoolean("Climber LS Vertical", climberVerticalSwitch.get());
        SmartDashboard.putBoolean("Climber LS Horizontal", climberHorizontalSwitch.get());

        SmartDashboard.putBoolean("Hatch LS", hatchSwitch.get());

        SmartDashboard.putBoolean("Line L", lineSensorL.get());
        SmartDashboard.putBoolean("Line C", lineSensorC.get());
        SmartDashboard.putBoolean("Line R", lineSensorR.get());

        SmartDashboard.putNumber("Gyro", yaw.get());
        SmartDashboard.putNumber("Drive Encoder", averageEncoder.get());
        SmartDashboard.putNumber("Drive Left Encoder", driveLeftEncoder.get());
        SmartDashboard.putNumber("Drive Right Encoder", driveRightEncoder.get());

        SmartDashboard.putNumber("Speed Forward", speedForward.get());
        SmartDashboard.putNumber("Speed Right", speedRight.get());
        SmartDashboard.putNumber("Speed Up", speedUp.get());
        
        SmartDashboard.putNumber("Rate Encoder", rateEncoderAverage.get());
    }

    public void resetSensors() {
        resetDriveEncoders();
        elevatorEncoder.set(0);
    }

    public void resetDriveEncoders() {
        driveLeftEncoder.set(0);
        driveRightEncoder.set(0);
    }

    public enum RobotPosition {

        LINE_BELOW,
        LINE_RIGHT,
        LINE_LEFT,
        NO_LINE;
    }

    /**
     * Sets the robotPosition to a value of the {@link RobotPosition} enum based off 2 digital line sensors.
     */
    public void determineRobotLinePositon() {
        boolean leftOnLine = lineSensorL.get(), rightOnLine = lineSensorR.get();

        if(leftOnLine && rightOnLine) robotPosition = RobotPosition.LINE_BELOW;
        else if(!leftOnLine && rightOnLine) robotPosition = RobotPosition.LINE_RIGHT;
        else if(leftOnLine && !rightOnLine) robotPosition = RobotPosition.LINE_LEFT;
        else robotPosition = RobotPosition.NO_LINE;
    }
}
