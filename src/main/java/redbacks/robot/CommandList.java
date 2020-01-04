package redbacks.robot;

import redbacks.arachne.core.references.CommandListStart;
import redbacks.arachne.ext.motion.pid.AcKillPID;
import redbacks.arachne.ext.motion.pid.AcPIDDynamicControl;
import redbacks.arachne.ext.motion.pid.AcSetIndivDrive;
import redbacks.arachne.ext.motion.pid.PIDPipe;
import redbacks.arachne.ext.motion.pid.Tolerances;
import redbacks.arachne.lib.actions.AcDoNothing;
import redbacks.arachne.lib.actions.AcInterrupt;
import redbacks.arachne.lib.actions.AcLambda;
import redbacks.arachne.lib.actions.AcSeq;
import redbacks.arachne.lib.actions.AcSetNumSen;
import redbacks.arachne.lib.actions.AcWait;
import redbacks.arachne.lib.actions.actuators.AcMotor;
import redbacks.arachne.lib.actions.actuators.AcSolenoid;
import redbacks.arachne.lib.actions.drive.AcDrive;
import redbacks.arachne.lib.actions.drive.AcTankDrive;
import redbacks.arachne.lib.checks.ChFalse;
import redbacks.arachne.lib.checks.ChTime;
import redbacks.arachne.lib.checks.ChTrue;
import redbacks.arachne.lib.checks.Check;
import redbacks.arachne.lib.checks.analog.ChNumSen;
import redbacks.arachne.lib.checks.digital.ChGettableBoolean;
import redbacks.arachne.lib.commands.CommandSetup;
import redbacks.arachne.lib.logic.RedbacksMath;
import redbacks.arachne.lib.sensors.NumericSensor;

import static redbacks.robot.Robot.*;
import static redbacks.robot.RobotMap.*;

import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.PIDSourceType;

import redbacks.robot.subsystems.SubsystemVision.Pipeline;

/**
 * @author Mitchell Barker, Darin Huang, Lucas Parker, Gabriela Ribeiro, Jon Reilly, Ben Schwarz, Sean Zammit
 */
public class CommandList extends CommandListStart {

    static {subsystemToUse = null;} 
    public static CommandSetup
        cameraActivate = newCom(new AcLambda(new ChTrue(), a -> activateCamera(true))),
        cameraDisable = newCom(new AcLambda(new ChTrue(), a -> activateCamera(false))),

        limelightSetLowest = newCom(Pipeline.LOW.activate()),
        limelightSetLeftmost = newCom(Pipeline.LEFT.activate()),
        limelightSetRightmost = newCom(Pipeline.RIGHT.activate()),

        intakeHatchSolOpen = newCom(new AcSolenoid.Single(intake.intakeHatchSol, true)),
        intakeHatchSolClose = newCom(new AcSolenoid.Single(intake.intakeHatchSol, false)),

        intakeCargoSolExtend = newCom(new AcSolenoid.Single(intake.intakeCargoSol, true)),
        intakeCargoSolRetract = newCom(new AcSolenoid.Single(intake.intakeCargoSol, false)),

        climberHorizontalSolExtend = newCom(new AcSolenoid.Double(climber.climberHorizontalSol, Value.kForward)),
        climberHorizontalSolRetract = newCom(new AcSolenoid.Double(climber.climberHorizontalSol, Value.kReverse)),
        climberHorizontalSolToggle = newCom(
            new AcLambda(new ChTrue(), a -> {
                if(climber.climberHorizontalSol.get() == Value.kForward) climber.climberHorizontalSol.set(Value.kReverse);
                else climber.climberHorizontalSol.set(Value.kForward);
            })
        ),

        climberRelease = newCom(new AcSolenoid.Single(climber.climberLockSol, true)),
        climberLock = newCom(new AcSolenoid.Single(climber.climberLockSol, false)),

        driverSetGrouped = newCom(new AcLambda(new ChTrue(), a -> isIndivDriveControl = false)),

        intakeCargoToggle = newCom(new AcLambda(new ChTrue(), a -> Robot.elevator.isCargo = !Robot.elevator.isCargo)),

        autoTrackToTarget = goToTarget(trackingPlaceSpeed, trackingPlaceSpeed),
        autoTrackToTargetSlow = goToTarget(trackingPlaceSpeedSlow, trackingPlaceSpeedSlow),
        
        limelightOn = newCom(vision.setLimelightMode(true)),
        limelightOff = newCom(vision.setLimelightMode(false)),

        killAll = newCom(new AcInterrupt.KillAllCommands(), new AcKillPID(), new AcLambda(new ChTrue(), a -> isIndivDriveControl = false));

    static {subsystemToUse = driver;}
    public static CommandSetup
        drive = newCom(new AcDrive(driver::arcadeDrive)),

        trackTarget = newCom(
            vision.setLimelightMode(true),
            new AcSetIndivDrive(),
            new AcPIDDynamicControl(new ChFalse(), false, 
                trackingRotKP, trackingRotKI, trackingRotKD, 
                vision::getIdealOffset,
                new Tolerances.Absolute(0), new NumericSensor() {
                    public double getSenVal() {
                        return vision.horizontalOffset.getDouble(0);
                    }
                }, false, 
                0, 0, PIDSourceType.kDisplacement, 
                -trackingRotMax, trackingRotMax,
                new PIDPipe(
                    d -> vision.isValidTarget() ? -d : 0,
                    new PIDPipe(d -> d - OI.axis_r_Y.get() * Math.abs(OI.axis_r_Y.get()), driver.leftMotor),
                    new PIDPipe(d -> d + OI.axis_r_Y.get() * Math.abs(OI.axis_r_Y.get()), driver.rightMotor)
                )
            )
        ),
        
        collectHatch = newCom(
            new AcSeq.Parallel(intakeHatchSolOpen),
            new AcSeq.Sequential(goToTarget()),
            new AcSeq.Parallel(intakeHatchSolClose)
        );

    static {subsystemToUse = climber;}
    public static CommandSetup 
        climberAnalog = newCom(
            new AcLambda(new ChFalse(), action -> {
                climber.climberVerticalMotor.set(RedbacksMath.signedPow(OI.axis_o_LY.get(), 2), action.command);
                climber.climberHorizontalMotor.set(RedbacksMath.signedPow(OI.axis_o_RX.get(), 2), action.command);
            })
        ),
        climberAuto = newCom(
            new AcSeq.Parallel(climberRelease),
            new AcWait(0.5),
            new AcMotor.Set(climber.climberVerticalMotor, climberVerticalSpeed, new ChGettableBoolean(sensors.climberVerticalSwitch, false)),
            new AcMotor.Set(climber.climberVerticalMotor, climberVerticalSpeed, new ChTime(0.5)),
            new AcMotor.Set(climber.climberVerticalMotor, climberVerticalSpeed, new ChGettableBoolean(sensors.climberVerticalSwitch, true)),
            new AcMotor.Disable(climber.climberVerticalMotor),
            new AcSeq.Parallel(driver, new AcTankDrive(new ChGettableBoolean(sensors.climberHorizontalSwitch, true), climberDriveSpeed, climberDriveSpeed)),
            new AcSeq.Parallel(climberHorizontalSolExtend),
            new AcMotor.Set(climber.climberHorizontalMotor, climberHorizontalSpeed, new ChGettableBoolean(sensors.climberHorizontalSwitch, true)),
            new AcMotor.Disable(climber.climberHorizontalMotor),
            new AcLambda(new ChFalse(), action -> climber.climberVerticalMotor.set(RedbacksMath.signedPow(OI.axis_o_LY.get(), 2), action.command))
        );
    
    static {subsystemToUse = intake;}
    public static CommandSetup
        intakeCargo = newCom( 
            new AcSeq.Parallel(intakeCargoSolExtend),
            new AcMotor.Set(intake.intakeMotor, intakeSpeed, new ChFalse())
        ),
        outtakeCargo = newCom(new AcMotor.Set(intake.intakeMotor, intakeDropSpeed, new ChFalse())),

        placeHatch = newCom(
            new AcSolenoid.Single(intake.intakeHatchSol, true),
            new AcSolenoid.Single(intake.intakeCargoSol, true),
            new AcWait(1),
            new AcSolenoid.Single(intake.intakeCargoSol, false)
        );
        
    static {subsystemToUse = vision;}
    public static CommandSetup readVision = newCom(new AcLambda(new ChFalse(), a -> vision.readVision()));

    static {subsystemToUse = sensors;}
    public static CommandSetup
        readSensors = newCom(new AcLambda(new ChFalse(), a -> sensors.readSensors())),
        resetDriveEncoders = newCom(new AcLambda(new ChFalse(), a -> sensors.resetDriveEncoders())),
        resetGyro = newCom(new AcSetNumSen(Robot.sensors.yaw, 0));
    
    static {subsystemToUse = monitor;}
    public static CommandSetup doMonitor = newCom(new AcLambda(new ChFalse(), Robot::doMonitor));

    static {subsystemToUse = vacuum;}
    public static CommandSetup 
        vacuumActivate = newCom(
            new AcSolenoid.Single(vacuum.vacuumSol, true), 
            new AcWait(2),
            new AcSolenoid.Single(vacuum.vacuumSol, false)
        );

    static {subsystemToUse = elevator;}
    public static CommandSetup
        elevatorLoop = newCom(
                new AcPIDDynamicControl(
                        new ChFalse(), false,
                        elevatorKP, elevatorKI, elevatorKD,
                        () -> elevator.setpoint, new Tolerances.Absolute(0),
                        sensors.elevatorEncoder, false, elevatorPosMin, elevatorPosMax, PIDSourceType.kDisplacement,
                        elevatorSpeedMin, elevatorSpeedMax,
                        new PIDPipe(elevator::addCompensation, elevator.elevatorMotor.controller)
                )
        ),
        elevatorToGround = newCom(
                new AcLambda(new ChTrue(), a -> elevator.setpoint = 0),
                new AcLambda(new ChNumSen(elevatorPosBumper, sensors.elevatorEncoder, false, false, false),
                        action -> elevator.elevatorMotor.set((sensors.elevatorEncoder.get() - elevatorPosBumper) / (elevatorPosMax - elevatorPosBumper) * (elevatorResetSpeedMax - elevatorResetSpeedMin) + elevatorResetSpeedMin, action.command)
                ),
                new AcMotor.Set(elevator.elevatorMotor, elevatorResetSpeedMin, new ChGettableBoolean(sensors.elevatorSwitch, true))
        ),
        elevatorAnalog = newCom(
                new AcLambda(new ChFalse(),  action -> {
                        elevator.setpoint = sensors.elevatorEncoder.get();
                        elevator.elevatorMotor.set(-OI.axis_o_LY.get(), action.command);
                })
        );

    static {subsystemToUse = null;} 
    public static CommandSetup
        elevatorToHatch1 = elevatorToGround,
        elevatorToHatch2 = newCom(elevator.setElevator(elevatorPosHatch2, false)),
        elevatorToHatch3 = newCom(elevator.setElevator(elevatorPosHatch3, false)),
        elevatorToHatchShip = elevatorToGround,

        elevatorToCargo1 = newCom(elevator.setElevator(elevatorPosCargo1, true)),
        elevatorToCargo2 = newCom(elevator.setElevator(elevatorPosCargo2, true)),
        elevatorToCargo3 = newCom(elevator.setElevator(elevatorPosCargo3, true)),
        elevatorToCargoShip = newCom(elevator.setElevator(elevatorPosCargoShip, false)),

        deployElevator = newCom(
            new AcWait(0.5),
            new AcSeq.Parallel(elevatorToHatch3),
            new AcDoNothing(new ChNumSen(elevatorPosHatch3 - elevatorTolerancePos, sensors.elevatorEncoder, true, true, false)),
            new AcSeq.Parallel(elevatorToGround)
        ),
        deployElevatorShort = newCom(
            // new AcLambda(new ChTrue(), action -> elevator.isCargo = true),
            new AcWait(1),
            // elevator.setElevator(elevatorPosDeploy, false),
            // new AcDoNothing(new ChNumSen(elevatorPosDeploy - elevatorTolerancePos, sensors.elevatorEncoder, true, true, false)),
            new AcSeq.Parallel(elevatorToGround)
        );

    public static CommandSetup goToTarget() {
        return goToTarget(new ChGettableBoolean(sensors.hatchSwitch, true));
    }

    public static CommandSetup goToTarget(Check check) {
        return goToTarget(check, trackingCloseSpeed, trackingFarSpeed);
    }

    public static CommandSetup goToTarget(double speedClose, double speedFar) {
        return goToTarget(new ChGettableBoolean(sensors.hatchSwitch, true), speedClose, speedFar);
    }

    private static class Distance {
        public double value = trackingCloseDis + 1;
    }
    
    public static CommandSetup goToTarget(Check check, double speedClose, double speedFar) {
        final Distance lastDistance = new Distance();

        return newCom(
            vision.setLimelightMode(true),
            new AcSetIndivDrive(),
            new AcPIDDynamicControl(
                check, false,
                trackingRotKP, trackingRotKI, trackingRotKD,
                vision::getIdealOffset,
                new Tolerances.Absolute(0), new NumericSensor() {
                    public double getSenVal() {
                        return vision.horizontalOffset.getDouble(0);
                    }
                }, false, 
                0, 0, PIDSourceType.kDisplacement, 
                -trackingRotMax, trackingRotMax,
                new PIDPipe(
                    d -> {
                        if(vision.isValidTarget()) {
                            lastDistance.value = vision.getStraightLineDistance();
                            return -d;
                        }

                        return 0;
                    },
                    new PIDPipe(d -> d + (lastDistance.value > trackingCloseDis ? speedFar : speedClose), driver.leftMotor),
                    new PIDPipe(d -> d - (lastDistance.value > trackingCloseDis ? speedFar : speedClose), driver.rightMotor)
                )
            ),
            new AcLambda(new ChTrue(), a -> Robot.isIndivDriveControl = false),
            vision.setLimelightMode(false)
        );
    }
}
