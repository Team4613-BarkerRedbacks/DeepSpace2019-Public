package redbacks.robot;

import redbacks.arachne.core.references.AutoStart;
import redbacks.arachne.lib.actions.AcDoNothing;
import redbacks.arachne.lib.actions.AcLambda;
import redbacks.arachne.lib.actions.AcSeq;
import redbacks.arachne.lib.actions.AcSetNumSen;
import redbacks.arachne.lib.actions.AcWait;
import redbacks.arachne.lib.actions.Action;
import redbacks.arachne.lib.actions.drive.AcDriveArc;
import redbacks.arachne.lib.actions.drive.AcDriveDirection;
import redbacks.arachne.lib.actions.drive.AcTankDrive;
import redbacks.arachne.lib.checks.ChTime;
import redbacks.arachne.lib.checks.ChTrue;
import redbacks.arachne.lib.checks.analog.ChNumSen;
import redbacks.arachne.lib.checks.digital.ChGettableBoolean;
import redbacks.arachne.lib.commands.CommandBase;
import redbacks.robot.actions.AcStraight;
import redbacks.robot.actions.AcTurn;
import redbacks.robot.subsystems.SubsystemVision.Pipeline;

import static redbacks.robot.Robot.*;
import static redbacks.robot.CommandList.*;
import static redbacks.robot.RobotMap.*;

import static redbacks.arachne.ext.motion.MotionSettings.standardSettings;
/**
 * @author Mitchell Barker, Darin Huang, Lucas Parker, Gabriela Ribeiro, Jon Reilly, Ben Schwarz, Sean Zammit
 */
public class Auto extends AutoStart {
    public static CommandBase getAutonomous(int autoNumber) {
        switch(autoNumber) {
            case -1:
                return createAuto();
      
            // R1-Rh1-RH1
            case 1:
                return createAuto(
                    new AcSetNumSen(sensors.yaw, 90),
                    new AcSeq.Parallel(
                        new AcWait(1.5),
                        new AcSeq.Parallel(elevatorToGround)
                    ),
                    // Score 1st hatch
                    new AcStraight(standardSettings, -5.8, 105, sensors.averageEncoder, true),
                    new AcTurn(standardSettings, 50),
                    new AcSeq.Sequential(autoTrackToTarget),
                    new AcSeq.Parallel(placeHatch),
                    // Collect 2nd hatch
                    doStraightDistance(0.4, 30, -0.7),
                    doStraightDistance(0.5, 90, 0.8),
                    doStraightDistance(2, 80, 0.9),
                    new AcSeq.Sequential(goToTarget()),
                    new AcSeq.Parallel(intakeHatchSolClose),
                    new AcWait(0.2),
                    new AcTankDrive(new ChTime(0.2), -0.2, -0.2),
                    // Score 2nd hatch
                    doStraightDistance(2.5, 80, -0.8),
                    new AcTurn(standardSettings, -50),
                    new AcSeq.Sequential(autoTrackToTarget),
                    new AcSeq.Parallel(placeHatch),
                    new AcTankDrive(new ChTime(0.5), -0.3, -0.3)
                );  
 
            // C1-CH5-CH6
            case 2: 
                return createAuto(
                    new AcSetNumSen(sensors.yaw, 90),
                    Pipeline.RIGHT.activate(),
                    new AcSeq.Parallel(
                        new AcWait(1),
                        new AcSeq.Parallel(elevatorToGround)
                    ),
                    // Score 1st hatch
                    doStraightDistance(1, 90, 0.75),
                    new AcDoNothing(new ChNumSen(elevatorLimelightPosMax, sensors.elevatorEncoder, false, true, false)),
                    new AcSeq.Sequential(autoTrackToTarget),
                    new AcSeq.Parallel(placeHatch),
                    // Collect 2nd hatch
                    new AcDriveArc(-0.9, -45, sensors.averageEncoder, 3),
                   // new AcDriveArc(-0.9, -60, sensors.averageEncoder, 3.5), potential change
                    doStraightDistance(1, -90, 0.9),
                    Pipeline.LOW.activate(),
                    new AcSeq.Sequential(goToTarget()),
                    new AcSeq.Parallel(intakeHatchSolClose),
                    new AcWait(0.2),
                    new AcTankDrive(new ChTime(0.2), -0.2, -0.2),
                    // Score 2nd hatch
                    new AcStraight(standardSettings, -6, -100, sensors.averageEncoder, true),
                    new AcTurn(standardSettings, 0),
                    Pipeline.LEFT.activate(),
                    new AcSeq.Sequential(autoTrackToTarget),
                    new AcSeq.Parallel(placeHatch),
                    new AcTankDrive(new ChTime(0.5), -0.3, -0.3)
                );

            // R2-Rh1-RH1
            case 3:
                return createAuto(
                    new AcSetNumSen(sensors.yaw, -90),
                    new AcSeq.Parallel(
                        new AcWait(2),
                        new AcSeq.Parallel(elevatorToGround)
                    ),
                    // Score 1st hatch
                    doStraightDistance(1.5, -90, 0.75),
                    doStraightDistance(4.5, -72, 0.75),
                    new AcTurn(standardSettings, 42),
                    new AcSeq.Sequential(autoTrackToTarget),
                    new AcSeq.Parallel(placeHatch),
                    // Collect 2nd hatch
                    doStraightDistance(0.4, 42, -0.7),
                    doStraightDistance(0.5, 90, 0.8),
                    doStraightDistance(2, 80, 0.9),
                    new AcSeq.Sequential(goToTarget()),
                    new AcSeq.Parallel(intakeHatchSolClose),
                    new AcWait(0.2),
                    new AcTankDrive(new ChTime(0.2), -0.2, -0.2),
                    // Score 2nd hatch
                    doStraightDistance(2.5, 80, -0.8),
                    new AcTurn(standardSettings, -50),
                    new AcSeq.Sequential(autoTrackToTarget),
                    new AcSeq.Parallel(placeHatch),
                    new AcTankDrive(new ChTime(0.5), -0.3, -0.3)
                );
            default:
            //R2-CH5-CH6
            case 4:
                return createAuto(
                    new AcSetNumSen(sensors.yaw, 90),
                    Pipeline.RIGHT.activate(),
                    new AcSeq.Parallel(
                        new AcWait(2),
                        new AcSeq.Parallel(elevatorToGround)
                    ),
                    // Score 1st hatch
                    doStraightDistance(1, 90, 0.75),
                    //Previous Variant
                    //doStraightDistance(1.3, 70, 0.8),
                    //New Variant
                    doStraightDistance(1.3, 75, 0.8),
                    new AcWait(0.8),
                    //End Variant
                    new AcSeq.Sequential(autoTrackToTarget),
                    new AcSeq.Parallel(placeHatch),
                    // Collect 2nd hatch
                    new AcDriveArc(-0.9, -45, sensors.averageEncoder, 3),
                    doStraightDistance(1.2, -90, 0.9),
                    Pipeline.LOW.activate(),
                    new AcSeq.Sequential(goToTarget()),
                    new AcSeq.Parallel(intakeHatchSolClose),
                    new AcWait(0.2),
                    new AcTankDrive(new ChTime(0.2), -0.2, -0.2),
                    // Score 2nd hatch
                    new AcStraight(standardSettings, -6, -100, sensors.averageEncoder, true),
                    new AcTurn(standardSettings, 0),
                    Pipeline.LEFT.activate(),
                    new AcSeq.Sequential(autoTrackToTarget),
                    new AcSeq.Parallel(placeHatch),
                    new AcTankDrive(new ChTime(0.5), -0.3, -0.3)
                );

            //C1-CH4-CH3
            case 5:
                return createAuto(
                    new AcSetNumSen(sensors.yaw, -90),
                    Pipeline.LEFT.activate(),
                    new AcSeq.Parallel(
                        new AcWait(1),
                        new AcSeq.Parallel(elevatorToGround)
                    ),
                    // Score 1st hatch
                    doStraightDistance(1, -90, 0.75),
                    new AcDoNothing(new ChNumSen(elevatorLimelightPosMax, sensors.elevatorEncoder, false, true, false)),
                    new AcSeq.Sequential(autoTrackToTarget),
                    new AcSeq.Parallel(placeHatch),
                    // Collect 2nd hatch
                    new AcDriveArc(-0.9, 45, sensors.averageEncoder, 3.5),
                    doStraightDistance(1, 90, 0.9),
                    Pipeline.LOW.activate(),
                    new AcSeq.Sequential(goToTarget()),
                    new AcSeq.Parallel(intakeHatchSolClose),
                    new AcWait(0.2),
                    new AcTankDrive(new ChTime(0.2), -0.2, -0.2),
                    // Score 2nd hatch
                    new AcStraight(standardSettings, -6, 100, sensors.averageEncoder, true),
                    new AcTurn(standardSettings, 0),
                    Pipeline.RIGHT.activate(),
                    new AcSeq.Sequential(autoTrackToTarget),
                    new AcSeq.Parallel(placeHatch),
                    new AcTankDrive(new ChTime(0.5), -0.3, -0.3)
                );

            //L2-CH4-CH3
            case 6:
                return createAuto(
                    new AcSetNumSen(sensors.yaw, -90),
                    Pipeline.LEFT.activate(),
                    new AcSeq.Parallel(
                        new AcWait(1.5),
                        new AcSeq.Parallel(elevatorToGround)
                    ),
                    // Score 1st hatch
                    doStraightDistance(1, -90, 0.75),
                    doStraightDistance(1.3, -70, 0.8),
                    new AcDoNothing(new ChNumSen(elevatorLimelightPosMax, sensors.elevatorEncoder, false, true, false)),
                    new AcSeq.Sequential(autoTrackToTarget),
                    new AcSeq.Parallel(placeHatch),
                    // Collect 2nd hatch
                    new AcDriveArc(-0.9, 45, sensors.averageEncoder, 3.5),
                    doStraightDistance(1.2, 90, 0.9),
                    Pipeline.LOW.activate(),
                    new AcSeq.Sequential(goToTarget()),
                    new AcSeq.Parallel(intakeHatchSolClose),
                    new AcWait(0.2),
                    new AcTankDrive(new ChTime(0.2), -0.2, -0.2),
                    // Score 2nd hatch
                    new AcStraight(standardSettings, -6, 100, sensors.averageEncoder, true),
                    new AcTurn(standardSettings, 0),
                    Pipeline.RIGHT.activate(),
                    new AcSeq.Sequential(autoTrackToTarget),
                    new AcSeq.Parallel(placeHatch),
                    new AcTankDrive(new ChTime(0.5), -0.3, -0.3)
                );

            // L1-Lh1-LH1
            case 7:
                return createAuto(
                    new AcSetNumSen(sensors.yaw, -90),
                    new AcSeq.Parallel(
                        new AcWait(1),
                        new AcSeq.Parallel(elevatorToGround)
                    ),
                    // Score 1st hatch
                    new AcStraight(standardSettings, -5.8, -105, sensors.averageEncoder, true),
                    new AcTurn(standardSettings, -50),
                    new AcSeq.Sequential(autoTrackToTarget),
                    new AcSeq.Parallel(placeHatch),
                    // Collect 2nd hatch
                    doStraightDistance(0.4, -30, -0.7),
                    doStraightDistance(0.5, -90, 0.8),
                    doStraightDistance(2, -80, 0.9),
                    new AcSeq.Sequential(goToTarget()),
                    new AcSeq.Parallel(intakeHatchSolClose),
                    new AcWait(0.2),
                    new AcTankDrive(new ChTime(0.2), -0.2, -0.2),
                    // Score 2nd hatch
                    doStraightDistance(2.5, -80, -0.8),
                    new AcTurn(standardSettings, 40),
                    new AcSeq.Sequential(autoTrackToTarget),
                    new AcSeq.Parallel(placeHatch),
                    new AcTankDrive(new ChTime(0.5), -0.3, -0.3)
                );  
            // L2-Lh1-LH1 
            case 8:
                return createAuto(
                    new AcSetNumSen(sensors.yaw, -90),
                    new AcSeq.Parallel(
                        new AcWait(1),
                        new AcSeq.Parallel(elevatorToGround)
                    ),
                    // Score 1st hatch
                    doStraightDistance(0.5, -90, -0.7),
                    new AcStraight(standardSettings, -5.8, -105, sensors.averageEncoder, true),
                    new AcTurn(standardSettings, -50),
                    new AcSeq.Sequential(autoTrackToTarget),
                    new AcSeq.Parallel(placeHatch),
                    // Collect 2nd hatch
                    doStraightDistance(0.4, -30, -0.7),
                    doStraightDistance(0.5, -90, 0.8),
                    doStraightDistance(2, -80, 0.9),
                    new AcSeq.Sequential(goToTarget()),
                    new AcSeq.Parallel(intakeHatchSolClose),
                    new AcWait(0.2),
                    new AcTankDrive(new ChTime(0.2), -0.2, -0.2),
                    // Score 2nd hatch
                    doStraightDistance(2.5, -80, -0.8),
                    new AcTurn(standardSettings, 40),
                    new AcSeq.Sequential(autoTrackToTarget),
                    new AcSeq.Parallel(placeHatch),
                    new AcTankDrive(new ChTime(0.5), -0.3, -0.3)
                );      
        }
    }
    
    private static Action doStraightDistance(double distance, double angle, double speed) {
        return new AcDriveDirection(new ChNumSen(distance * standardSettings.getEncoderTicksPerMetre(), sensors.averageEncoder), speed, angle);
    }
}