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
public class OldAuto extends AutoStart {

    public static CommandBase getAutonomous(int autoNumber) {
		switch(autoNumber) {
            // Nothing
            case -1:
                return createAuto();

            // R1-Rh3-[RH3]
            case 2:
                return createAuto(
                    new AcSetNumSen(sensors.yaw, 90),
                    new AcSeq.Parallel(
                        new AcWait(0.5),
                        new AcSeq.Parallel(elevatorToHatch2)
                    ),
                    // Score 1st hatch [base 6]
                    new AcStraight(standardSettings, -5.5, 105, sensors.averageEncoder, true),
                    new AcTurn(standardSettings, 50),
                    new AcSeq.Parallel(elevatorToHatch3),
                    new AcDoNothing(new ChNumSen(elevatorPosHatch3 - elevatorTolerancePos, sensors.elevatorEncoder, true, true, false)),
                    new AcSeq.Sequential(autoTrackToTarget),
                    new AcSeq.Parallel(placeHatch),
                    // Collect 2nd hatch [base 6]
                    doStraightDistance(0.4, 30, -0.7),
                    new AcSeq.Parallel(elevatorToGround),
                    doStraightDistance(0.5, 90, 0.8),
                    doStraightDistance(2, 80, 0.9),
                    new AcSeq.Sequential(goToTarget()),
                    new AcSeq.Parallel(intakeHatchSolClose),
                    new AcWait(0.3),
                    new AcTankDrive(new ChTime(0.2), -0.2, -0.2),
                    // Score 2nd hatch [base 6]
                    doStraightDistance(2.5, 80, -0.8),
                    new AcTurn(standardSettings, -40),
                    new AcSeq.Parallel(elevatorToHatch3),
                    new AcDoNothing(new ChNumSen(elevatorPosHatch3 - elevatorTolerancePos, sensors.elevatorEncoder, true, true, false)),
                    new AcSeq.Sequential(autoTrackToTarget),
                    new AcSeq.Parallel(placeHatch),
                    new AcTankDrive(new ChTime(0.5), -0.3, -0.3)
                );

            // R1-Rh1-RH1
            default:
            case 6:
                return createAuto(
                    new AcSetNumSen(sensors.yaw, 90),
                    new AcSeq.Parallel(deployElevator),
                    // Score 1st hatch
                    new AcStraight(standardSettings, -5.5, 105, sensors.averageEncoder, true),
                    new AcTurn(standardSettings, 50),
                    new AcSeq.Sequential(autoTrackToTarget),
                    new AcSeq.Parallel(placeHatch),
                    // Collect 2nd hatch
                    doStraightDistance(0.4, 30, -0.7),
                    doStraightDistance(0.5, 90, 0.8),
                    doStraightDistance(2, 80, 0.9),
                    new AcSeq.Sequential(goToTarget()),
                    new AcSeq.Parallel(intakeHatchSolClose),
                    new AcWait(0.3),
                    new AcTankDrive(new ChTime(0.2), -0.2, -0.2),
                    // Score 2nd hatch
                    doStraightDistance(2.5, 80, -0.8),
                    new AcTurn(standardSettings, -50),
                    new AcSeq.Sequential(autoTrackToTarget),
                    new AcSeq.Parallel(placeHatch),
                    new AcTankDrive(new ChTime(0.5), -0.3, -0.3)
                );
            
            //L1-Rh1-RH1
            case 7: 
                return createAuto(
                    new AcSetNumSen(sensors.yaw, -90),
                    new AcSeq.Parallel(deployElevator),
                    // Score 1st hatch
                    new AcStraight(standardSettings, -6, -105, sensors.averageEncoder, true),
                    new AcTurn(standardSettings, -50),
                    new AcSeq.Sequential(autoTrackToTarget),
                    new AcSeq.Parallel(placeHatch),
                    // Collect 2nd hatch
                    doStraightDistance(0.4, -30, -0.7),
                    doStraightDistance(0.5, -90, 0.8),
                    doStraightDistance(2, -80, 0.9),
                    new AcSeq.Sequential(goToTarget()),
                    new AcSeq.Parallel(intakeHatchSolClose),
                    new AcWait(0.3),
                    new AcTankDrive(new ChTime(0.2), -0.2, -0.2),
                    // Score 2nd hatch
                    doStraightDistance(2.5, -80, -0.8),
                    new AcTurn(standardSettings, 50),
                    new AcSeq.Sequential(autoTrackToTarget),
                    new AcSeq.Parallel(placeHatch),
                    new AcTankDrive(new ChTime(0.5), -0.3, -0.3)
                );

            // L1-CH3-CH2
            case 11: return l_CH3_CH2(false);

            // R1-CH6-CH7
            case 12: 
                return createAuto(
                    Pipeline.LEFT.activate(),
                    new AcSetNumSen(sensors.yaw, -90),
                    // Descend and deploy
                    new AcSeq.Parallel(deployElevator),
                    // Score 1st hatch [Common to all r_CH6 autos]
                    new AcStraight(standardSettings, -5.3, -80, sensors.averageEncoder, true),
                    new AcTurn(standardSettings, 0),
                    new AcSeq.Sequential(autoTrackToTarget),
                    new AcSeq.Parallel(placeHatch),
                    new AcTankDrive(new ChTime(0.5), -0.5, -0.5),
                    // Collect 2nd hatch [New path]
                    new AcTurn(standardSettings, -90),
                    new AcDriveArc(0.9, -125, sensors.averageEncoder, 1.4),
                    new AcSeq.Parallel(limelightOn),
                    new AcDriveArc(new ChGettableBoolean(vision::isValidTarget, true), 0.9, -95, sensors.averageEncoder, 2),
                    new AcDriveDirection(new ChGettableBoolean(vision::isValidTarget, true), 0.6, -90),
                    new AcSeq.Sequential(goToTarget()),
                    new AcSeq.Parallel(intakeHatchSolClose),
                    new AcWait(0.3),
                    new AcTankDrive(new ChTime(0.2), -0.2, -0.2),
                    // Score 2nd hatch [Same as below]
                    new AcStraight(standardSettings, -6.6, -98, sensors.averageEncoder, true),
                    new AcTurn(standardSettings, 0),
                    Pipeline.CENTRE.activate(),
                    new AcSeq.Sequential(autoTrackToTarget),
                    new AcSeq.Parallel(placeHatch),
                    new AcTankDrive(new ChTime(0.5), -0.3, -0.3)
                );

            // R1-CH6-CH5
            case 14: return r_CH6_CH5(false, true);

            // R1-CH6-RH1
            case 18: return r_CH6_RH1(false, true);

            // L2-CH3-CH2
            case 21: return l_CH3_CH2(true);

            // R2-CH6-CH7
            case 22: return r_CH6_CH7(true, true);

            // R2-CH6-CH5
            case 24: return r_CH6_CH5(true, true);

            // R2-CH6-RH1
            case 28: return r_CH6_RH1(true, true);

            // [OLD] R1-RH3-[Rh3]
            // Auto 2 from Australian Regional events. Scores 1.9 hatches on level 3 of the rocket.
            // DO NOT USE. Breaks elevator during turn.
            case 52:
                return createAuto(
                    new AcSetNumSen(sensors.yaw, -90),
                    new AcSeq.Parallel(
                        new AcWait(0.5),
                        new AcSeq.Sequential(elevatorToHatch3)),
                    // Score 1st hatch
                    doStraightDistance(2, -60, 0.7),
                    new AcSeq.Sequential(autoTrackToTarget),
                    new AcSeq.Parallel(placeHatch),
                    // Collect 2nd hatch
                    new AcDriveArc(-0.6, 75, sensors.averageEncoder, 0.85),
                    new AcSeq.Parallel(elevatorToGround),
                    doStraightDistance(0.5, 75, 0.55),
                    new AcSeq.Sequential(autoTrackToTargetSlow),
                    new AcSeq.Parallel(intakeHatchSolClose),
                    new AcWait(0.3),
                    new AcTankDrive(new ChTime(0.3), -0.2, -0.2),
                    // Align for 2nd hatch
                    new AcSeq.Parallel(
                        new AcWait(0.5),
                        new AcSeq.Parallel(elevatorToHatch2)    
                    ),
                    doStraightDistance(4, 82, -0.85),
                    doStraightDistance(2, 90, -0.6),
                    new AcSeq.Parallel(elevatorToHatch3),
                    new AcTurn(standardSettings, 40)
                );

            // [OLD] R1-CH6-CH7
            case 62: return r_CH6_CH7(false, false);

            // [OLD] R1-CH6-CH5
            case 64: return r_CH6_CH5(false, false);

            // [OLD] R1-CH6-RH1
            case 68: return r_CH6_RH1(false, false);

            // [OLD] R2-CH6-CH7
            case 72: return r_CH6_CH7(true, false);

            // R2-CH6-CH5
            case 74: return r_CH6_CH5(true, false);

            // R2-CH6-RH1
            case 78: return r_CH6_RH1(true, false);
        }
    }

    private static CommandBase r_CH6_CH7(boolean level2, boolean newPath) {
        if(newPath) {
            return createAuto(
                Pipeline.LEFT.activate(),
                new AcSetNumSen(sensors.yaw, -90),
                // Descend and deploy
                new AcSeq.Parallel(level2 ? descendFromRightLevel2() : new AcDoNothing(new ChTrue())),
                new AcSeq.Parallel(level2 ? deployElevatorShort : deployElevator),
                // Score 1st hatch [Common to all r_CH6 autos]
                new AcStraight(standardSettings, -5.6, -82, sensors.averageEncoder, true),
                new AcTurn(standardSettings, 0),
                new AcSeq.Sequential(autoTrackToTarget),
                new AcSeq.Parallel(placeHatch),
                new AcTankDrive(new ChTime(0.5), -0.5, -0.5),
                // Collect 2nd hatch [New path]
                new AcTurn(standardSettings, -90),
                new AcDriveArc(0.9, -125, sensors.averageEncoder, 1.4),
                new AcSeq.Parallel(limelightOn),
                new AcDriveArc(new ChGettableBoolean(vision::isValidTarget, true), 0.9, -95, sensors.averageEncoder, 2),
                new AcDriveDirection(new ChGettableBoolean(vision::isValidTarget, true), 0.6, -90),
                new AcSeq.Sequential(goToTarget()),
                new AcSeq.Parallel(intakeHatchSolClose),
                new AcWait(0.3),
                new AcTankDrive(new ChTime(0.2), -0.2, -0.2),
                // Score 2nd hatch [Same as below]
                new AcStraight(standardSettings, -6.8, -98, sensors.averageEncoder, true),
                new AcTurn(standardSettings, 0),
                Pipeline.CENTRE.activate(),
                new AcSeq.Sequential(autoTrackToTarget),
                new AcSeq.Parallel(placeHatch),
                new AcTankDrive(new ChTime(0.5), -0.3, -0.3)
            );
        }
        else {
            return createAuto(
                Pipeline.LEFT.activate(),
                new AcSetNumSen(sensors.yaw, -90),
                // Descend and deploy
                new AcSeq.Parallel(level2 ? descendFromRightLevel2() : new AcDoNothing(new ChTrue())),
                new AcSeq.Parallel(level2 ? deployElevatorShort : deployElevator),
                // Score 1st hatch [Common to all r_CH6 autos]
                new AcStraight(standardSettings, -5, -82, sensors.averageEncoder, true),
                new AcTurn(standardSettings, 0),
                new AcSeq.Sequential(autoTrackToTarget),
                new AcSeq.Parallel(placeHatch),
                new AcTankDrive(new ChTime(0.5), -0.5, -0.5),
                // Collect 2nd hatch [Old path]
                new AcTurn(standardSettings, -110),
                doStraightDistance(2.6, -118, 0.8),
                doStraightDistance(0.5, -90, 0.5),
                new AcSeq.Sequential(goToTarget()),
                new AcSeq.Parallel(intakeHatchSolClose),
                new AcWait(0.3),
                new AcTankDrive(new ChTime(0.2), -0.2, -0.2),
                // Score 2nd hatch [Same as above]
                new AcStraight(standardSettings, -6.8, -98, sensors.averageEncoder, true),
                new AcTurn(standardSettings, 0),
                Pipeline.CENTRE.activate(),
                new AcSeq.Sequential(autoTrackToTarget),
                new AcSeq.Parallel(placeHatch),
                new AcTankDrive(new ChTime(0.5), -0.3, -0.3)
            );
        }
    }

    private static CommandBase r_CH6_CH5(boolean level2, boolean newPath) {
        if(newPath) {
            return createAuto(
                Pipeline.LEFT.activate(),
                new AcSetNumSen(sensors.yaw, -90),
                // Descend and deploy
                new AcSeq.Parallel(level2 ? descendFromRightLevel2() : new AcDoNothing(new ChTrue())),
                new AcSeq.Parallel(level2 ? deployElevatorShort : deployElevator),
                // Score 1st hatch [Common to all r_CH6 autos]
                new AcStraight(standardSettings, -5, -82, sensors.averageEncoder, true),
                new AcTurn(standardSettings, 0),
                new AcSeq.Sequential(autoTrackToTarget),
                new AcSeq.Parallel(placeHatch),
                new AcTankDrive(new ChTime(0.5), -0.5, -0.5),
                // Collect 2nd hatch [New path]
                new AcTurn(standardSettings, -90),
                new AcDriveArc(0.9, -125, sensors.averageEncoder, 1.4),
                new AcSeq.Parallel(limelightOn),
                new AcDriveArc(new ChGettableBoolean(vision::isValidTarget, true), 0.9, -95, sensors.averageEncoder, 2),
                new AcDriveDirection(new ChGettableBoolean(vision::isValidTarget, true), 0.6, -90),
                new AcSeq.Sequential(goToTarget()),
                new AcSeq.Parallel(intakeHatchSolClose),
                new AcWait(0.3),
                new AcTankDrive(new ChTime(0.2), -0.2, -0.2),
                // Score 2nd hatch [Same as below]
                new AcLambda(new ChTrue(), a -> sensors.yaw.set(sensors.yaw.get() + 180)),
                doStraightDistance(0.5, 90, -0.6),
                new AcDriveArc(-0.8, 0, sensors.averageEncoder, 3.5),
                new AcTurn(standardSettings, -90),
                Pipeline.RIGHT.activate(),
                new AcSeq.Sequential(autoTrackToTarget),
                new AcSeq.Parallel(placeHatch),
                new AcTankDrive(new ChTime(0.5), -0.3, -0.3)
            );
        }
        else {
            return createAuto(
                Pipeline.LEFT.activate(),
                new AcSetNumSen(sensors.yaw, -90),
                // Descend and deploy
                new AcSeq.Parallel(level2 ? descendFromRightLevel2() : new AcDoNothing(new ChTrue())),
                new AcSeq.Parallel(level2 ? deployElevatorShort : deployElevator),
                // Score 1st hatch [Common to all r_CH6 autos]
                new AcStraight(standardSettings, -5, -82, sensors.averageEncoder, true),
                new AcTurn(standardSettings, 0),
                new AcSeq.Sequential(autoTrackToTarget),
                new AcSeq.Parallel(placeHatch),
                new AcTankDrive(new ChTime(0.5), -0.5, -0.5),
                // Collect 2nd hatch [Old path]
                new AcTurn(standardSettings, -110),
                doStraightDistance(2.6, -118, 0.8),
                doStraightDistance(0.5, -90, 0.5),
                new AcSeq.Sequential(goToTarget()),
                new AcSeq.Parallel(intakeHatchSolClose),
                new AcWait(0.3),
                new AcTankDrive(new ChTime(0.2), -0.2, -0.2),
                // Score 2nd hatch [Same as above]
                new AcLambda(new ChTrue(), a -> sensors.yaw.set(sensors.yaw.get() + 180)),
                doStraightDistance(0.5, 90, -0.6),
                new AcDriveArc(-0.8, 0, sensors.averageEncoder, 3.5),
                new AcTurn(standardSettings, -90),
                Pipeline.RIGHT.activate(),
                new AcSeq.Sequential(autoTrackToTarget),
                new AcSeq.Parallel(placeHatch),
                new AcTankDrive(new ChTime(0.5), -0.3, -0.3)
            );
        }
    }

    private static CommandBase r_CH6_RH1(boolean level2, boolean newPath) {
        if(newPath) {
            return createAuto(
                Pipeline.LEFT.activate(),
                new AcSetNumSen(sensors.yaw, -90),
                // Descend and deploy
                new AcSeq.Parallel(level2 ? descendFromRightLevel2() : new AcDoNothing(new ChTrue())),
                new AcSeq.Parallel(level2 ? deployElevatorShort : deployElevator),
                // Score 1st hatch [Common to all r_CH6 autos]
                new AcStraight(standardSettings, -5, -80, sensors.averageEncoder, true),
                new AcTurn(standardSettings, 0),
                new AcSeq.Sequential(autoTrackToTarget),
                new AcSeq.Parallel(placeHatch),
                new AcTankDrive(new ChTime(0.5), -0.5, -0.5),
                // Collect 2nd hatch [New path]
                new AcTurn(standardSettings, -90),
                new AcDriveArc(0.9, -125, sensors.averageEncoder, 1.4),
                new AcSeq.Parallel(limelightOn),
                new AcDriveArc(new ChGettableBoolean(vision::isValidTarget, true), 0.9, -95, sensors.averageEncoder, 2),
                new AcDriveDirection(new ChGettableBoolean(vision::isValidTarget, true), 0.6, -90),
                new AcSeq.Sequential(goToTarget()),
                new AcSeq.Parallel(intakeHatchSolClose),
                new AcWait(0.3),
                new AcTankDrive(new ChTime(0.2), -0.2, -0.2),
                // Score 2nd hatch [Same as below]
                new AcLambda(new ChTrue(), a -> sensors.yaw.set(sensors.yaw.get() + 180)),
                doStraightDistance(2.5, 80, -0.8),
                new AcTurn(standardSettings, -40),
                new AcSeq.Sequential(autoTrackToTarget),
                new AcSeq.Parallel(placeHatch),
                new AcTankDrive(new ChTime(0.5), -0.3, -0.3)
            );
        }
        else {
            return createAuto(
                Pipeline.LEFT.activate(),
                new AcSetNumSen(sensors.yaw, -90),
                // Descend and deploy
                new AcSeq.Parallel(level2 ? descendFromRightLevel2() : new AcDoNothing(new ChTrue())),
                new AcSeq.Parallel(level2 ? deployElevatorShort : deployElevator),
                // Score 1st hatch [Common to all r_CH6 autos]
                new AcStraight(standardSettings, -5, -80, sensors.averageEncoder, true),
                new AcTurn(standardSettings, 0),
                new AcSeq.Sequential(autoTrackToTarget),
                new AcSeq.Parallel(placeHatch),
                new AcTankDrive(new ChTime(0.5), -0.5, -0.5),
                // Collect 2nd hatch [Old path]
                new AcTurn(standardSettings, -110),
                doStraightDistance(2.6, -118, 0.8),
                doStraightDistance(0.5, -90, 0.5),
                new AcSeq.Sequential(goToTarget()),
                new AcSeq.Parallel(intakeHatchSolClose),
                new AcWait(0.3),
                new AcTankDrive(new ChTime(0.2), -0.2, -0.2),
                // Score 2nd hatch [Same as above]
                new AcLambda(new ChTrue(), a -> sensors.yaw.set(sensors.yaw.get() + 180)),
                doStraightDistance(2.5, 80, -0.8),
                new AcTurn(standardSettings, -40),
                new AcSeq.Sequential(autoTrackToTarget),
                new AcSeq.Parallel(placeHatch),
                new AcTankDrive(new ChTime(0.5), -0.3, -0.3)
            );
        }
    }

    private static CommandBase l_CH3_CH2(boolean level2) {
        return createAuto(
            Pipeline.RIGHT.activate(),
            new AcSetNumSen(sensors.yaw, 90),
            // Descend and deploy
            new AcSeq.Parallel(level2 ? descendFromLeftLevel2() : new AcDoNothing(new ChTrue())),
            new AcSeq.Parallel(level2 ? deployElevatorShort : deployElevator),
            // Score 1st hatch [dup 14]
            new AcStraight(standardSettings, -5, 82, sensors.averageEncoder, true),
            new AcTurn(standardSettings, 20),
            new AcSeq.Sequential(autoTrackToTarget),
            new AcSeq.Parallel(placeHatch),
            new AcTankDrive(new ChTime(0.5), -0.5, -0.5),
            // Collect 2nd hatch
            new AcTurn(standardSettings, 90),
            new AcDriveArc(0.9, 125, sensors.averageEncoder, 1.4),
            new AcSeq.Parallel(limelightOn),
            new AcDriveArc(new ChGettableBoolean(vision::isValidTarget, true), 0.9, 95, sensors.averageEncoder, 2),
            new AcDriveDirection(new ChGettableBoolean(vision::isValidTarget, true), 0.6, 90),
            new AcSeq.Sequential(goToTarget()),
            new AcSeq.Parallel(intakeHatchSolClose),
            new AcWait(0.3),
            new AcTankDrive(new ChTime(0.2), -0.2, -0.2),
            // Score 2nd hatch
            new AcStraight(standardSettings, -6.4, 105, sensors.averageEncoder, true),
            new AcTurn(standardSettings, 20),
            Pipeline.CENTRE.activate(),
            new AcSeq.Sequential(autoTrackToTarget),
            new AcSeq.Parallel(placeHatch),
            new AcTankDrive(new ChTime(0.5), -0.3, -0.3)
        );
    }

    private static Action doStraightDistance(double distance, double angle, double speed) {
        return new AcDriveDirection(new ChNumSen(distance * standardSettings.getEncoderTicksPerMetre(), sensors.averageEncoder), speed, angle);
    }

    private static Action descendFromRightLevel2() {
        return doStraightDistance(0.5, -90, -0.7);
    }

    private static Action descendFromLeftLevel2() {
        return doStraightDistance(0.5, 90, -0.7);
    }
}