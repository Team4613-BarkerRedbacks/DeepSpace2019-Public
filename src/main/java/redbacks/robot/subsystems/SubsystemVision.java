package redbacks.robot.subsystems;

import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.networktables.NetworkTableEntry;

import edu.wpi.first.networktables.NetworkTable;
import redbacks.arachne.core.SubsystemBase;
import redbacks.arachne.lib.actions.AcLambda;
import redbacks.arachne.lib.actions.Action;
import redbacks.arachne.lib.checks.ChTrue;
import redbacks.robot.Robot;

import java.lang.Math;

/**
 * Handles limelight functionality.
 * @author Mitchell Barker, Darin Huang, Lucas Parker, Gabriela Ribeiro, Jon Reilly, Ben Schwarz, Sean Zammit
 */
public class SubsystemVision extends SubsystemBase {

    public final double limelightHeight = 270; // Millimetres, height difference from limelight to target
    public final double limelightHeightCargo = 60;
    public final double limelightOffset = 160; // Millimetres, distance from limelight to centre line of robot
    
    // Decreasing magnitude shifts RIGHT
    // e.g. -25 to -20 shifts right 
    public final double limelightAngle = Math.toRadians(-20); // Radians, angle of the limelight from the horizontal
    
    public final int lightsOn = 3, lightsOff = 1;

    public static NetworkTable limelight = NetworkTableInstance.getDefault().getTable("limelight");
    public NetworkTableEntry horizontalOffset = limelight.getEntry("tx");
    public NetworkTableEntry verticalOffset = limelight.getEntry("ty");
    public NetworkTableEntry discoveredTarget = limelight.getEntry("tv");
    public NetworkTableEntry coverageTargetArea = limelight.getEntry("ta");
    public NetworkTableEntry rotation = limelight.getEntry("ts");
    public NetworkTableEntry latency = limelight.getEntry("tl");
    public NetworkTableEntry shortSideBoundBoxPx = limelight.getEntry("tshort");
    public NetworkTableEntry longSideBoundBoxPx = limelight.getEntry("tlong");
    public NetworkTableEntry horizontalBoundBoxPx = limelight.getEntry("thor");
    public NetworkTableEntry verticalBoundBoxPx = limelight.getEntry("tvert");
    public NetworkTableEntry rawScreenSpaceX = limelight.getEntry("tx0");
    public NetworkTableEntry rawScreenSpaceY = limelight.getEntry("ty0");

    public NetworkTableEntry ledMode = limelight.getEntry("ledMode");
    public static NetworkTableEntry pipeline = limelight.getEntry("pipeline");

    public SubsystemVision() {
        super();

        ledMode.setNumber(lightsOff);
    }

    public double getStraightLineDistance() {
        double angle = limelightAngle + Math.toRadians(verticalOffset.getDouble(0));
        double height = Robot.intake.shouldExtend() ? limelightHeightCargo : limelightHeight;

        return Math.abs(height / Math.tan(angle));
    }

    /**
     * Gets the ideal, or desired angular offset for the current distance to a target in order to be considered aligned.
     */
    public double getIdealOffset() {
        return 90 - Math.toDegrees(Math.atan2(getStraightLineDistance(), limelightOffset));
    }

    public void readVision() {
        SmartDashboard.putNumber("[Limelight] Distance", getStraightLineDistance());
        SmartDashboard.putNumber("[Limelight] Angle", getIdealOffset());
        SmartDashboard.putBoolean("[Limelight] Has target", discoveredTarget.getDouble(0) == 1);
    }

    public Action setLimelightMode(boolean tracking) {
        if(tracking) return new AcLambda(new ChTrue(), a -> ledMode.setNumber(lightsOn));
        else return new AcLambda(new ChTrue(), a -> ledMode.setNumber(lightsOff));
    }

    public boolean isValidTarget() {
        if(horizontalOffset.getDouble(0) == 0 && verticalOffset.getDouble(0) == 0) return false;

        if(Math.abs(horizontalOffset.getDouble(0)) > 20 && getStraightLineDistance() < 800) return false;
        // if(getStraightLineDistance() > 5000 || getStraightLineDistance() < 400) return false;
        if(discoveredTarget.getDouble(0) == 0) return false;

        return true;
    }

    public static enum Pipeline {

        LOW(0),
        LEFT(1),
        RIGHT(2),
        CENTRE(3);

        public final int id;

        private Pipeline(int id) {
            this.id = id;
        }

        public Action activate() {
            return new AcLambda(new ChTrue(), a -> pipeline.setNumber(id));
        }
    }
}
