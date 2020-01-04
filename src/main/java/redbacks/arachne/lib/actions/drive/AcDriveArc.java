package redbacks.arachne.lib.actions.drive;

import redbacks.arachne.core.ArachneRobot;
import redbacks.arachne.ext.motion.MotionSettings;
import redbacks.arachne.lib.actions.Action;
import redbacks.arachne.lib.checks.ChMulti;
import redbacks.arachne.lib.checks.Check;
import redbacks.arachne.lib.checks.analog.ChNumSen;
import redbacks.arachne.lib.logic.LogicOperators;
import redbacks.arachne.lib.sensors.NumericSensor;

/**
 * Action to allow robot to drive along arcs.
 * Will likely be abstracted and integrated into Arachne for 2020 season in a similar form to AcDriveDirection.
 * 
 * @author Mitchell Barker, Darin Huang, Lucas Parker, Gabriela Ribeiro, Jon Reilly, Ben Schwarz, Sean Zammit
 */
public class AcDriveArc extends Action {

    double speed, heading, distance;
    NumericSensor encoder;

    double startHeading;
    
	public AcDriveArc(double speed, double heading, NumericSensor encoder, double distance) {
		super(new ChNumSen(distance * MotionSettings.standardSettings.getEncoderTicksPerMetre(), encoder));
		this.speed = speed;
        this.heading = heading;
        this.distance = distance;

        this.encoder = encoder;
    }
    
    public AcDriveArc(Check check, double speed, double heading, NumericSensor encoder, double distance) {
		super(
            new ChMulti(LogicOperators.OR,
                new ChNumSen(distance * MotionSettings.standardSettings.getEncoderTicksPerMetre(), encoder),
                check
            )
        );
		this.speed = speed;
        this.heading = heading;
        this.distance = distance;

        this.encoder = encoder;
	}
	
	public void onStart() {
        ArachneRobot.isIndivDriveControl = false;
        encoder.set(0);

        startHeading = DriveSettings.gyro.get();
	}

	public void onRun() {
		double correction = (DriveSettings.gyro.get() - getCurrentTarget()) * DriveSettings.gyroCorrection;
		
		DriveSettings.drivetrain.tankDrive(speed - correction, speed + correction);
    }
    
    public double getCurrentTarget() {
        double target = Math.abs(encoder.get() / MotionSettings.standardSettings.getEncoderTicksPerMetre() / distance) * (heading - startHeading) + startHeading;
        double minCap = Math.min(heading, startHeading);
        double maxCap = Math.max(heading, startHeading);

        return Math.max(Math.min(target, maxCap), minCap);
    }
}