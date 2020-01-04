package redbacks.arachne.ext.motion.pid;

import edu.wpi.first.wpilibj.PIDController;
import redbacks.arachne.ext.motion.MotionExtender;
import redbacks.arachne.lib.actions.Action;
import redbacks.arachne.lib.checks.ChTrue;

/**
 * @author Mitchell Barker, Darin Huang, Lucas Parker, Gabriela Ribeiro, Jon Reilly, Ben Schwarz, Sean Zammit
 */
public class AcKillPID extends Action {

    public AcKillPID() {
        super(new ChTrue());
    }

    public void onFinish() {
        for(PIDController controller : MotionExtender.getInstance().activePIDs) controller.disable();
    }
}