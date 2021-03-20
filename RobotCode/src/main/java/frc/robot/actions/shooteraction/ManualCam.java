package frc.robot.actions.shooteraction;

import frc.lib.statemachine.Action;
import frc.robot.Constants;
import frc.robot.subsystems.Shooter;

public class ManualCam extends Action {
    public int angle = 0;
    public ManualCam(int angle)
    {
        this.angle = angle;
    }

    /**
     * code to run on action start
     * 
     */
    @Override
    public void onStart() {
        Shooter.getInstance().setCamDemandOpen(angle);
    }

    /**
     * code to run while action loops
     * <p>
     * approx every 20 miliseconds
     */
    @Override
    public void onLoop() {
    }

    /**
     * method that tells the state machine the action is finished earlier than the
     * scheduler
     *
     * @return true when action is ready to self terminate
     */
    @Override
    public boolean isFinished() {
        return false;
    }

    /**
     * code to run when the action has ben called by the state machine to stop
     */
    @Override
    public void onStop() {
        Shooter.getInstance().setCamDemandOpen(Constants.CAM_ANGLE_HIGH);
    }
}
