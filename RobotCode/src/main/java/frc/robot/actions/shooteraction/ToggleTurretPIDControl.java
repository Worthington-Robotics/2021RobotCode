package frc.robot.actions.shooteraction;

import frc.lib.statemachine.Action;
import frc.robot.subsystems.Shooter;

public class ToggleTurretPIDControl extends Action {
    public static boolean toggle = false;

    /**
     * code to run on action start
     * 
     */
    @Override
    public void onStart() {
        toggle = !toggle;
        if (toggle) {
            Shooter.getInstance().setTurretLimelightControl(0);
        } else {
            Shooter.getInstance().setTurretDemand(0);
        }
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
        return true;
    }

    /**
     * code to run when the action has ben called by the state machine to stop
     */
    @Override
    public void onStop() {
    }
}
