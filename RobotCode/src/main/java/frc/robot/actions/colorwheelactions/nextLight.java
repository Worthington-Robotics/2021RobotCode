package frc.robot.actions.colorwheelactions;

import frc.lib.statemachine.Action;
import frc.robot.subsystems.Lights;

public class nextLight extends Action {
    /**
     * code to run on action start
     */
    @Override
    public void onStart() {
        Lights.getInstance().nextLight();
    }

    /**
     * code to run while action loops
     * <p>approx every 20 miliseconds
     */
    @Override
    public void onLoop() {
    }

    /**
     * method that tells the state machine the action is finished earlier than the scheduler
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
