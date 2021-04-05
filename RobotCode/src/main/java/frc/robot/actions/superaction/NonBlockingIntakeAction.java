package frc.robot.actions.superaction;

import frc.lib.statemachine.Action;
import frc.lib.statemachine.StateMachine;
import frc.robot.subsystems.Superstructure;
//TODO implement this to work
public class NonBlockingIntakeAction extends Action {
    private Superstructure superstructure;

    public NonBlockingIntakeAction() {
        superstructure = Superstructure.getInstance();
    }

    @Override public void onStart() {
        superstructure.setIntaking(true);
    }

    @Override public void onLoop() {
        superstructure.setIntaking(true);}

    @Override public boolean isFinished() {
        return false;
    }

    @Override public void onStop() {
        superstructure.setIntaking(false);
    }
}
