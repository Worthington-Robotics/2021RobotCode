package frc.robot.actions.superaction;

import frc.lib.statemachine.Action;
import frc.robot.Constants;
import frc.robot.subsystems.Superstructure;

public class IntakeAction extends Action {
    private Superstructure superstructure;

    public IntakeAction() {
        superstructure = Superstructure.getInstance();
    }

    @Override public void onStart() {
        superstructure.setIntakeDemand(Constants.HIGH_BELT_DEMAND);
    }

    @Override public void onLoop() {

    }

    @Override public boolean isFinished() {
        return Constants.DISTANCE_STOP_MM >= superstructure.getIntakeDistance();
    }

    @Override public void onStop() {
        superstructure.setDeliveryBeltDemand(Constants.STOP_BELT_DEMAND);
    }
}