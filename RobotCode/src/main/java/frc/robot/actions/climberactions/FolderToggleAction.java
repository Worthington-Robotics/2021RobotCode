package frc.robot.actions.climberactions;

import frc.lib.statemachine.Action;
import frc.robot.subsystems.Climber;


public class FolderToggleAction extends Action {
    @Override
    public void onStart() {
        if (Climber.getInstance().readyToUnfold) {
            Climber.getInstance().setUnfold(true);
        }
        //System.out.println("Climb is unfolding.");
    }

    @Override
    public void onLoop() {

    }

    @Override
    public boolean isFinished() {
            return !Climber.getInstance().readyToFold;

    }

    @Override
    public void onStop() {
        if (Climber.getInstance().readyToFold) {
            Climber.getInstance().setUnfold(false);
        }
    }
}