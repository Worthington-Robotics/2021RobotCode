package frc.robot.actions.driveactions;

import frc.lib.statemachine.Action;
import frc.robot.subsystems.Drive;

public class Shift extends Action {

    @Override
    public void onStart() {
        //System.out.print("Shift Engaged");
        Drive.getInstance().setTrans(true);
        Drive.getInstance().manualShifterOverride(true);
    }

    @Override
    public void onLoop() {
    }

    @Override
    public boolean isFinished() {
        return false;
    }

    @Override
    public void onStop() {
        Drive.getInstance().setTrans(false);
        Drive.getInstance().manualShifterOverride(false);
    }
}
