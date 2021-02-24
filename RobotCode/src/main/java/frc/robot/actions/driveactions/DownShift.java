package frc.robot.actions.driveactions;

import frc.lib.statemachine.Action;
import frc.robot.subsystems.Drive;

public class DownShift extends Action {

    @Override
    public void onStart() {
        Drive.getInstance().manualShifterOverride(true);
        Drive.getInstance().setTrans(false);
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
        Drive.getInstance().manualShifterOverride(false);
    }
}
