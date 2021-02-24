package frc.robot.actions.driveactions;

import frc.lib.statemachine.Action;
import frc.lib.util.DriveSignal;
import frc.robot.subsystems.Drive;

public class TurnLockout extends Action {
    boolean ccw;
    double radius = .7;

    public TurnLockout(boolean ccw)
    {
        this.ccw = ccw;
    }

    public TurnLockout(boolean ccw, double radius)
    {
        this.ccw = ccw;
        this.radius = radius;
    }

    @Override
    public void onStart() {
        Drive.getInstance().setTurnLockout(ccw, radius);
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
        Drive.getInstance().setOpenLoop(DriveSignal.NEUTRAL);
    }
}
