package frc.robot.actions.shooteraction;

import frc.lib.statemachine.Action;
import frc.robot.subsystems.Shooter;

public class FaceLeft45Deg extends Action {

    @Override
    public void onStart() {
        Shooter.getInstance().setTurret45Left();

    }

    @Override
    public void onLoop() {
        // TODO Auto-generated method stub

    }

    @Override
    public boolean isFinished() {
        // TODO Auto-generated method stub
        return false;
    }

    @Override
    public void onStop() {
        Shooter.getInstance().setTurretDemand(0);

    }

}