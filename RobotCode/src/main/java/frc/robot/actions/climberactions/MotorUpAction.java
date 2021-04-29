package frc.robot.actions.climberactions;

import frc.lib.statemachine.Action;
import frc.robot.subsystems.Climber;


public class MotorUpAction extends Action {
    
    @Override
    public void onStart() {
        Climber.getInstance().setMotorPower(1);
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
        Climber.getInstance().setMotorPower(0);
    }
}