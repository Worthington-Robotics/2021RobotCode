package frc.robot.actions.superaction;

import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import frc.lib.statemachine.Action;
import frc.robot.Constants;
import frc.robot.subsystems.Superstructure;

public class ArmAction extends Action {
    private Superstructure superstructure;

    public ArmAction() {
        superstructure = Superstructure.getInstance();
    }

    @Override public void onStart() {
        superstructure.setArmExtension(Value.kForward);
    }

    @Override public void onLoop() {}

    @Override public boolean isFinished() {
        return false;
    }

    @Override public void onStop() {
        superstructure.setArmExtension(Value.kReverse);

    }
}