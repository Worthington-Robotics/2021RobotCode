package frc.robot.autoactiongroups;

import frc.lib.statemachine.Action;
import frc.lib.statemachine.StateMachineDescriptor;
import frc.robot.actions.driveactions.*;
import frc.robot.actions.shooteraction.SetManualFlywheel;
import frc.robot.actions.superaction.IntakeAction;
import frc.robot.actions.waitactions.LineCrossWait;
import frc.robot.actions.waitactions.TimedWait;
import frc.lib.models.DriveTrajectoryGenerator;

public class GalacticSearchD extends StateMachineDescriptor {
    public GalacticSearchD() {
        addParallel(new Action[] {new TimedWait(), new SetManualFlywheel()}, 50);
        addParallel(new Action[] {new DriveTra(DriveTrajectoryGenerator.getInstance().GSE, false)}, 50);
        addParallel(new Action[] {new LineCrossWait(-3.4, false, true)}, 10000);
        addParallel(new Action[] {new IntakeAction()}, 2000);
        addParallel(new Action[] {new LineCrossWait(-4.9, false, true)}, 10000);
        addParallel(new Action[] {new IntakeAction()}, 1500);
        addParallel(new Action[] {new LineCrossWait(-6.5, false, true)}, 9000);
        addParallel(new Action[] {new IntakeAction()}, 1500);
    }
}