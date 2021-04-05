package frc.robot.autoactiongroups;

import frc.lib.statemachine.Action;
import frc.lib.statemachine.StateMachineDescriptor;
import frc.robot.actions.driveactions.*;
import frc.robot.actions.shooteraction.SetManualFlywheel;
import frc.robot.actions.superaction.IntakeAction;
import frc.robot.actions.waitactions.LineCrossWait;
import frc.robot.actions.waitactions.TimedWait;
import frc.lib.models.DriveTrajectoryGenerator;

public class GalacticSearchC extends StateMachineDescriptor {
    public GalacticSearchC() {
        addParallel(new Action[] {new TimedWait(), new SetManualFlywheel()}, 750);
        addParallel(new Action[] {new DriveTra(DriveTrajectoryGenerator.getInstance().getGC1(), false)}, 500);
        addParallel(new Action[] {new LineCrossWait(-.5, false, true)}, 6000);
        addParallel(new Action[] {new IntakeAction()}, 2500);
        addParallel(new Action[] {new LineCrossWait(0, true, false)}, 6000);
        addParallel(new Action[] {new IntakeAction()}, 2500);
        addParallel(new Action[] {new LineCrossWait(.2, true, true)}, 9000);
        addParallel(new Action[] {new IntakeAction()}, 2500);
        addParallel(new Action[] {new DriveTra(DriveTrajectoryGenerator.getInstance().getGC2(), false)}, 500);
    }
}