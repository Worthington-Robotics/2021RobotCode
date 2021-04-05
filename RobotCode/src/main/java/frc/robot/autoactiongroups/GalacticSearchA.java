package frc.robot.autoactiongroups;

import frc.lib.statemachine.Action;
import frc.lib.statemachine.StateMachineDescriptor;
import frc.robot.actions.driveactions.*;
import frc.robot.actions.shooteraction.SetManualFlywheel;
import frc.robot.actions.superaction.IntakeAction;
import frc.robot.actions.waitactions.LineCrossWait;
import frc.robot.actions.waitactions.TimedWait;
import frc.lib.models.DriveTrajectoryGenerator;

public class GalacticSearchA extends StateMachineDescriptor {
    public GalacticSearchA() {
        addParallel(new Action[] {new TimedWait(), new SetManualFlywheel()}, 750);
        addParallel(new Action[] {new DriveTra(DriveTrajectoryGenerator.getInstance().getGA1(), false)}, 500);
        addParallel(new Action[] {new LineCrossWait(-.55, false, true)}, 6000);
        addParallel(new Action[] {new IntakeAction()}, 2000);
        addParallel(new Action[] {new LineCrossWait(-2.3, false, true)}, 6000);
        addParallel(new Action[] {new IntakeAction()}, 2000);
        addParallel(new Action[] {new LineCrossWait(-.6, true, true)}, 9000);
        addParallel(new Action[] {new IntakeAction()}, 2000);
        addParallel(new Action[] {new DriveTra(DriveTrajectoryGenerator.getInstance().getGA2(), false)}, 500);
    }
}