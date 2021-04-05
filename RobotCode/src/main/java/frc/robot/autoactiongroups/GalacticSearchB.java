package frc.robot.autoactiongroups;

import frc.lib.statemachine.Action;
import frc.lib.statemachine.StateMachineDescriptor;
import frc.robot.actions.driveactions.*;
import frc.robot.actions.shooteraction.SetManualFlywheel;
import frc.robot.actions.superaction.IntakeAction;
import frc.robot.actions.waitactions.LineCrossWait;
import frc.robot.actions.waitactions.TimedWait;
import frc.lib.models.DriveTrajectoryGenerator;

public class GalacticSearchB extends StateMachineDescriptor {
    public GalacticSearchB() {
        addParallel(new Action[] {new TimedWait(), new SetManualFlywheel()}, 750);
        addParallel(new Action[] {new DriveTra(DriveTrajectoryGenerator.getInstance().getGB1(), false)}, 500);
        addParallel(new Action[] {new LineCrossWait(-3.575, false, true)}, 9000);
        addParallel(new Action[] {new IntakeAction()}, 3500);
        addParallel(new Action[] {new LineCrossWait(.5, true, true)}, 9000);
        addParallel(new Action[] {new IntakeAction()}, 2500);
        addParallel(new Action[] {new LineCrossWait(-5.95, false, true)}, 9000);
        addParallel(new Action[] {new IntakeAction()}, 3000);
        addParallel(new Action[] {new DriveTra(DriveTrajectoryGenerator.getInstance().getGB2(), false)}, 500);
    }
}