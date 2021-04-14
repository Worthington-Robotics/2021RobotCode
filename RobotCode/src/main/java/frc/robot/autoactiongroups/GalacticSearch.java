package frc.robot.autoactiongroups;

import frc.lib.statemachine.Action;
import frc.lib.statemachine.StateMachineDescriptor;
import frc.robot.actions.driveactions.*;
import frc.robot.actions.shooteraction.SetManualFlywheel;
import frc.robot.actions.superaction.IntakeAction;
import frc.robot.actions.waitactions.LineCrossWait;
import frc.robot.actions.waitactions.TimedWait;
import frc.lib.models.DriveTrajectoryGenerator;
import frc.robot.subsystems.JetsonAILink;
import frc.lib.util.Util;

import edu.wpi.first.wpilibj.DriverStation;

public class GalacticSearch extends StateMachineDescriptor {
    public GalacticSearch() {
        double xOffset = JetsonAILink.getInstance().xOffset();
        double yOffset = JetsonAILink.getInstance().yOffset();
        DriverStation.reportWarning("Marks Code In Progress, Warning", false);
        if (xOffset != 0.0) {
            if (Math.abs(xOffset) < 2.0) {
                if (!Util.epsilonEquals(Math.abs(yOffset), 1.5, 1)) {
                    addParallel(new Action[] {new TimedWait(), new SetManualFlywheel()}, 50);
                    addParallel(new Action[] {new DriveTra(DriveTrajectoryGenerator.getInstance().GSA, false)}, 50);
                    addParallel(new Action[] {new LineCrossWait(-.55, false, true)}, 6000);
                    addParallel(new Action[] {new IntakeAction()}, 2000);
                    addParallel(new Action[] {new LineCrossWait(-2, false, true)}, 6000);
                    addParallel(new Action[] {new IntakeAction()}, 2000);
                    addParallel(new Action[] {new LineCrossWait(.2, true, true)}, 9000);
                    addParallel(new Action[] {new IntakeAction()}, 3000);
                } else {
                    addParallel(new Action[] {new TimedWait(), new SetManualFlywheel()}, 50);
                    addParallel(new Action[] {new DriveTra(DriveTrajectoryGenerator.getInstance().GSC, false)}, 50);
                    addParallel(new Action[] {new LineCrossWait(-.5, false, true)}, 6000);
                    addParallel(new Action[] {new IntakeAction()}, 1500);
                    addParallel(new Action[] {new LineCrossWait(0, true, false)}, 6000);
                    addParallel(new Action[] {new IntakeAction()}, 1500);
                    addParallel(new Action[] {new LineCrossWait(.2, true, true)}, 9000);
                    addParallel(new Action[] {new IntakeAction()}, 1000);
                }
            } else {
                if (Math.signum(yOffset) >= 0) {
                    addParallel(new Action[] {new TimedWait(), new SetManualFlywheel()}, 500);
                    addParallel(new Action[] {new DriveTra(DriveTrajectoryGenerator.getInstance().GSB, false)}, 500);
                    addParallel(new Action[] {new LineCrossWait(-3.57, false, true)}, 9000);
                    addParallel(new Action[] {new IntakeAction()}, 3000);
                    addParallel(new Action[] {new LineCrossWait(.5, true, true)}, 9000);
                    addParallel(new Action[] {new IntakeAction()}, 2500);
                    addParallel(new Action[] {new LineCrossWait(-5.95, false, true)}, 9000);
                    addParallel(new Action[] {new IntakeAction()}, 2000);
                } else {
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
        } else {
            DriverStation.reportError("Auto Not Found: Cannot detect desiered path", true);
        }
    }
}