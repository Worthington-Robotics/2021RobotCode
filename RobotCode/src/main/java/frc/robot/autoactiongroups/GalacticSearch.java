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

        if (xOffset == 0.0) {
            if (Math.abs(xOffset) < 2.0) {
                if (!Util.epsilonEquals(Math.abs(yOffset), 1.5, .3)) {
                    //Path for Path A, Red
                } else {
                    //Path for Path B, Red
                }
            } else {
                if (Util.epsilonEquals(Math.abs(yOffset), 1.5, .3)) {
                    //Path for Path A, Blue
                } else {
                    //Path for Path B, Blue
                }
            }
        } else {
            DriverStation.reportError("No Ball Detected: Cannot Create Trajectory", true);
        }
    }
}