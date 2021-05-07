package frc.robot.autoactiongroups;

import java.util.Arrays;
import java.util.List;

import frc.lib.geometry.Pose2d;
import frc.lib.geometry.Rotation2d;
import frc.lib.models.DriveTrajectoryGenerator;
import frc.lib.statemachine.Action;
import frc.lib.statemachine.StateMachineDescriptor;
import frc.robot.actions.driveactions.DriveTra;
import frc.robot.actions.shooteraction.OffsetDecrease;
import frc.robot.actions.shooteraction.OffsetIncrease;
import frc.robot.actions.shooteraction.ToggleTurretPIDControl;
import frc.robot.actions.shooteraction.TurretPIDControl;
import frc.robot.actions.superaction.IntakeAction;
import frc.robot.actions.superaction.ShootAllAction;
import frc.robot.actions.superaction.ShootBallAction;
import frc.robot.actions.waitactions.LineCrossWait;
import frc.robot.actions.waitactions.TimedWait;

public class ThreeBall extends StateMachineDescriptor {
    public ThreeBall() {
        // Shoot 3 while backing and tracking
        addParallel(new Action[] { new ToggleTurretPIDControl() , new IntakeAction() }, 50);
        addParallel(new Action[] { new DriveTra(DriveTrajectoryGenerator.getInstance().GiveEmAnInch()), new TimedWait(),
                new OffsetIncrease()}, 3000);
        addParallel(new Action[] { new ShootAllAction()}, 1500);
        addParallel(new Action[] { new ToggleTurretPIDControl(), new OffsetDecrease() }, 50);
    }
}