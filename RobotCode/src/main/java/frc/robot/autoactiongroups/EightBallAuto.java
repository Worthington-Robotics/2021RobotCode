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

public class EightBallAuto extends StateMachineDescriptor {
    public EightBallAuto() {
        // Shoot 3 while backing and tracking
        addParallel(new Action[] {new ToggleTurretPIDControl()}, 50);
        addParallel(new Action[] {new DriveTra(DriveTrajectoryGenerator.getInstance().B8P), new LineCrossWait(-.9, false, true), new OffsetIncrease()}, 3000);
        addParallel(new Action[] {new IntakeAction()}, 400);
        addParallel(new Action[] {new ShootBallAction(), new IntakeAction(), new OffsetIncrease()}, 1000);
        addParallel(new Action[] {new ShootBallAction(), new IntakeAction()}, 1000);
        addParallel(new Action[] {new ShootBallAction(), new IntakeAction()}, 1000);
        addParallel(new Action[] {new IntakeAction(), new OffsetDecrease(), new OffsetDecrease(), new OffsetDecrease(), new LineCrossWait(-4.2, false, true)}, 4000);
        addParallel(new Action[] {new DriveTra(DriveTrajectoryGenerator.getInstance().B9P)}, 50);
        addParallel(new Action[] {new TurretPIDControl()}, 1000);
        addParallel(new Action[] {new ShootBallAction(), new TurretPIDControl()}, 750);
        addParallel(new Action[] {new ShootBallAction(), new TurretPIDControl()}, 750);
        addParallel(new Action[] {new ShootBallAction(), new TurretPIDControl(), new OffsetIncrease()}, 750);
    }
}