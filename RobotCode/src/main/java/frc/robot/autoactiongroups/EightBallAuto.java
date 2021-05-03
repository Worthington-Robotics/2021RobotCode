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
import frc.robot.actions.shooteraction.TurretPIDControl;
import frc.robot.actions.superaction.IntakeAction;
import frc.robot.actions.superaction.ShootAllAction;
import frc.robot.actions.superaction.ShootBallAction;
import frc.robot.actions.waitactions.LineCrossWait;

public class EightBallAuto extends StateMachineDescriptor {
    public EightBallAuto() {
        // Shoot 3 while backing and tracking
        addParallel(new Action[] {new TurretPIDControl()}, 750);
        addParallel(new Action[] {new DriveTra(DriveTrajectoryGenerator.getInstance().getBallI()), new LineCrossWait(-1.1, false, true), new OffsetIncrease(), new OffsetIncrease()}, 3000);
        addParallel(new Action[] {new TurretPIDControl()}, 500);
        addParallel(new Action[] {new ShootBallAction(), new TurretPIDControl(), new IntakeAction()}, 1000);
        addParallel(new Action[] {new ShootAllAction(), new TurretPIDControl(), new IntakeAction()}, 3000);
        addParallel(new Action[] {new TurretPIDControl(), new IntakeAction(), new OffsetDecrease()}, 2500);
        addParallel(new Action[] {getTraj(Arrays.asList(getPose(-4.4, 0, 0), getPose(-.5, .5, 10)), false), new TurretPIDControl(), new IntakeAction()}, 3000); // the angle should be 3.7, 30 but isnt for now
        addParallel(new Action[] {new TurretPIDControl()}, 1000);
        addParallel(new Action[] {new TurretPIDControl(), new ShootAllAction()}, 3000);
    }

    private Pose2d getPose(double x, double y, double deg) {
        return new Pose2d(x, y, Rotation2d.fromDegrees(deg));
    }

    private Action getTraj(List<Pose2d> waypoints, boolean reversed) {
        return new DriveTra(DriveTrajectoryGenerator.getInstance().generateTrajectory(reversed, waypoints, null, 1.4, 4, 10));
    }
}