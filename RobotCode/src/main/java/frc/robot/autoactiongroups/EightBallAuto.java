package frc.robot.autoactiongroups;

import java.util.Arrays;
import java.util.List;

import frc.lib.geometry.Pose2d;
import frc.lib.geometry.Rotation2d;
import frc.lib.models.DriveTrajectoryGenerator;
import frc.lib.statemachine.Action;
import frc.lib.statemachine.StateMachineDescriptor;
import frc.robot.actions.driveactions.DriveTra;
import frc.robot.actions.shooteraction.TurretPIDControl;
import frc.robot.actions.superaction.IntakeAction;
import frc.robot.actions.superaction.ShootAllAction;
import frc.robot.actions.superaction.ShootBallAction;

public class EightBallAuto extends StateMachineDescriptor {
    public EightBallAuto() {
        // Shoot 3 while backing and tracking
        addParallel(new Action[] {new TurretPIDControl()}, 1500);
        addParallel(new Action[] {getTraj(Arrays.asList(getPose(0,0,0), getPose(-5.6,0,0)), true), new TurretPIDControl()}, 1500);
        addParallel(new Action[] {new ShootBallAction(), new TurretPIDControl(), new IntakeAction()}, 1000);
        addParallel(new Action[] {new ShootAllAction(), new TurretPIDControl(), new IntakeAction()}, 3000);
        addParallel(new Action[] {new TurretPIDControl(), new IntakeAction()}, 4000);
        addParallel(new Action[] {getTraj(Arrays.asList(getPose(-5.6, 0, 0), getPose(-1.5, .5, 10)), false), new TurretPIDControl(), new IntakeAction()}, 3000); // the angle should be 3.7, 30 but isnt for now
        //addParallel(new Action[] {getTrajH(Arrays.asList(getPose(-5.4, 0, 0), getPose(-3,0,0), getPose(-1.4, 3.7, 30)), false), new TurretPIDControl()}, 7000); // the angle should be 3.7, 30 but isnt for now
        addParallel(new Action[] {new TurretPIDControl()}, 1000);
        addParallel(new Action[] {new TurretPIDControl(), new ShootAllAction()}, 3000);
        // addParallel(new Action[] {getTrajL(Arrays.asList(getPose(-1.4, 3.7, 30), getPose(-3.9, 2.3, 30)), true), new TurretPIDControl(), new ShootAllAction(), new IntakeAction()}, 6000);
        // addParallel(new Action[] {getTraj(Arrays.asList(getPose(-3.9, 2.3, 30), getPose(-1.422, 4.27, 30)), false), new TurretPIDControl()}, 3000);
        // addParallel(new Action[] {getTrajL(Arrays.asList(getPose(-1.422, 4.27, 30), getPose(-3.84, 3.3, 30)), true), new TurretPIDControl(), new ShootAllAction(), new IntakeAction()}, 4000);
        // addSequential(new TrajectoryCompleteWait(), 6000);
        // Go back forward
        // addSequential(getTraj(Arrays.asList(getPose(-3.84, 3.3, 30), getPose(-1.42, 4.27, 30)), true), 20L);
        // addSequential(new TrajectoryCompleteWait(), 6000);
        // Shoot 5
        //addSequential(new ShootAllAction(), 20L);
    }

    private Pose2d getPose(double x, double y, double deg) {
        return new Pose2d(x, y, Rotation2d.fromDegrees(deg));
    }

    private Action getTrajL(List<Pose2d> waypoints, boolean reversed) {
        return new DriveTra(DriveTrajectoryGenerator.getInstance().generateTrajectory(reversed, waypoints, null, .4, 4, 10));
    }

    private Action getTraj(List<Pose2d> waypoints, boolean reversed) {
        return new DriveTra(DriveTrajectoryGenerator.getInstance().generateTrajectory(reversed, waypoints, null, .55, 4, 10));
    }

    private Action getTrajH(List<Pose2d> waypoints, boolean reversed) {
        return new DriveTra(DriveTrajectoryGenerator.getInstance().generateTrajectory(reversed, waypoints, null, 1.25, 4, 10));
    }
}