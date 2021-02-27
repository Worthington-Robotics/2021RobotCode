package frc.robot.autoactiongroups;

import java.util.Arrays;
import java.util.List;

import frc.lib.geometry.Pose2d;
import frc.lib.geometry.Rotation2d;
import frc.lib.models.DriveTrajectoryGenerator;
import frc.lib.statemachine.Action;
import frc.lib.statemachine.StateMachineDescriptor;
import frc.lib.trajectory.Trajectory;
import frc.robot.actions.driveactions.DriveTra;
import frc.robot.actions.superaction.ShootAllAction;
import frc.robot.actions.superaction.ShootBallAction;

public class EightBallAuto extends StateMachineDescriptor {
    public EightBallAuto() {
        // Shoot 3
        addSequential(new ShootAllAction(), 20L);
        // Back up and get 3
        addSequential(getTraj(Arrays.asList(getPose(0,0,0), getPose(-196,0,0)), true), 20L);
        // Shoot 3
        addSequential(new ShootAllAction(), 20L);
        // Move towards middle of field
        addSequential(getTraj(Arrays.asList(getPose(-196,0,0), getPose(-68,156,30)), true), 20L);
        // Go back and get 3 more
        addSequential(getTraj(Arrays.asList(getPose(-68,156,30), getPose(-157,94,30)), true), 20L);
        // Go back forward
        addSequential(getTraj(Arrays.asList(getPose(-157, 94, 30), getPose(-56, 168, 30)), true), 20L);
        // Go back to get other 2
        addSequential(getTraj(Arrays.asList(getPose(-56, 168, 30), getPose(-151, 130, 30)), true), 20L);
        // Go back forward
        addSequential(getTraj(Arrays.asList(getPose(-151, 130, 30), getPose(-56, 168, 30)), true), 20L);
        // Shoot 5
        addSequential(new ShootAllAction(), 20L);
    }

    private Pose2d getPose(int x, int y, int deg) {
        return new Pose2d(x, y, Rotation2d.fromDegrees(deg));
    }

    private Action getTraj(List<Pose2d> waypoints, boolean reversed) {
        return new DriveTra(DriveTrajectoryGenerator.getInstance().generateTrajectory(reversed, waypoints, null, 1.75, 5, 10));
    }
}