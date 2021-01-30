package frc.lib.models;

import frc.lib.geometry.Pose2d;
import frc.lib.geometry.Pose2dWithCurvature;
import frc.lib.geometry.Rotation2d;
import frc.lib.trajectory.Trajectory;
import frc.lib.trajectory.timing.CentripetalAccelerationConstraint;
import frc.lib.trajectory.timing.TimedState;
import frc.lib.trajectory.timing.TimingConstraint;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;

public class DriveTrajectoryGenerator {
    private static final DriveTrajectoryGenerator m_instance = new DriveTrajectoryGenerator();
    private final DriveMotionPlanner DMP;
    public final Pose2d HabStart;
    private DriveTrajectoryGenerator() {
        DMP           = new DriveMotionPlanner();
        HabStart      = new Pose2d(0, 0, Rotation2d.fromDegrees(0));
    }

    public static DriveTrajectoryGenerator getInstance() {
        return m_instance;
    }

    public Trajectory<TimedState<Pose2dWithCurvature>> generateTrajectory(
            boolean reversed,
            final List<Pose2d> waypoints,
            final List<TimingConstraint<Pose2dWithCurvature>> constraints,
            double max_vel,  // m/s
            double max_accel,  // m/s^2
            double max_voltage) {
        return DMP.generateTrajectory(reversed, waypoints, constraints, 0.0, 0.0, max_vel, max_accel, max_voltage);
    }

    public Trajectory<TimedState<Pose2dWithCurvature>> getTwoMeters() {
        List<Pose2d> Points = new ArrayList<>();
        Points.add(new Pose2d(0, 0, Rotation2d.identity()));
        Points.add(new Pose2d(6, 0, Rotation2d.identity()));
        return generateTrajectory(false, Points, Arrays.asList(new CentripetalAccelerationConstraint(60)), 2.0, 2.0, 10.0);
    }

    public Trajectory<TimedState<Pose2dWithCurvature>> getThreeByThree() {
        List<Pose2d> Points = new ArrayList<>();
        Points.add(new Pose2d(0, 0, Rotation2d.identity()));
        Points.add(new Pose2d(3, -3, Rotation2d.fromDegrees(270)));
        return generateTrajectory(false, Points, Arrays.asList(new CentripetalAccelerationConstraint(60)), .5, 2.0, 10.0);
    }

    public Trajectory<TimedState<Pose2dWithCurvature>> getSnkCurve() {
        List<Pose2d> Points = new ArrayList<>();
        Points.add(new Pose2d(0, 0, Rotation2d.identity()));
        Points.add(new Pose2d(3, -3, Rotation2d.fromDegrees(270)));
        Points.add(new Pose2d(6, -3, Rotation2d.fromDegrees(90)));
        return generateTrajectory(false, Points, Arrays.asList(new CentripetalAccelerationConstraint(60)), .5, 2.0, 10.0);
    }

    public Trajectory<TimedState<Pose2dWithCurvature>> getLoop() {
        List<Pose2d> Points = new ArrayList<>();
        Points.add(new Pose2d(0, 0, Rotation2d.identity()));
        Points.add(new Pose2d(3, -3, Rotation2d.fromDegrees(270)));
        Points.add(new Pose2d(6, -3, Rotation2d.fromDegrees(90)));
        Points.add(new Pose2d(0, 0, Rotation2d.fromDegrees(180)));
        return generateTrajectory(false, Points, Arrays.asList(new CentripetalAccelerationConstraint(60)), .5, 2.0, 10.0);
    }
}
