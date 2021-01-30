package frc.lib.models;

import frc.lib.geometry.Pose2d;
import frc.lib.geometry.Rotation2d;
import frc.lib.trajectory.*;
import frc.lib.trajectory.constraint.CentripetalAccelerationConstraint;
import frc.lib.trajectory.constraint.TrajectoryConstraint;

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

    public Trajectory generateTrajectory(
            boolean reversed,
            final List<Pose2d> waypoints,
            final List<TrajectoryConstraint> constraints,
            double max_vel,  // m/s
            double max_accel,  // m/s^2
            double max_voltage) {
        return DMP.generateTrajectory(reversed, waypoints, constraints, 0.0, 0.0, max_vel, max_accel, max_voltage);
    }

    public Trajectory getTwoMeters() {
        List<Pose2d> Points = new ArrayList<>();
        Points.add(new Pose2d(0, 0, Rotation2d.identity()));
        Points.add(new Pose2d(6, 0, Rotation2d.identity()));
        return generateTrajectory(false, Points, Arrays.asList(new CentripetalAccelerationConstraint(60)), 2.0, 2.0, 10.0);
    }

    public Trajectory getThreeByThree() {
        List<Pose2d> Points = new ArrayList<>();
        Points.add(new Pose2d(0, 0, Rotation2d.identity()));
        Points.add(new Pose2d(3, -3, Rotation2d.fromDegrees(270)));
        return generateTrajectory(false, Points, Arrays.asList(new CentripetalAccelerationConstraint(60)), .5, 2.0, 10.0);
    }

    public Trajectory getSnkCurve() {
        List<Pose2d> Points = new ArrayList<>();
        Points.add(new Pose2d(0, 0, Rotation2d.identity()));
        Points.add(new Pose2d(3, -3, Rotation2d.fromDegrees(270)));
        Points.add(new Pose2d(6, -3, Rotation2d.fromDegrees(90)));
        return generateTrajectory(false, Points, Arrays.asList(new CentripetalAccelerationConstraint(60)), .5, 2.0, 10.0);
    }

    public Trajectory getLoop() {
        List<Pose2d> Points = new ArrayList<>();
        Points.add(new Pose2d(0, 0, Rotation2d.identity()));
        Points.add(new Pose2d(3, -3, Rotation2d.fromDegrees(270)));
        Points.add(new Pose2d(6, -3, Rotation2d.fromDegrees(90)));
        Points.add(new Pose2d(0, 0, Rotation2d.fromDegrees(180)));
        return generateTrajectory(false, Points, Arrays.asList(new CentripetalAccelerationConstraint(60)), .5, 2.0, 10.0);
    }
}
