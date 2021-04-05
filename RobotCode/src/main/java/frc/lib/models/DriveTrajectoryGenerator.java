package frc.lib.models;

import frc.lib.geometry.Pose2d;
import frc.lib.geometry.Rotation2d;
import frc.lib.geometry.Translation2d;
import frc.lib.trajectory.Trajectory;
import frc.lib.trajectory.constraint.CentripetalAccelerationConstraint;
import frc.lib.trajectory.constraint.TrajectoryConstraint;
import frc.lib.trajectory.constraint.VelocityLimitRegionConstraint;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;


public class DriveTrajectoryGenerator {
    private static final DriveTrajectoryGenerator m_instance = new DriveTrajectoryGenerator();
    private final DriveMotionPlanner DMP;
    private DriveTrajectoryGenerator() {
        DMP           = new DriveMotionPlanner();
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
                Trajectory tra;
                if(!reversed)
                {tra = DMP.generateTrajectory(waypoints, constraints, 0.0, 0.0, max_vel, max_accel, max_voltage);}
                else
                {tra = DMP.generateRevTrajectory(waypoints, constraints, 0.0, 0.0, max_vel, max_accel, max_voltage);}
        return tra;
    }

    public Trajectory getTwoMeters() {
        List<Pose2d> Points = new ArrayList<>();
        Points.add(new Pose2d(0, 0, Rotation2d.identity()));
        Points.add(new Pose2d(6, 0, Rotation2d.identity()));
        return generateTrajectory(false, Points, Arrays.asList(new CentripetalAccelerationConstraint(60)),1.75, 2.0, 2.0);
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
        return generateTrajectory(false, Points, Arrays.asList(new CentripetalAccelerationConstraint(10)), .5, 2.0, 10.0);
    }

    public Trajectory getSki(){
        List<Pose2d> Points = new ArrayList<>();
        Points.add(new Pose2d(0, 0, Rotation2d.identity()));
        Points.add(new Pose2d(1.2, 1.07, Rotation2d.fromDegrees(89)));
        //Points.add(new Pose2d(3.5, 2.85, Rotation2d.fromDegrees(0)));
        Points.add(new Pose2d(6.1, 1.2, Rotation2d.fromDegrees(270)));
        Points.add(new Pose2d(7.8, 1.1, Rotation2d.fromDegrees(89)));
        Points.add(new Pose2d(6, 1.2, Rotation2d.fromDegrees(270)));
        //Points.add(new Pose2d(3.3, -.6, Rotation2d.fromDegrees(180)));
        Points.add(new Pose2d(1.17, 1.07, Rotation2d.fromDegrees(115)));
        Points.add(new Pose2d(.5, 2.3, Rotation2d.fromDegrees(125)));
        return generateTrajectory(false, Points, null, 1.75, 4.0, 10.0);
    }

    public Trajectory getBar(){
        List<Pose2d> Points = new ArrayList<>();
        Points.add(new Pose2d(0, 0, Rotation2d.identity()));
        Points.add(new Pose2d(3.7, -1.3, Rotation2d.fromDegrees(270)));
        Points.add(new Pose2d(2.3, -1.3, Rotation2d.fromDegrees(90)));
        Points.add(new Pose2d(5.25, -.7, Rotation2d.fromDegrees(10)));
        Points.add(new Pose2d(5.25, 1.7, Rotation2d.fromDegrees(180)));
        Points.add(new Pose2d(5.25, -.6, Rotation2d.fromDegrees(301)));
        Points.add(new Pose2d(6.9, -2.3, Rotation2d.fromDegrees(0.1)));
        Points.add(new Pose2d(6.9, -.5, Rotation2d.fromDegrees(180)));
        Points.add(new Pose2d(0, 0, Rotation2d.fromDegrees(180)));
        return generateTrajectory(false, Points, null, 1.75, 2.0, 10.0);
    }

    public Trajectory getBounceA(){
        List<Pose2d> Points = new ArrayList<>();
        Points.add(new Pose2d(0, 0, Rotation2d.identity()));
        Points.add(new Pose2d(1.4, 1, Rotation2d.fromDegrees(90)));
        return generateTrajectory(false, Points, null, 1.6, 4.0, 10.0);
    }

    public Trajectory getBounceB(){
        List<Pose2d> Points = new ArrayList<>();
        Points.add(new Pose2d(1.4, 1, Rotation2d.fromDegrees(90)));
        Points.add(new Pose2d(2.35, -2.75, Rotation2d.fromDegrees(180)));
        Points.add(new Pose2d(3.75, 1, Rotation2d.fromDegrees(270)));
        return generateTrajectory(true, Points, null, 1.6, 3.5, 10.0);
    }

    public Trajectory getBounceC(){
        List<Pose2d> Points = new ArrayList<>();
        Points.add(new Pose2d(3.75, 1, Rotation2d.fromDegrees(270)));
        Points.add(new Pose2d(4.9, -2.75, Rotation2d.fromDegrees(0)));
        Points.add(new Pose2d(6.1, 1, Rotation2d.fromDegrees(90)));
        return generateTrajectory(false, Points, null, 1.6, 3.5, 10.0);
    }

    public Trajectory getBounceD(){
        List<Pose2d> Points = new ArrayList<>();
        Points.add(new Pose2d(6.1, 1, Rotation2d.fromDegrees(90)));
        Points.add(new Pose2d(7, -0.6, Rotation2d.fromDegrees(180)));
        return generateTrajectory(true, Points, null, 1.6, 4.0, 10.0);
    }

    public Trajectory getGA1(){
        List<Pose2d> Points = new ArrayList<>();
        Points.add(new Pose2d(0, 0, Rotation2d.identity()));
        Points.add(new Pose2d(-1.35, .45, Rotation2d.fromDegrees(330)));
        Points.add(new Pose2d(-2.9, 1.3, Rotation2d.fromDegrees(315)));
        Points.add(new Pose2d(-3.85, -1.8, Rotation2d.fromDegrees(90)));
        return generateTrajectory(true, Points, Arrays.asList(new VelocityLimitRegionConstraint(new Translation2d(-3,0), new Translation2d(0,2), .6), new VelocityLimitRegionConstraint(new Translation2d(-4.2, -3), new Translation2d(-3.5, -.4), .6)), 1, 3.0, 10.0);
    }

    public Trajectory getGA2(){
        List<Pose2d> Points = new ArrayList<>();
        Points.add(new Pose2d(-3.83, -1.8, Rotation2d.fromDegrees(90)));
        Points.add(new Pose2d(-7.55, -.2, Rotation2d.fromDegrees(180)));
        return generateTrajectory(false, Points, null, 1.75, 5.0, 10.0);
    }

    public Trajectory getGB1(){
        List<Pose2d> Points = new ArrayList<>();
        Points.add(new Pose2d(0, 0, Rotation2d.identity()));
        Points.add(new Pose2d(-3.84, 2.23, Rotation2d.fromDegrees(72)));
        Points.add(new Pose2d(-4.79, -.3, Rotation2d.fromDegrees(67)));
        Points.add(new Pose2d(-6, .4, Rotation2d.fromDegrees(270)));
        return generateTrajectory(true, Points, null/*Arrays.asList(new VelocityLimitRegionConstraint(new Translation2d(-3,0), new Translation2d(0,2), .6), new VelocityLimitRegionConstraint(new Translation2d(-4.2, -3), new Translation2d(-3.5, -.4), .6))*/, .5, 3.0, 10.0);
    }

    public Trajectory getGB2(){
        List<Pose2d> Points = new ArrayList<>();
        Points.add(new Pose2d(-6, .4, Rotation2d.fromDegrees(270)));
        Points.add(new Pose2d(-8, 1, Rotation2d.fromDegrees(0)));
        return generateTrajectory(true, Points, null, 1.75, 5.0, 10.0);
    }

    public Trajectory getGC1(){
        List<Pose2d> Points = new ArrayList<>();
        Points.add(new Pose2d(0, 0, Rotation2d.identity()));
        Points.add(new Pose2d(-1.4, -1.2, Rotation2d.fromDegrees(45)));
        Points.add(new Pose2d(-2.4, 1.07, Rotation2d.fromDegrees(270)));
        Points.add(new Pose2d(-4.62, -.7, Rotation2d.fromDegrees(80)));
        return generateTrajectory(true, Points, null /*Arrays.asList(new VelocityLimitRegionConstraint(new Translation2d(-3,0), new Translation2d(0,2), .6), new VelocityLimitRegionConstraint(new Translation2d(-4.2, -3), new Translation2d(-3.5, -.4), .6))*/, .5, 3.0, 10.0);
    }

    public Trajectory getGC2(){
        List<Pose2d> Points = new ArrayList<>();
        Points.add(new Pose2d(-4.62, -.7, Rotation2d.fromDegrees(80)));
        Points.add(new Pose2d(-7.55, 0, Rotation2d.fromDegrees(180)));
        return generateTrajectory(false, Points, null, 1.75, 5.0, 10.0);
    }

    public Trajectory getGD1(){
        List<Pose2d> Points = new ArrayList<>();
        Points.add(new Pose2d(0, 0, Rotation2d.identity()));
        Points.add(new Pose2d(-3.4, 1.7, Rotation2d.fromDegrees(43)));
        Points.add(new Pose2d(-4.9, .32, Rotation2d.fromDegrees(43)));
        Points.add(new Pose2d(-6.7, 1.56, Rotation2d.fromDegrees(296)));
        return generateTrajectory(true, Points, null /*Arrays.asList(new VelocityLimitRegionConstraint(new Translation2d(-3,0), new Translation2d(0,2), .6), new VelocityLimitRegionConstraint(new Translation2d(-4.2, -3), new Translation2d(-3.5, -.4), .6))*/, .5, 3.0, 10.0);
    }

    public Trajectory getGD2(){
        List<Pose2d> Points = new ArrayList<>();
        Points.add(new Pose2d(-6.7, 1.56, Rotation2d.fromDegrees(296)));
        Points.add(new Pose2d(-7.7, 2, Rotation2d.fromDegrees(330)));
        return generateTrajectory(true, Points, null, 1.75, 5.0, 10.0);
    }
}
