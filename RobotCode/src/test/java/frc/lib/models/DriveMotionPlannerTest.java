// package frc.lib.models;

// import static org.junit.Assert.assertTrue;

// import java.util.ArrayList;
// import java.util.Arrays;
// import java.util.List;

// import org.junit.Test;

// import frc.lib.geometry.Pose2d;
// import frc.lib.geometry.Rotation2d;
// import frc.lib.geometry.Twist2d;
// import frc.lib.trajectory.TimedState;
// import frc.lib.trajectory.Trajectory;
// import frc.lib.trajectory.constraint.CentripetalAccelerationConstraint;
// import frc.lib.util.TestLogger;
// import frc.lib.util.Util;
// import frc.lib.util.Logable.LogData;
// import frc.robot.Constants;
// import frc.robot.Kinematics;
// import frc.lib.models.DriveMotionPlanner.FollowerType;

// public class DriveMotionPlannerTest {

//     protected static double kEpsilon = 0.01;

//     // NOTE: this test assumes the trajectory has been created such that it always
//     // moves closer to the end
//     // this allows to check for knotting or any other undesirable behaviour
//     @Test
//     public void forwardTrajectory() {
//         DriveMotionPlanner planner = new DriveMotionPlanner();
//         List<Pose2d> points = new ArrayList<>();
//         points.add(new Pose2d(0, 0, Rotation2d.fromDegrees(0)));
//         points.add(new Pose2d(4, 0, Rotation2d.fromDegrees(0)));
//         Trajectory trajectory = planner.generateTrajectory(false, points,
//                 Arrays.asList(new CentripetalAccelerationConstraint(60)), 0, 0, 2, 2, 10.0);
//         assertTrue("trajectory time is not sane", trajectory.getTotalTimeSeconds() < 10.0);
//         assertTrue("Trajectory is not always heading towards end", checkTrajectoryHeadsForEnd(trajectory));
//     }

//     // NOTE: this test assumes the trajectory has been created such that it always
//     // moves closer to the end
//     // this allows to check for knotting or any other undesirable behaviour
//     @Test
//     public void forwardSwerveRightTrajectory() {
//         DriveMotionPlanner planner = new DriveMotionPlanner();
//         List<Pose2d> points = new ArrayList<>();
//         points.add(new Pose2d(0, 0, Rotation2d.fromDegrees(0)));
//         points.add(new Pose2d(4, 4, Rotation2d.fromDegrees(90)));
//         Trajectory trajectory = planner.generateTrajectory(false, points,
//                 Arrays.asList(new CentripetalAccelerationConstraint(60)), 0, 0, 2, 2, 10.0);
//         assertTrue("trajectory time is not sane", trajectory.getTotalTimeSeconds() < 10.0);
//         assertTrue("Trajectory is not always heading towards end", checkTrajectoryHeadsForEnd(trajectory));
//     }

//     /**
//      * helper function to detmine if a trajectory is always getting closer to its
//      * end
//      * 
//      * @param trajectory
//      * @return if the trajectory always gets closer to its end
//      */
//     public boolean checkTrajectoryHeadsForEnd(Trajectory trajectory) {
//         TimedState start = trajectory.sample(0), end = trajectory.sample(trajectory.getTotalTimeSeconds());
//         double lastDistance = start.getPose().distance(end.pose);
//         boolean pass = true;
//         for (TimedState state : trajectory.getStates()) {
//             if (state.equals(start) || state.equals(end))
//                 continue;
//             pass &= state.getPose().distance(end.getPose()) < lastDistance;
//             lastDistance = state.getPose().distance(end.pose);
//         }
//         return pass;
//     }

//     @Test
//     public void forwardFFD() {
//         DriveMotionPlanner planner = new DriveMotionPlanner();

//         // create list of waypoints
//         Pose2d pose = new Pose2d(0, 0, Rotation2d.fromDegrees(0)), end = new Pose2d(4, 0, Rotation2d.fromDegrees(0));
//         List<Pose2d> points = new ArrayList<>();
//         points.add(pose);
//         points.add(end);

//         // configure planner
//         Trajectory trajectory = planner.generateTrajectory(false, points,
//                 Arrays.asList(new CentripetalAccelerationConstraint(60)), 0, 0, 2, 2, 10.0);
//         planner.setTrajectory(trajectory);
//         planner.setFollowerType(FollowerType.FEEDFORWARD_ONLY);
//         double dt = 0.01;

//         // create data storage
//         TestData data = new TestData();
//         DriveMotionPlanner.Output output;
//         double rightAdjVel = 0.0, leftAdjVel = 0.0;

//         TestLogger<TestData> logger = new TestLogger<TestData>(data, "forwardFFD");
//         for (double time = 0.0; time < trajectory.getTotalTimeSeconds(); time += dt) {
//             output = planner.update(time, pose);
//             rightAdjVel = output.right_velocity * Constants.DRIVE_WHEEL_RADIUS;
//             leftAdjVel = output.left_velocity * Constants.DRIVE_WHEEL_RADIUS;
//             Twist2d delta = Kinematics.forwardKinematics(leftAdjVel, rightAdjVel);
//             // Add some systemic error.
//             // delta = new Twist2d(delta.dx * 1.0 * dt, delta.dy * 1.0 * dt, delta.dtheta *
//             // 1.00 * dt);
//             pose = pose.exp(delta.scaled(dt));

//             data.setFields(planner.setpoint(), pose, delta, leftAdjVel, rightAdjVel);
//             logger.update(data, time);
//         }
//         assertTrue(
//                 "End pose does not match end of trajectory. Expected: " + end.toString() + " got: " + pose.toString(),
//                 poseEpsilon(pose, end));

//     }

//     @Test
//     public void forwardArcRightFFD() {
//         DriveMotionPlanner planner = new DriveMotionPlanner();

//         // create list of waypoints
//         Pose2d pose = new Pose2d(0, 0, Rotation2d.fromDegrees(0)), end = new Pose2d(4, 4, Rotation2d.fromDegrees(90));
//         List<Pose2d> points = new ArrayList<>();
//         points.add(pose);
//         points.add(end);

//         // configure planner
//         Trajectory trajectory = planner.generateTrajectory(false, points,
//                 Arrays.asList(new CentripetalAccelerationConstraint(60)), 0, 0, 2, 2, 10.0);
//         planner.setTrajectory(trajectory);
//         planner.setFollowerType(FollowerType.FEEDFORWARD_ONLY);
//         double dt = 0.01;

//         // create data storage
//         TestData data = new TestData();
//         DriveMotionPlanner.Output output;
//         double rightAdjVel = 0.0, leftAdjVel = 0.0;

//         TestLogger<TestData> logger = new TestLogger<TestData>(data, "forwardArcRightFFD");
//         for (double time = 0.0; time < trajectory.getTotalTimeSeconds(); time += dt) {
//             output = planner.update(time, pose);
//             rightAdjVel = output.right_velocity * Constants.DRIVE_WHEEL_RADIUS;
//             leftAdjVel = output.left_velocity * Constants.DRIVE_WHEEL_RADIUS;
//             Twist2d delta = Kinematics.forwardKinematics(leftAdjVel, rightAdjVel);
//             // Add some systemic error.
//             // delta = new Twist2d(delta.dx * 1.0 * dt, delta.dy * 1.0 * dt, delta.dtheta *
//             // 1.00 * dt);
//             pose = pose.exp(delta.scaled(dt));

//             data.setFields(planner.setpoint(), pose, delta, leftAdjVel, rightAdjVel);
//             logger.update(data, time);
//         }
//         assertTrue(
//                 "End pose does not match end of trajectory. Expected: " + end.toString() + " got: " + pose.toString(),
//                 poseEpsilon(pose, end));
//     }

//     @Test
//     public void forwardSwerveRightFFD() {
//         DriveMotionPlanner planner = new DriveMotionPlanner();

//         // create list of waypoints
//         Pose2d pose = new Pose2d(0, 0, Rotation2d.fromDegrees(0)), mid1 = new Pose2d(1, 0, Rotation2d.fromDegrees(0)),
//                 mid2 = new Pose2d(3, 2, Rotation2d.fromDegrees(0)), end = new Pose2d(4, 2, Rotation2d.fromDegrees(0));
//         List<Pose2d> points = new ArrayList<>();
//         points.add(pose);
//         points.add(mid1);
//         points.add(mid2);
//         points.add(end);

//         // configure planner
//         Trajectory trajectory = planner.generateTrajectory(false, points,
//                 Arrays.asList(new CentripetalAccelerationConstraint(60)), 0, 0, 2, 2, 10.0);
//         planner.setTrajectory(trajectory);
//         planner.setFollowerType(FollowerType.FEEDFORWARD_ONLY);
//         double dt = 0.01;

//         // create data storage
//         TestData data = new TestData();
//         DriveMotionPlanner.Output output;
//         double rightAdjVel = 0.0, leftAdjVel = 0.0;

//         TestLogger<TestData> logger = new TestLogger<TestData>(data, "forwardSwerveRightFFD");
//         for (double time = 0.0; time < trajectory.getTotalTimeSeconds(); time += dt) {
//             output = planner.update(time, pose);
//             rightAdjVel = output.right_velocity * Constants.DRIVE_WHEEL_RADIUS;
//             leftAdjVel = output.left_velocity * Constants.DRIVE_WHEEL_RADIUS;
//             Twist2d delta = Kinematics.forwardKinematics(leftAdjVel, rightAdjVel);
//             // Add some systemic error.
//             // delta = new Twist2d(delta.dx * 1.0 * dt, delta.dy * 1.0 * dt, delta.dtheta *
//             // 1.00 * dt);
//             pose = pose.exp(delta.scaled(dt));

//             data.setFields(planner.setpoint(), pose, delta, leftAdjVel, rightAdjVel);
//             logger.update(data, time);
//         }
//         assertTrue(
//                 "End pose does not match end of trajectory. Expected: " + end.toString() + " got: " + pose.toString(),
//                 poseEpsilon(pose, end));
//     }

//     @Test
//     public void forwardController() {
//         DriveMotionPlanner planner = new DriveMotionPlanner();

//         // create list of waypoints
//         Pose2d pose = new Pose2d(0, 0, Rotation2d.fromDegrees(0)), end = new Pose2d(4, 0, Rotation2d.fromDegrees(0));
//         List<Pose2d> points = new ArrayList<>();
//         points.add(pose);
//         points.add(end);

//         // configure planner
//         Trajectory trajectory = planner.generateTrajectory(false, points,
//                 Arrays.asList(new CentripetalAccelerationConstraint(60)), 0, 0, 2, 2, 10.0);
//         planner.setTrajectory(trajectory);
//         planner.setFollowerType(FollowerType.NONLINEAR_FEEDBACK);
//         double dt = 0.01;

//         // create data storage
//         TestData data = new TestData();
//         DriveMotionPlanner.Output output;
//         double rightAdjVel = 0.0, leftAdjVel = 0.0;

//         TestLogger<TestData> logger = new TestLogger<TestData>(data, "forwardController");
//         for (double time = 0.0; time < trajectory.getTotalTimeSeconds(); time += dt) {
//             output = planner.update(time, pose);
//             rightAdjVel = output.right_velocity * Constants.DRIVE_WHEEL_RADIUS;
//             leftAdjVel = output.left_velocity * Constants.DRIVE_WHEEL_RADIUS;
//             Twist2d delta = Kinematics.forwardKinematics(leftAdjVel, rightAdjVel);
//             // Add some systemic error.
//             // delta = new Twist2d(delta.dx * 1.0 * dt, delta.dy * 1.0 * dt, delta.dtheta *
//             // 1.00 * dt);
//             pose = pose.exp(delta.scaled(dt));

//             data.setFields(planner.setpoint(), pose, delta, leftAdjVel, rightAdjVel);
//             logger.update(data, time);
//         }
//         assertTrue(
//                 "End pose does not match end of trajectory. Expected: " + end.toString() + " got: " + pose.toString(),
//                 poseEpsilon(pose, end));

//     }

//     @Test
//     public void forwardArcRightController() {
//         DriveMotionPlanner planner = new DriveMotionPlanner();

//         // create list of waypoints
//         Pose2d pose = new Pose2d(0, 0, Rotation2d.fromDegrees(0)), end = new Pose2d(4, 4, Rotation2d.fromDegrees(90));
//         List<Pose2d> points = new ArrayList<>();
//         points.add(pose);
//         points.add(end);

//         // configure planner
//         Trajectory trajectory = planner.generateTrajectory(false, points,
//                 Arrays.asList(new CentripetalAccelerationConstraint(60)), 0, 0, 2, 2, 10.0);
//         planner.setTrajectory(trajectory);
//         planner.setFollowerType(FollowerType.NONLINEAR_FEEDBACK);
//         double dt = 0.01;

//         // create data storage
//         TestData data = new TestData();
//         DriveMotionPlanner.Output output;
//         double rightAdjVel = 0.0, leftAdjVel = 0.0;

//         TestLogger<TestData> logger = new TestLogger<TestData>(data, "forwardArcRightController");
//         for (double time = 0.0; time < trajectory.getTotalTimeSeconds(); time += dt) {
//             output = planner.update(time, pose);
//             rightAdjVel = output.right_velocity * Constants.DRIVE_WHEEL_RADIUS;
//             leftAdjVel = output.left_velocity * Constants.DRIVE_WHEEL_RADIUS;
//             Twist2d delta = Kinematics.forwardKinematics(leftAdjVel, rightAdjVel);
//             // Add some systemic error.
//             // delta = new Twist2d(delta.dx * 1.0 * dt, delta.dy * 1.0 * dt, delta.dtheta *
//             // 1.00 * dt);
//             pose = pose.exp(delta.scaled(dt));

//             data.setFields(planner.setpoint(), pose, delta, leftAdjVel, rightAdjVel);
//             logger.update(data, time);
//         }
//         assertTrue(
//                 "End pose does not match end of trajectory. Expected: " + end.toString() + " got: " + pose.toString(),
//                 poseEpsilon(pose, end));
//     }

//     @Test
//     public void forwardSwerveRightController() {
//         DriveMotionPlanner planner = new DriveMotionPlanner();

//         // create list of waypoints
//         Pose2d pose = new Pose2d(0, 0, Rotation2d.fromDegrees(0)), mid1 = new Pose2d(1, 0, Rotation2d.fromDegrees(0)),
//                 mid2 = new Pose2d(3, 2, Rotation2d.fromDegrees(0)), end = new Pose2d(4, 2, Rotation2d.fromDegrees(0));
//         List<Pose2d> points = new ArrayList<>();
//         points.add(pose);
//         points.add(mid1);
//         points.add(mid2);
//         points.add(end);

//         // configure planner
//         Trajectory trajectory = planner.generateTrajectory(false, points,
//                 Arrays.asList(new CentripetalAccelerationConstraint(60)), 0, 0, 2, 2, 10.0);
//         planner.setTrajectory(trajectory);
//         planner.setFollowerType(FollowerType.NONLINEAR_FEEDBACK);
//         double dt = 0.01;

//         // create data storage
//         TestData data = new TestData();
//         DriveMotionPlanner.Output output;
//         double rightAdjVel = 0.0, leftAdjVel = 0.0;

//         TestLogger<TestData> logger = new TestLogger<TestData>(data, "forwardSwerveRightController");
//         for (double time = 0.0; time < trajectory.getTotalTimeSeconds(); time += dt) {
//             output = planner.update(time, pose);
//             rightAdjVel = output.right_velocity * Constants.DRIVE_WHEEL_RADIUS;
//             leftAdjVel = output.left_velocity * Constants.DRIVE_WHEEL_RADIUS;
//             Twist2d delta = Kinematics.forwardKinematics(leftAdjVel, rightAdjVel);
//             // Add some systemic error.
//             // delta = new Twist2d(delta.dx * 1.0 * dt, delta.dy * 1.0 * dt, delta.dtheta *
//             // 1.00 * dt);
//             pose = pose.exp(delta.scaled(dt));

//             data.setFields(planner.setpoint(), pose, delta, leftAdjVel, rightAdjVel);
//             logger.update(data, time);
//         }
//         assertTrue(
//                 "End pose does not match end of trajectory. Expected: " + end.toString() + " got: " + pose.toString(),
//                 poseEpsilon(pose, end));
//     }

//     /**
//      * helper function that allows a pose to be equal within some roundoff error
//      * margin
//      * 
//      * @param first
//      * @param second
//      * @return
//      */
//     public static boolean poseEpsilon(Pose2d first, Pose2d second) {
//         return first.getTranslation().epsilonEquals(second.getTranslation(), kEpsilon)
//                 && Util.epsilonEquals(first.getRotation().getRadians(), second.getRotation().getRadians(), kEpsilon);
//     }

//     public static class TestData extends LogData {
//         public double expectedTime = 0.0;
//         public double expectedVel = 0.0;
//         public double expectedAccel = 0.0;
//         public Pose2d expectedPose = new Pose2d();
//         public Pose2d actualPose = new Pose2d();
//         public Twist2d outputTwist = new Twist2d(0, 0, 0);
//         public double leftCalcVel = 0.0;
//         public double rightCalcVel = 0.0;

//         public void setFields(TimedState state, Pose2d pose, Twist2d kinematicResult, double leftCalcVel,
//                 double rightCalcVel) {
//             expectedTime = state.time;
//             expectedVel = state.velocity;
//             expectedAccel = state.acceleration;
//             expectedPose = state.getPose();
//             actualPose = pose;
//             outputTwist = kinematicResult;
//             this.rightCalcVel = rightCalcVel;
//             this.leftCalcVel = leftCalcVel;
//         }

//     }

// }