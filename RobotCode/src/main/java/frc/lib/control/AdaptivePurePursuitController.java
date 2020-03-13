package frc.lib.control;

import java.util.Optional;
import java.util.Set;

import frc.lib.geometry.Pose2d;
import frc.lib.geometry.Rotation2d;
import frc.lib.geometry.Translation2d;
import frc.lib.geometry.Twist2d;
import frc.lib.util.CSVWritable;

/**
 * Implements an adaptive pure pursuit controller. See:
 * https://www.ri.cmu.edu/pub_files/pub1/kelly_alonzo_1994_4/kelly_alonzo_1994_4
 * .pdf
 * 
 * Basically, we find a spot on the path we'd like to follow and calculate the
 * wheel speeds necessary to make us land on that spot. The target spot is a
 * specified distance ahead of us, and we look further ahead the greater our
 * tracking error.
 */
public class AdaptivePurePursuitController {
    private static final double kEpsilon = 1E-9;

    double mFixedLookahead;
    Path mPath;
    Twist2d mLastCommand;
    double mLastTime;
    double mMaxAccel;
    double mDt;
    boolean mReversed;
    double mPathCompletionTolerance;

    public AdaptivePurePursuitController(double fixed_lookahead, double max_accel, double nominal_dt, Path path,
            boolean reversed, double path_completion_tolerance) {
        mFixedLookahead = fixed_lookahead;
        mMaxAccel = max_accel;
        mPath = path;
        mDt = nominal_dt;
        mLastCommand = null;
        mReversed = reversed;
        mPathCompletionTolerance = path_completion_tolerance;
    }

    public boolean isDone() {
        double remainingLength = mPath.getRemainingLength();
        return remainingLength <= mPathCompletionTolerance;
    }

    public ControllerOutput update(Pose2d robot_pose, double now) {
        Pose2d pose = robot_pose;
        if (mReversed) {
            pose = new Pose2d(robot_pose.getTranslation(),
                    robot_pose.getRotation().rotateBy(Rotation2d.fromRadians(Math.PI)));
        }

        double distance_from_path = mPath.update(robot_pose.getTranslation());
        if (this.isDone()) {
            return ControllerOutput.identity();
        }

        PathSegment.Sample lookahead_point = mPath.getLookaheadPoint(robot_pose.getTranslation(),
                distance_from_path + mFixedLookahead);
        Optional<Circle> circle = joinPath(pose, lookahead_point.translation);

        double speed = lookahead_point.speed;
        if (mReversed) {
            speed *= -1;
        }
        // Ensure we don't accelerate too fast from the previous command
        double dt = now - mLastTime;
        if (mLastCommand == null) {
            mLastCommand = new Twist2d(0, 0, 0);
            dt = mDt;
        }
        double accel = (speed - mLastCommand.dx) / dt;
        if (accel < -mMaxAccel) {
            speed = mLastCommand.dx - mMaxAccel * dt;
        } else if (accel > mMaxAccel) {
            speed = mLastCommand.dx + mMaxAccel * dt;
        }

        // Ensure we slow down in time to stop
        // vf^2 = v^2 + 2*a*d
        // 0 = v^2 + 2*a*d
        double remaining_distance = mPath.getRemainingLength();
        double max_allowed_speed = Math.sqrt(2 * mMaxAccel * remaining_distance);
        if (Math.abs(speed) > max_allowed_speed) {
            speed = max_allowed_speed * Math.signum(speed);
        }
        final double kMinSpeed = 4.0;
        if (Math.abs(speed) < kMinSpeed) {
            // Hack for dealing with problems tracking very low speeds with
            // Talons
            speed = kMinSpeed * Math.signum(speed);
        }

        Twist2d rv;
        if (circle.isPresent()) {
            rv = new Twist2d(speed, 0,
                    (circle.get().turn_right ? -1 : 1) * Math.abs(speed) / circle.get().radius);
        } else {
            rv = new Twist2d(speed, 0, 0);
        }
        mLastTime = now;
        mLastCommand = rv;
        return new ControllerOutput(rv, lookahead_point, remaining_distance);
    }

    public Set<String> getMarkersCrossed() {
        return mPath.getMarkersCrossed();
    }

    public static class Circle {
        public final Translation2d center;
        public final double radius;
        public final boolean turn_right;

        public Circle(Translation2d center, double radius, boolean turn_right) {
            this.center = center;
            this.radius = radius;
            this.turn_right = turn_right;
        }
    }

    public static Optional<Circle> joinPath(Pose2d robot_pose, Translation2d lookahead_point) {
        double x1 = robot_pose.getTranslation().x();
        double y1 = robot_pose.getTranslation().y();
        double x2 = lookahead_point.x();
        double y2 = lookahead_point.y();

        Translation2d pose_to_lookahead = robot_pose.getTranslation().inverse().translateBy(lookahead_point);
        double cross_product = pose_to_lookahead.x() * robot_pose.getRotation().sin()
                - pose_to_lookahead.y() * robot_pose.getRotation().cos();
        if (Math.abs(cross_product) < kEpsilon) {
            return Optional.empty();
        }

        double dx = x1 - x2;
        double dy = y1 - y2;
        double my = (cross_product > 0 ? -1 : 1) * robot_pose.getRotation().cos();
        double mx = (cross_product > 0 ? 1 : -1) * robot_pose.getRotation().sin();

        double cross_term = mx * dx + my * dy;

        if (Math.abs(cross_term) < kEpsilon) {
            // Points are colinear
            return Optional.empty();
        }

        return Optional.of(new Circle(
                new Translation2d((mx * (x1 * x1 - x2 * x2 - dy * dy) + 2 * my * x1 * dy) / (2 * cross_term),
                        (-my * (-y1 * y1 + y2 * y2 + dx * dx) + 2 * mx * y1 * dx) / (2 * cross_term)),
                .5 * Math.abs((dx * dx + dy * dy) / cross_term), cross_product > 0));
    }

    public static class ControllerOutput implements CSVWritable{
        public Twist2d command;
        public PathSegment.Sample lookahead;
        public double remaining_distance;

        private static final ControllerOutput identity = new ControllerOutput(Twist2d.identity(), 
            new PathSegment.Sample(Translation2d.identity(), 0), 0);
        public static ControllerOutput identity(){ return identity;}
        
        public ControllerOutput(Twist2d command, PathSegment.Sample lookahead, double remaining_distance){
            this.command = command;
            this.lookahead = lookahead;
            this.remaining_distance = remaining_distance;
        }

		@Override
		public String toCSV() {
			return command.toCSV() + ", " + lookahead.translation.toCSV() + ", " + lookahead.speed + ", " + remaining_distance;
		}

		@Override
		public int getNumFields() {
			return command.getNumFields() + lookahead.translation.getNumFields() + 1 + 1;
        }
        
        @Override
        public String toString() {
            return "Twist: " + command.toString() + ", Lookahead point: " + lookahead.translation.toString() + 
            ", Lookahead speed: " + lookahead.speed + ", Remaining path: " + remaining_distance;
        }

    }   

}
