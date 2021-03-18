package frc.robot.actions.aiactions;

import frc.lib.geometry.Pose2d;
import frc.lib.models.DriveTrajectoryGenerator;
import frc.lib.statemachine.Action;
import frc.robot.subsystems.JetsonAILink;
import frc.robot.subsystems.Drive;
import frc.robot.subsystems.PoseEstimator;
import frc.robot.subsystems.Superstructure;

import java.util.Arrays;

public class BallFollowAction extends Action {
    @Override public void onStart() {
        Pose2d initialPose = PoseEstimator.getInstance().getLatestFieldToVehicle().getValue();
        Pose2d firstBallPose = JetsonAILink.getInstance().getFirst().transformBy(initialPose);

        Superstructure.getInstance().setIntaking(true);

        Drive.getInstance().setTrajectory(
                DriveTrajectoryGenerator.getInstance()
                        .generateTrajectory(true, Arrays.asList(initialPose, firstBallPose), null,
                                .05, 4, 10));
        
    }

    @Override public void onLoop() {}

    @Override public boolean isFinished() {
        return Drive.getInstance().isDoneWithTrajectory();
    }

    @Override public void onStop() {
        Superstructure.getInstance().setIntaking(false);
    }
}