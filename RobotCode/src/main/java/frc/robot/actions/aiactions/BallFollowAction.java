package frc.robot.actions.aiactions;

import frc.lib.geometry.Pose2d;
import frc.lib.models.DriveTrajectoryGenerator;
import frc.lib.statemachine.Action;
import frc.lib.util.DriveSignal;
import frc.robot.subsystems.JetsonAILink;
import frc.robot.subsystems.Drive;
import frc.robot.subsystems.PoseEstimator;
import frc.robot.subsystems.Superstructure;

import java.util.Arrays;

public class BallFollowAction extends Action {
    @Override public void onStart() {
        Pose2d initialPose = PoseEstimator.getInstance().getLatestFieldToVehicle().getValue();
        Pose2d firstBallPose = JetsonAILink.getInstance().getFirst().transformBy(initialPose);

        System.out.println(initialPose.toString());
        System.out.println(firstBallPose.toString());

        Drive.getInstance().setTrajectory(
                DriveTrajectoryGenerator.getInstance()
                        .generateTrajectory(true, Arrays.asList(initialPose, firstBallPose), null,
                                .25, 4, 10));

        Superstructure.getInstance().setIntaking(true);
    }

    @Override public void onLoop() {}

    @Override public boolean isFinished() {
        return Drive.getInstance().isDoneWithTrajectory();
    }

    @Override public void onStop() {
        Superstructure.getInstance().setIntaking(false);
        Drive.getInstance().setOpenLoop(DriveSignal.NEUTRAL);
    }
}