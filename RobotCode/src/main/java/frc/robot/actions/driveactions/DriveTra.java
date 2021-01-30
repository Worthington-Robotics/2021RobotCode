package frc.robot.actions.driveactions;

import edu.wpi.first.wpilibj.Timer;
import frc.lib.statemachine.Action;
import frc.lib.trajectory.*;
import frc.robot.subsystems.Drive;
import frc.robot.subsystems.PoseEstimator;


public class DriveTra extends Action {

    private static final Drive mDrive = Drive.getInstance();
    private static final PoseEstimator mRobotState = PoseEstimator.getInstance();
    private final Trajectory mTra;
    private final boolean mResetPose;

    public DriveTra(Trajectory Tra) {
        mTra = Tra;
        mResetPose = false;
    }

    public DriveTra(Trajectory Tra, boolean resetpos) {
        mTra = Tra;
        mResetPose = resetpos;
    }

    @Override
    public void onStart() {
        System.out.println("Starting Tra");
        if (mResetPose) {
            mRobotState.reset(Timer.getFPGATimestamp(), mTra.sample(0).pose);
        }
        mDrive.setTrajectory(mTra);
    }

    @Override
    public void onLoop() {

    }

    @Override
    public boolean isFinished() {
        return true;
    }

    @Override
    public void onStop() {

    }
}
