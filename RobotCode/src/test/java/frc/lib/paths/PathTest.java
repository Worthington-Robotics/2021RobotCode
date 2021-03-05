package frc.lib.paths;

import org.junit.Test;

import frc.lib.geometry.Pose2d;
import frc.lib.geometry.Rotation2d;
import frc.lib.trajectory.Trajectory;
import frc.lib.util.TestLogger;
import frc.lib.util.Logable.LogData;
import frc.lib.models.DriveTrajectoryGenerator;

public class PathTest {
    @Test
    public void skiPathTest() {
        Trajectory trajectory = DriveTrajectoryGenerator.getInstance().getSki();
        TestData data = new TestData();
        TestLogger<TestData> logger = new TestLogger<TestData>(data, "skiPathTest");
        Pose2d pose = new Pose2d();
                for(int i = 0; i < trajectory.getStates().size(); i++)
                {
                    pose = trajectory.getStates().get(i).getPose();
                    data.setPoseGuess(pose);
                    logger.update(data, i);
                    
                }
    }

    @Test
    public void barPathTest() {
        Trajectory trajectory = DriveTrajectoryGenerator.getInstance().getBar();
        TestData data = new TestData();
        TestLogger<TestData> logger = new TestLogger<TestData>(data, "barPathTest");
        Pose2d pose = new Pose2d();
                for(int i = 0; i < trajectory.getStates().size(); i++)
                {
                    pose = trajectory.getStates().get(i).getPose();
                    data.setPoseGuess(pose);
                    logger.update(data, i);
                    
                }
    }

    @Test
    public void bonPathTest() {
        Trajectory trajectoryA = DriveTrajectoryGenerator.getInstance().getBounceA();
        Trajectory trajectoryB = DriveTrajectoryGenerator.getInstance().getBounceB();
        Trajectory trajectoryC = DriveTrajectoryGenerator.getInstance().getBounceC();
        Trajectory trajectoryD = DriveTrajectoryGenerator.getInstance().getBounceD();
        TestData data = new TestData();
        TestLogger<TestData> logger = new TestLogger<TestData>(data, "bonPathTest");
        Pose2d pose = new Pose2d();
                for(int i = 0; i < trajectoryA.getStates().size(); i++)
                {
                    pose = trajectoryA.getStates().get(i).getPose();
                    data.setPoseGuess(pose);
                    logger.update(data, i);
                    
                }
                for(int i = 0; i < trajectoryB.getStates().size(); i++)
                {
                    pose = trajectoryB.getStates().get(i).getPose();
                    data.setPoseGuess(pose);
                    logger.update(data, i);
                    
                }
                for(int i = 0; i < trajectoryC.getStates().size(); i++)
                {
                    pose = trajectoryC.getStates().get(i).getPose();
                    data.setPoseGuess(pose);
                    logger.update(data, i);
                    
                }
                for(int i = 0; i < trajectoryD.getStates().size(); i++)
                {
                    pose = trajectoryD.getStates().get(i).getPose();
                    data.setPoseGuess(pose);
                    logger.update(data, i);
                    
                }
    }
    
    @Test
    public void BallPathTest() {
        Trajectory trajectoryA = DriveTrajectoryGenerator.getInstance().getBounceA();
        Trajectory trajectoryB = DriveTrajectoryGenerator.getInstance().getBounceB();
        Trajectory trajectoryC = DriveTrajectoryGenerator.getInstance().getBounceC();
        Trajectory trajectoryD = DriveTrajectoryGenerator.getInstance().getBounceD();
        TestData data = new TestData();
        TestLogger<TestData> logger = new TestLogger<TestData>(data, "bonPathTest");
        Pose2d pose = new Pose2d();
                for(int i = 0; i < trajectoryA.getStates().size(); i++)
                {
                    pose = trajectoryA.getStates().get(i).getPose();
                    data.setPoseGuess(pose);
                    logger.update(data, i);
                    
                }
                for(int i = 0; i < trajectoryB.getStates().size(); i++)
                {
                    pose = trajectoryB.getStates().get(i).getPose();
                    data.setPoseGuess(pose);
                    logger.update(data, i);
                    
                }
                for(int i = 0; i < trajectoryC.getStates().size(); i++)
                {
                    pose = trajectoryC.getStates().get(i).getPose();
                    data.setPoseGuess(pose);
                    logger.update(data, i);
                    
                }
                for(int i = 0; i < trajectoryD.getStates().size(); i++)
                {
                    pose = trajectoryD.getStates().get(i).getPose();
                    data.setPoseGuess(pose);
                    logger.update(data, i);
                    
                }
    }

    @Test
    public void bonBPathTest() {
        Trajectory trajectory = DriveTrajectoryGenerator.getInstance().getBounceB();
        TestData data = new TestData();
        TestLogger<TestData> logger = new TestLogger<TestData>(data, "bonBPathTest");
        Pose2d pose = new Pose2d();
                for(int i = 0; i < trajectory.getStates().size(); i++)
                {
                    pose = trajectory.getStates().get(i).getPose();
                    data.setPoseGuess(pose);
                    logger.update(data, i);
                    
                }
    }

    @Test
    public void bonCPathTest() {
        Trajectory trajectory = DriveTrajectoryGenerator.getInstance().getBounceC();
        TestData data = new TestData();
        TestLogger<TestData> logger = new TestLogger<TestData>(data, "bonCPathTest");
        Pose2d pose = new Pose2d();
                for(int i = 0; i < trajectory.getStates().size(); i++)
                {
                    pose = trajectory.getStates().get(i).getPose();
                    data.setPoseGuess(pose);
                    logger.update(data, i);
                    
                }
    }

    public static class TestData extends LogData {
        public Pose2d expectedPose = new Pose2d();
        public Rotation2d angle = Rotation2d.fromDegrees(0);

        public void setPoseGuess(Pose2d pose)
        {
            this.expectedPose = pose;
            this.angle = pose.getRotation();
        }
}
}
