package frc.robot.autoactiongroups;

import frc.lib.statemachine.StateMachineDescriptor;
import frc.robot.actions.driveactions.*;
import frc.robot.actions.waitactions.LineCrossWait;
import frc.robot.actions.waitactions.PointCloudWait;
import frc.robot.actions.waitactions.TrajectoryCompleteWait;
import frc.lib.models.DriveTrajectoryGenerator;

public class BoingBoing extends StateMachineDescriptor {
    public BoingBoing() {
        addSequential(new DriveTra(DriveTrajectoryGenerator.getInstance().getBounceA(), false), 100);
        addSequential(new LineCrossWait(.85, true, false), 7500);
        addSequential(new DriveTra(DriveTrajectoryGenerator.getInstance().getBounceB(), false), 100);
        addSequential(new LineCrossWait(-2.8, true, true), 7500);
        addSequential(new DriveTra(DriveTrajectoryGenerator.getInstance().getBounceC(), false), 100);
        addSequential(new LineCrossWait(.85, true, false), 7500);
        addSequential(new DriveTra(DriveTrajectoryGenerator.getInstance().getBounceD(), false), 100);
        addSequential(new LineCrossWait(0, true, true), 7500);
        addSequential(new LineCrossWait(.85, true, false), 7500);
        addSequential(new DriveTra(DriveTrajectoryGenerator.getInstance().getBounceF(), false), 100);
    }
}