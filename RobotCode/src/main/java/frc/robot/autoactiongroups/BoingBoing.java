package frc.robot.autoactiongroups;

import frc.lib.statemachine.StateMachineDescriptor;
import frc.robot.actions.driveactions.*;
import frc.robot.actions.waitactions.TrajectoryCompleteWait;
import frc.lib.models.DriveTrajectoryGenerator;

public class BoingBoing extends StateMachineDescriptor {
    public BoingBoing() {
        addSequential(new DriveTra(DriveTrajectoryGenerator.getInstance().getBounceA(), false), 100);
        addSequential(new TrajectoryCompleteWait(), 7500);
        //addSequential(new DriveTra(DriveTrajectoryGenerator.getInstance().getBounceB(), false), 100);
        //addSequential(new TrajectoryCompleteWait(), 7500);
        //addParallel(new Action[] {new DriveTra(DriveTrajectoryGenerator.getInstance().getBounceC(), false), new TrajectoryCompleteWait()}, 7500);
        //addParallel(new Action[] {new DriveTra(DriveTrajectoryGenerator.getInstance().getBounceD(), false), new TrajectoryCompleteWait()}, 7500);
    }
}