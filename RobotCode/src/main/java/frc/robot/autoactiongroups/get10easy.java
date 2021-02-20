package frc.robot.autoactiongroups;

import frc.lib.models.DriveTrajectoryGenerator;
import frc.lib.statemachine.StateMachineDescriptor;
import frc.robot.actions.driveactions.DriveTra;

public class get10easy extends StateMachineDescriptor {
    public get10easy() {
        addSequential(new DriveTra(DriveTrajectoryGenerator.getInstance().getTwoMeters()), 2000);
    }
}