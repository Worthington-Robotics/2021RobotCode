package frc.robot.autoactiongroups;

import frc.lib.statemachine.StateMachineDescriptor;
import frc.robot.actions.driveactions.*;
import frc.lib.models.DriveTrajectoryGenerator;

public class Barrel extends StateMachineDescriptor {
    public Barrel() {
        addSequential(new DriveTra(DriveTrajectoryGenerator.getInstance().BAR, false), 1000);
    }
}