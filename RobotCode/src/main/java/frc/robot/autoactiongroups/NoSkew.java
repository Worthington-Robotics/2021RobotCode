package frc.robot.autoactiongroups;

import frc.lib.statemachine.StateMachineDescriptor;
import frc.robot.actions.driveactions.*;
import frc.lib.models.DriveTrajectoryGenerator;

public class NoSkew extends StateMachineDescriptor {
    public NoSkew() {
        addSequential(new DriveTra(DriveTrajectoryGenerator.getInstance().getLoop(), false), 1000);
    }
}