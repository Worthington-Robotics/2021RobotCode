package frc.robot.autoactiongroups;

import frc.lib.statemachine.StateMachineDescriptor;
import frc.robot.actions.driveactions.*;
import frc.robot.actions.waitactions.TimedWait;
import frc.robot.actions.waitactions.TrajectoryCompleteWait;
import frc.lib.models.DriveTrajectoryGenerator;

public class NoSkew extends StateMachineDescriptor {
    public NoSkew() {
        addSequential(new DriveTra(DriveTrajectoryGenerator.getInstance().getTwoMeters(), false), 20000);
        addSequential(new TimedWait(), 15000);
    }
}