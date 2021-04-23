package frc.robot.autoactiongroups;

import frc.lib.models.DriveTrajectoryGenerator;
import frc.lib.statemachine.StateMachineDescriptor;
import frc.robot.actions.driveactions.DriveTra;

public class ThreeByThree extends StateMachineDescriptor {
    public ThreeByThree(){
        addSequential(new DriveTra(DriveTrajectoryGenerator.getInstance().getTwoMeters()), 50);
    }
}
