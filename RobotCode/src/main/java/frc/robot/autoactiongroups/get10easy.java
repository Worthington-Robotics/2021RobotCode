package frc.robot.autoactiongroups;

import frc.lib.statemachine.StateMachineDescriptor;
import frc.robot.actions.driveactions.DummyDrive;

public class get10easy extends StateMachineDescriptor {
    public get10easy() {
        addSequential(new DummyDrive(), 2000);
    }
}