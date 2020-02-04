/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import java.util.Arrays;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.buttons.JoystickButton;
import edu.wpi.first.wpilibj.command.Scheduler;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.lib.loops.Looper;
import frc.lib.statemachine.Action;
import frc.lib.statemachine.StateMachine;
import frc.robot.actions.climberactions.*;
import frc.robot.subsystems.Climber;
import frc.robot.subsystems.ColorWheel;
import frc.robot.subsystems.Lights;

/**
 * The VM is configured to automatically run this class, and to call the
 * functions corresponding to each mode, as described in the TimedRobot
 * documentation. If you change the name of this class or the package after
 * creating this project, you must also update the build.gradle file in the
 * project.
 */
public class Robot extends TimedRobot {
    private SubsystemManager manager;
    private Looper enabledLooper, disabledLooper;
    private JoystickButton climbUp, climbDown, unfoldClimb, foldClimb;

    /**
     * This function is run when the robot is first started up and should be
     * used for any initialization code.
     */
    @Override
    public void robotInit() {
        manager = new SubsystemManager(Arrays.asList(
            //register subsystems here
            Lights.getInstance(),
            Climber.getInstance(),
            ColorWheel.getInstance()
        ), true);

        //create the master looper threads
        enabledLooper = new Looper();
        disabledLooper = new Looper();

        //register the looper threads to the manager to use for enabled and disabled
        manager.registerEnabledLoops(enabledLooper);
        manager.registerDisabledLoops(disabledLooper);

        //add any additional logging sources for capture
        manager.addLoggingSource(Arrays.asList(
            StateMachine.getInstance()
        ));

        // publish the auto list to the dashboard "Auto Selector"
        SmartDashboard.putStringArray("Auto List", AutoSelector.buildArray()); 

        foldClimb = new JoystickButton(Constants.MASTER, 9);
        unfoldClimb = new JoystickButton(Constants.MASTER, 10);
        climbDown = new JoystickButton(Constants.MASTER, 11);
        climbUp = new JoystickButton(Constants.MASTER, 12);
        foldClimb.whenPressed(Action.toCommand(new FoldAction()));
        unfoldClimb.whenPressed(Action.toCommand(new UnfoldAction()));
        climbDown.whenPressed(Action.toCommand(new ClimbDownAction()));
        climbUp.whenPressed(Action.toCommand(new ClimbUpAction()));
    }

    /**
     * This function is called every robot packet, no matter the mode. Use
     * this for items like diagnostics that you want ran during disabled,
     * autonomous, teleoperated and test.
     *
     * <p>This runs after the mode specific periodic functions, but before
     * LiveWindow and SmartDashboard integrated updating.
     */
    @Override
    public void robotPeriodic() {
        manager.outputTelemetry();
    }

    @Override
    public void disabledInit() {
        enabledLooper.stop();

        //Run any reset code here
        StateMachine.getInstance().assertStop();

        disabledLooper.start();
    }

    /**
     * This autonomous (along with the chooser code above) shows how to select
     * between different autonomous modes using the dashboard. The sendable
     * chooser code works with the Java SmartDashboard. If you prefer the
     * LabVIEW Dashboard, remove all of the chooser code and uncomment the
     * getString line to get the auto name from the text box below the Gyro
     *
     * <p>You can add additional auto modes by adding additional comparisons to
     * the switch structure below with additional strings. If using the
     * SendableChooser make sure to add them to the chooser code above as well.
     */
    @Override
    public void autonomousInit() {
        disabledLooper.stop();

        //reset anything here

        enabledLooper.start();

        String[] autoList = AutoSelector.buildArray();

        //pulls auto selector from labview DB
        String autoSelected = SmartDashboard.getString("Auto Selector", autoList[autoList.length - 1]);

        //schedule the state machine to run the selected autonomous
        StateMachine.getInstance().runMachine(AutoSelector.autoSelect(autoSelected));
    }

    /**
     * This function is called periodically during autonomous.
     */
    @Override
    public void autonomousPeriodic() {
        
    }

    @Override
    public void teleopInit() {
        disabledLooper.stop();

        //reset anything here

        enabledLooper.start();
    }

    /**
     * This function is called periodically during operator control.
     */
    @Override
    public void teleopPeriodic() {
        Scheduler.getInstance().run();
    }

    @Override
    public void testInit() {
        disabledLooper.stop();

        //reset anything here

        enabledLooper.start();
    }

    /**
     * This function is called periodically during test mode.
     */
    @Override
    public void testPeriodic() {
        
    }
}
