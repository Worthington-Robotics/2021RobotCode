/*----------------------------------------------------------------------------*/
/* Copyright (c) 1892-1893 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import java.util.Arrays;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.buttons.JoystickButton;
import edu.wpi.first.wpilibj.command.Scheduler;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.trajectory.TrajectoryGenerator;
import frc.lib.loops.Looper;
import frc.lib.models.DriveTrajectoryGenerator;
import frc.lib.statemachine.Action;
import frc.lib.statemachine.StateMachine;
import frc.lib.util.AxisAction;
import frc.lib.util.DebouncedJoystickButton;
import frc.lib.util.DriveSignal;
import frc.lib.util.POVTrigger;
import frc.lib.util.VersionData;
import frc.robot.subsystems.*;
import frc.robot.actions.driveactions.*;
import frc.robot.actions.aiactions.BallFollowAction;
import frc.robot.actions.climberactions.*;
import frc.robot.actions.colorwheelactions.nextLight;
import frc.robot.actions.shooteraction.*;
import frc.robot.actions.superaction.*;
import frc.robot.actions.waitactions.SoutAction;

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
    
    //Master joystick buttons
    //private JoystickButton gyrPovTrigger = new JoystickButton(Constants.MASTER, 10);
    //private JoystickButton turnLockout = new JoystickButton(Constants.MASTER, 4);
    private JoystickButton DownshiftTrigger = new JoystickButton(Constants.MASTER, 1);
    private JoystickButton inverse = new JoystickButton(Constants.MASTER, 2);
    private JoystickButton unfolder = new JoystickButton(Constants.MASTER, 3);
    private JoystickButton folder = new JoystickButton(Constants.MASTER, 4);
    private JoystickButton unpin = new JoystickButton(Constants.MASTER, 5);
    private JoystickButton climbDown = new JoystickButton(Constants.MASTER, 8);
    private JoystickButton climbUp = new JoystickButton(Constants.MASTER, 10);
    

    //Co-pilot joystick buttons
    private POVTrigger recenter = new POVTrigger(Constants.SECOND);
    private JoystickButton shootOne = new JoystickButton(Constants.SECOND, 1);
    private JoystickButton turretPIDControl = new JoystickButton(Constants.SECOND, 2);
    //private JoystickButton fieldCentricTurret = new JoystickButton(Constants.SECOND, 3);
    private JoystickButton dump = new JoystickButton(Constants.SECOND, 4);
    private JoystickButton limelightRPM = new JoystickButton(Constants.SECOND, 5);
    private JoystickButton manualFlyWheel = new JoystickButton(Constants.SECOND, 6);
    private DebouncedJoystickButton intakeUP = new DebouncedJoystickButton(Constants.SECOND, 9);
    private JoystickButton OffsetUp = new JoystickButton(Constants.SECOND, 10);
    private JoystickButton intake = new JoystickButton(Constants.SECOND, 3);
    private JoystickButton OffsetDown = new JoystickButton(Constants.SECOND, 12);

    
    //Wheel buttons
    private AxisAction reverse = new AxisAction(Constants.WHEEL, 3, .5, false);
    private AxisAction nextLight = new AxisAction(Constants.WHEEL, 2, .8, false);
    private JoystickButton flywheelManual = new JoystickButton(Constants.WHEEL, 1);
    private JoystickButton gyroLock = new JoystickButton(Constants.WHEEL, 2);
    private JoystickButton shootAll = new JoystickButton(Constants.WHEEL, 3);
    private JoystickButton wheelIntake = new JoystickButton(Constants.WHEEL, 4);
    private JoystickButton shiftUp = new JoystickButton(Constants.WHEEL, 5);
    private JoystickButton shiftDown = new JoystickButton(Constants.WHEEL, 6);
    private JoystickButton turretLock = new JoystickButton(Constants.WHEEL, 7);
    private JoystickButton wheelTargeting = new JoystickButton(Constants.WHEEL, 10);
    private JoystickButton wheelIntakeArm = new JoystickButton(Constants.WHEEL, 8);

    /**
     * This function is run when the robot is first started up and should be used
     * for any initialization code.
     */
    @Override
    public void robotInit() {
        manager = new SubsystemManager(Arrays.asList(
            // register subsystems here
            PoseEstimator.getInstance(), 
            Climber.getInstance(), 
            Drive.getInstance(),
            Shooter.getInstance(),
            Lights.getInstance(),
            //JetsonAILink.getInstance(),
            Superstructure.getInstance()),
             true);

        // create the master looper threads
        DriveTrajectoryGenerator.getInstance();
        enabledLooper = new Looper();
        disabledLooper = new Looper();

        // register the looper threads to the manager to use for enabled and disabled
        manager.registerEnabledLoops(enabledLooper);
        manager.registerDisabledLoops(disabledLooper);

        // add any additional logging sources for capture
        manager.addLoggingSource(Arrays.asList(StateMachine.getInstance()));

        // publish the auto list to the dashboard "Auto Selector"
        SmartDashboard.putStringArray("Auto List", AutoSelector.buildArray());

        initButtons();
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

        StateMachine.getInstance().assertStop();
        Superstructure.getInstance().setInit();
        Shooter.getInstance().setRampUp();

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
        //Shooter.getInstance().softStart();
        Drive.getInstance().reset(); 
        PoseEstimator.getInstance().reset();
        Superstructure.getInstance().reset();

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
        Drive.getInstance().setOpenLoop(DriveSignal.NEUTRAL);
        //Constants.WHEELS = SmartDashboard.getBoolean("Drive/Wheel Control", Constants.WHEELS);
        initButtons();
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
        Drive.getInstance().reset();
        PoseEstimator.getInstance().reset();
        //Shooter.getInstance().disable();
        //Superstructure.getInstance().reset();

        enabledLooper.start();
    }

    /**
     * This function is called periodically during test mode.
     */
    @Override
    public void testPeriodic() {
        Scheduler.getInstance().run();
    }

    public void initButtons(){
        // create buttons and register actions
        if(Constants.WHEELS)
        {
        reverse.whileHeld(Action.toCommand(new Inverse()));
        shiftUp.whileHeld(Action.toCommand(new Shift()));
        shiftDown.whileHeld(Action.toCommand(new DownShift()));
        shootAll.whileHeld(Action.toCommand(new ShootAllAction()));
        wheelIntake.whileHeld(Action.toCommand(new IntakeAction()));
        gyroLock.whileHeld(Action.toCommand(new GyroLock()));
        nextLight.whenPressed(Action.toCommand(new nextLight()));
        flywheelManual.whenPressed(Action.toCommand(new SetManualFlywheel()));
        wheelTargeting.whileHeld(Action.toCommand(new TurretPIDControl()));
        wheelIntakeArm.toggleWhenPressed(Action.toCommand(new ToggleIntake()));
        turretLock.whileHeld(Action.toCommand(new TurretPIDControl()));
    }
        climbDown.whileHeld(Action.toCommand(new MotorDownAction()));
        climbUp.whileHeld(Action.toCommand(new MotorUpAction()));
        unfolder.whenPressed(Action.toCommand(new ClimbUpAction()));
        folder.whenPressed(Action.toCommand(new ClimbDownAction()));
        unpin.whenPressed(Action.toCommand(new ClimbUnpinedAction()));
        DownshiftTrigger.whileHeld(Action.toCommand(new DownShift()));
        OffsetUp.whenPressed(Action.toCommand(new OffsetIncrease()));
        OffsetDown.whenPressed(Action.toCommand(new OffsetDecrease()));
        recenter.whileHeld(Action.toCommand(new Recenter(0)));
        //fieldCentricTurret.whenPressed(Action.toCommand(new FieldCentricTurret()));
        turretPIDControl.whileHeld(Action.toCommand(new TurretPIDControl()));
        dump.whileHeld(Action.toCommand(new DumpAction()));
        manualFlyWheel.whenPressed(Action.toCommand(new SetManualFlywheel()));
        inverse.whileHeld(Action.toCommand(new Inverse()));
        shootOne.whenPressed(Action.toCommand(new ShootBallAction()));
        intake.whileHeld(Action.toCommand(new IntakeAction()));
        limelightRPM.whenPressed(Action.toCommand(new softStart()));
        intakeUP.toggleWhenPressed(Action.toCommand(new ToggleIntake()));
        shootOne.whileActive(Action.toCommand(new ShootAllAction()));
        VersionData.WriteBuildInfoToDashboard();

    }
}