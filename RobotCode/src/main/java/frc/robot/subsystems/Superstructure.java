package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.SupplyCurrentLimitConfiguration;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.playingwithfusion.TimeOfFlight;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.lib.drivers.SimTimeOfFlight;
import frc.lib.loops.ILooper;
import frc.lib.loops.Loop;
import frc.robot.Constants;

/**
 * The Superstructure subsystem combines both the Indexer and Intake systems for
 * performance economy.
 *
 * This version of the Superstructure is designed for the second design
 * iteration of the robot, specifically noting the new top wheels used in the
 * Indexer.
 *
 * @author Kayla
 */
public class Superstructure extends Subsystem {
    // TalonSRX mtr1;
    // TalonSRX mtr2;
    // TalonSRX mtr3;
    // TalonSRX mtr4;
    // TalonSRX mtr5;
    TalonSRX[] mtrArray;

    SimTimeOfFlight[] sensorArray;
    // SimTimeOfFlight sen1;
    // SimTimeOfFlight sen2;
    // SimTimeOfFlight sen3;
    // SimTimeOfFlight sen4;
    // SimTimeOfFlight sen5;

    double[] sensorThresholds = { 75, 60, 60, 75, 75 };

    SuperStructureIO periodicIO;

    private static Superstructure instance = new Superstructure();

    public static Superstructure getInstance() {
        return instance;
    }

    /**
     * Initializes motors, sensors, and the intake's DoubleSolenoid arm. Also resets
     * IO, motors, and sensors to default functioning settings.
     */
    private Superstructure() {
        mtrArray = new TalonSRX[5];
        mtrArray[0] = new TalonSRX(Constants.ID_SUPER_DELIVERY_WHEEL);
        mtrArray[1] = new TalonSRX(Constants.ID_SUPER_INDEX1);
        mtrArray[2] = new TalonSRX(Constants.ID_SUPER_INDEX2);
        mtrArray[3] = new TalonSRX(Constants.ID_SUPER_INDEX3);
        mtrArray[4] = new TalonSRX(Constants.ID_SUPER_INTAKE);

        // mtr1 = new TalonSRX(Constants.ID_SUPER_DELIVERY_WHEEL);
        // mtr2 = new TalonSRX(Constants.ID_SUPER_INDEX1);
        // mtr3 = new TalonSRX(Constants.ID_SUPER_INDEX2);
        // mtr4 = new TalonSRX(Constants.ID_SUPER_INDEX3);
        // mtr5 = new TalonSRX(Constants.ID_SUPER_INTAKE);

        sensorArray = new SimTimeOfFlight[5];
        sensorArray[0] = new SimTimeOfFlight(Constants.ID_SUPER_TOF1);
        sensorArray[1] = new SimTimeOfFlight(Constants.ID_SUPER_TOF2);
        sensorArray[2] = new SimTimeOfFlight(Constants.ID_SUPER_TOF3);
        sensorArray[3] = new SimTimeOfFlight(Constants.ID_SUPER_TOF4);
        sensorArray[4] = new SimTimeOfFlight(Constants.ID_SUPER_TOF5);
        reset();

        // sen1 = new SimTimeOfFlight(Constants.ID_SUPER_TOF1);
        // sen2 = new SimTimeOfFlight(Constants.ID_SUPER_TOF2);
        // sen3 = new SimTimeOfFlight(Constants.ID_SUPER_TOF3);
        // sen4 = new SimTimeOfFlight(Constants.ID_SUPER_TOF4);
        // sen5 = new SimTimeOfFlight(Constants.ID_SUPER_TOF5);

    }

    /**
     * Updates all periodic variables and sensors.
     */
    @Override
    public void readPeriodicInputs() {
        // periodicIO.sensorDistances[0] = sen1.getRange();
        // periodicIO.sensorDistances[1] = sen2.getRange();
        // periodicIO.sensorDistances[2] = sen3.getRange();
        // periodicIO.sensorDistances[3] = sen4.getRange();
        // periodicIO.sensorDistances[4] = sen5.getRange();

        for (int i = 0; i < periodicIO.sensorDistances.length; i++) {
            periodicIO.sensorDistances[i] = sensorArray[i].getRange();
        }

    }

    /**
     * Writes the periodic outputs to actuators (motors, etc.).
     */
    @Override
    public void writePeriodicOutputs() {
        // mtr1.set(ControlMode.PercentOutput, periodicIO.motorDemands[0]);
        // mtr2.set(ControlMode.PercentOutput, periodicIO.motorDemands[1]);
        // mtr3.set(ControlMode.PercentOutput, periodicIO.motorDemands[2]);
        // mtr4.set(ControlMode.PercentOutput, periodicIO.motorDemands[3]);
        // mtr5.set(ControlMode.PercentOutput, periodicIO.motorDemands[4]);

        for (int i = 0; i < mtrArray.length; i++) {
            mtrArray[i].set(ControlMode.PercentOutput, periodicIO.motorDemands[i]);
        }
    }

    /**
     * Registers Loops to loop while the robot is active.
     * 
     * @param enabledLooper the subsystem's Looper
     */
    @Override
    public void registerEnabledLoops(ILooper enabledLooper) {
        enabledLooper.register(new Loop() {
            @Override
            public void onLoop(double timestamp) {
                setIntaking(true);
                for (int i = 0; i < periodicIO.sensorDistances.length; i++) {
                    periodicIO.ballPresent[i] = isBallPresent(i);
                }
                if (periodicIO.robotWantDump) {
                    periodicIO.state = IndexerState.DUMP_STATE;
                } else if (periodicIO.shooterWantBall) {
                    periodicIO.state = IndexerState.SHOOT_STATE;
                    periodicIO.shooterWantBall = false;
                }
                switch (periodicIO.state) {
                    case MTR_1_TO_4_FORWARD:
                        for (int i = 0; i < periodicIO.motorDemands.length - 1; i++) {
                            periodicIO.motorDemands[i] = 1;
                        }
                        if (periodicIO.ballPresent[0]) {
                            periodicIO.state = IndexerState.MTR_2_TO_4_FORWARD;
                        }
                        break;

                    case MTR_2_TO_4_FORWARD:
                        periodicIO.motorDemands[0] = 0;
                        for (int i = 1; i < periodicIO.motorDemands.length - 1; i++) {
                            periodicIO.motorDemands[i] = 1;
                        }
                        if (periodicIO.ballPresent[1]) {
                            periodicIO.state = IndexerState.MTR_3_TO_4_FORWARD;
                        }
                        if (!periodicIO.ballPresent[0]) {
                            periodicIO.state = IndexerState.MTR_1_TO_4_FORWARD;
                        }
                        break;

                    case MTR_3_TO_4_FORWARD:
                        for (int i = 0; i < 2; i++) {
                            periodicIO.motorDemands[i] = 0;
                        }
                        for (int i = 2; i < periodicIO.motorDemands.length - 1; i++) {
                            periodicIO.motorDemands[i] = 1;
                        }
                        if (periodicIO.ballPresent[2]) {
                            periodicIO.state = IndexerState.MTR_4_FORWARD;
                        }
                        if (!periodicIO.ballPresent[1]) {
                            periodicIO.state = IndexerState.MTR_2_TO_4_FORWARD;
                        }
                        break;

                    case MTR_4_FORWARD:
                        for (int i = 0; i < 3; i++) {
                            periodicIO.motorDemands[i] = 0;
                        }
                        for (int i = 3; i < periodicIO.motorDemands.length - 1; i++) {
                            periodicIO.motorDemands[i] = 1;
                        }
                        if (periodicIO.ballPresent[3]) {
                            periodicIO.state = IndexerState.MTR_1_TO_4_OFF;
                        }
                        if (!periodicIO.ballPresent[2]) {
                            periodicIO.state = IndexerState.MTR_3_TO_4_FORWARD;
                        }
                        break;

                    // case MTR_5_FORWARD:
                    //     for (int i = 0; i < 4; i++) {
                    //         periodicIO.motorDemands[i] = 0;
                    //     }
                    //     for (int i = 4; i < periodicIO.motorDemands.length; i++) {
                    //         periodicIO.motorDemands[i] = 1;
                    //     }
                    //     if (periodicIO.ballPresent[4]) {
                    //         periodicIO.state = IndexerState.MTR_1_TO_5_OFF;
                    //     }
                    //     if (!periodicIO.ballPresent[3]) {
                    //         periodicIO.state = IndexerState.MTR_4_TO_5_FORWARD;
                    //     }
                    //     break;

                    case MTR_1_TO_4_OFF:
                        for (int i = 0; i < periodicIO.motorDemands.length - 1; i++) {
                            periodicIO.motorDemands[i] = 0;
                        }
                        if (!periodicIO.ballPresent[4]) {
                            periodicIO.state = IndexerState.MTR_4_FORWARD;
                        }
                        break;

                    case SHOOT_STATE:
                        periodicIO.motorDemands[0] = 1;
                        if (!periodicIO.ballPresent[0]) {
                            periodicIO.state = IndexerState.MTR_1_TO_4_FORWARD;
                            periodicIO.shooterWantBall = true;
                        }
                        break;

                    case DUMP_STATE:
                        for (int i = 0; i < periodicIO.motorDemands.length - 1; i++) {
                            periodicIO.motorDemands[i] = -1;
                        }
                        if (!periodicIO.robotWantDump) {
                            periodicIO.state = IndexerState.MTR_1_TO_4_FORWARD;
                        }

                        break;
                }
            }

            @Override
            public void onStart(double timestamp) {
            }

            @Override
            public void onStop(double timestamp) {
            }
        });
    }

    public void enterSixAndSeven() {
    }

    public LogData getLogger() {
        return periodicIO;
    }

    /**
     * Resets and configures the subsystem.
     */
    @Override
    public void reset() {
        periodicIO = new SuperStructureIO();
    }

    /**
     * Outputs and reads all logging information to/from the SmartDashboard.
     */
    @Override
    public void outputTelemetry() {
        SmartDashboard.putNumberArray("SuperStructure/Sensor Distances", periodicIO.sensorDistances);
        SmartDashboard.putNumberArray("SuperStructure/MotorDemand", periodicIO.motorDemands);
        SmartDashboard.putBooleanArray("SuperStructure/BallPresent", periodicIO.ballPresent);
        SmartDashboard.putString("SuperStructure/State", periodicIO.state.toString());
    }

    public enum IndexerState {
        DISABLED, MTR_1_TO_4_FORWARD, MTR_2_TO_4_FORWARD, MTR_3_TO_4_FORWARD, MTR_4_FORWARD, MTR_1_TO_4_OFF,
        SHOOT_STATE, DUMP_STATE;

        public String toString() {
            return name().charAt(0) + name().substring(1).toLowerCase();
        }
    }

    public void wantDump(boolean wantDump) {
        this.periodicIO.robotWantDump = wantDump;
    }

    public void wantBall(boolean wantBall) {
        this.periodicIO.shooterWantBall = wantBall;
    }

    public boolean isBallPresent(int id) {
        return periodicIO.sensorDistances[id] != 0 && periodicIO.sensorDistances[id] <= sensorThresholds[id];
    }

    public void setIntaking(boolean setIntaking){
        if(periodicIO.state == IndexerState.DISABLED){
            periodicIO.state = IndexerState.MTR_1_TO_4_FORWARD;
        }
        
        if (setIntaking) {
            periodicIO.motorDemands[4] = 1;
        } else {
            periodicIO.motorDemands[4] = 0;
        }
    }

    public class SuperStructureIO extends PeriodicIO {
        public double[] motorDemands = new double[5];
        public double[] sensorDistances = new double[5];
        public boolean[] ballPresent = new boolean[5];

        public IndexerState state = IndexerState.DISABLED;

        public boolean robotWantDump = false;
        public boolean shooterWantBall = false;

    }
}
