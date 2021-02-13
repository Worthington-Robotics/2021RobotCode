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
        TalonSRX mtr1;
        TalonSRX mtr2;
        TalonSRX mtr3;
        TalonSRX mtr4;
        TalonSRX mtr5;

        SimTimeOfFlight sen1;
        SimTimeOfFlight sen2;
        SimTimeOfFlight sen3;
        SimTimeOfFlight sen4;
        SimTimeOfFlight sen5;

        double[] sensorThresholds = {75, 60, 60, 75, 75};

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
        mtr1 = new TalonSRX(Constants.ID_SUPER_DELIVERY_WHEEL);
        mtr2 = new TalonSRX(Constants.ID_SUPER_INDEX1);
        mtr3 = new TalonSRX(Constants.ID_SUPER_INDEX2);
        mtr4 = new TalonSRX(Constants.ID_SUPER_INDEX3);
        mtr5 = new TalonSRX(Constants.ID_SUPER_INTAKE);

        sen1 = new SimTimeOfFlight(Constants.ID_SUPER_TOF1);
        sen2 = new SimTimeOfFlight(Constants.ID_SUPER_TOF2);
        sen3 = new SimTimeOfFlight(Constants.ID_SUPER_TOF3);
        sen4 = new SimTimeOfFlight(Constants.ID_SUPER_TOF4);
        sen5 = new SimTimeOfFlight(Constants.ID_SUPER_TOF5);

        reset();
    }

    /**
     * Updates all periodic variables and sensors.
     */
    @Override
    public void readPeriodicInputs() {
        periodicIO.sensorDistances[0] = sen1.getRange();
        periodicIO.sensorDistances[1] = sen2.getRange();
        periodicIO.sensorDistances[2] = sen3.getRange();
        periodicIO.sensorDistances[3] = sen4.getRange();
        periodicIO.sensorDistances[4] = sen5.getRange();

    }

    /**
     * Writes the periodic outputs to actuators (motors, etc.).
     */
    @Override
    public void writePeriodicOutputs() {
       mtr1.set(ControlMode.PercentOutput, periodicIO.motorDemands[0]); 
       mtr2.set(ControlMode.PercentOutput, periodicIO.motorDemands[1]);
       mtr3.set(ControlMode.PercentOutput, periodicIO.motorDemands[2]);
       mtr4.set(ControlMode.PercentOutput, periodicIO.motorDemands[3]);
       mtr5.set(ControlMode.PercentOutput, periodicIO.motorDemands[4]);
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
              for(int i = 0; i < periodicIO.sensorDistances.length; i++){
                  periodicIO.ballPresent[i] = isBallPresent(i);
              }  
                switch(periodicIO.state){
                    case STATE_0: 
                    for(int i = 0; i < periodicIO.motorDemands.length; i++){
                        periodicIO.motorDemands[i] = 1;
                    }
                    if(periodicIO.ballPresent[0]){
                        periodicIO.state = IndexerState.STATE_1;
                    }
                    break;

                    case STATE_1:
                    for(int i = 1; i < periodicIO.motorDemands.length; i++){
                        periodicIO.motorDemands[i] = 1;
                    }
                    if(periodicIO.ballPresent[1]){
                        periodicIO.state = IndexerState.STATE_2;
                    } else{
                        enterSixAndSeven();
                    }
                    break;

                    case STATE_2:
                    for(int i = 2; i < periodicIO.motorDemands.length; i++){
                        periodicIO.motorDemands[i] = 1;
                    }
                    if(periodicIO.ballPresent[2]){
                        periodicIO.state = IndexerState.STATE_3;
                    } else{
                        enterSixAndSeven();
                    }
                    break;

                    case STATE_3:
                    for(int i = 3; i < periodicIO.motorDemands.length; i++){
                        periodicIO.motorDemands[i] = 1;
                    }
                    if(periodicIO.ballPresent[3]){
                        periodicIO.state = IndexerState.STATE_4;
                    } else {
                        enterSixAndSeven();
                    }
                    break;

                    case STATE_4:
                    for(int i = 4; i < periodicIO.motorDemands.length; i++){
                        periodicIO.motorDemands[i] = 1;
                    }
                    if(periodicIO.ballPresent[4]){
                        periodicIO.state = IndexerState.STATE_5;
                    } else {
                        enterSixAndSeven();
                    }
                    break;

                    case STATE_5:
                    for(int i = 0; i < periodicIO.motorDemands.length; i++){
                        periodicIO.motorDemands[i] = 0;
                    } 
                    enterSixAndSeven();
                    break;

                    case STATE_6:
                    periodicIO.motorDemands[0] = 1;
                    if (periodicIO.ballPresent[0]){
                        periodicIO.state = IndexerState.STATE_0;
                    }
                    case STATE_7:
                    if(!periodicIO.wantDump){
                        periodicIO.state = IndexerState.STATE_0;
                    }
                }
            }

            @Override
            public void onStart(double timestamp) {
            }

            @Override
            public void onStop(double timestamp) {
            }
        }
        )
        ;
    }

    public void enterSixAndSeven(){
        if(periodicIO.wantDump){
            periodicIO.state = IndexerState.STATE_7;
        } else if(periodicIO.wantBall){
            periodicIO.state = IndexerState.STATE_6;
            periodicIO.wantBall = false;
        }
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
        SmartDashboard.putNumberArray("SuperStructure/Sensor 1", periodicIO.sensorDistances);
        SmartDashboard.putNumberArray("SuperStructure/MotorDemand", periodicIO.motorDemands);
        SmartDashboard.putString("State", periodicIO.state.toString());
    }

    public enum IndexerState{
        STATE_0, STATE_1, STATE_2, STATE_3, STATE_4, STATE_5, STATE_6, STATE_7;
        public String toString() {
            return name().charAt(0) + name().substring(1).toLowerCase();
        } 
    }

    public void wantDump(boolean wantDump){
        this.periodicIO.wantDump = wantDump;
    }

    public void wantBall(boolean wantBall){
        this.periodicIO.wantBall = true;
    }

    public boolean isBallPresent(int id){
        return periodicIO.sensorDistances[id] != 0 && periodicIO.sensorDistances[id] <= sensorThresholds[id];
    }

    public class SuperStructureIO extends PeriodicIO{
        public double[] motorDemands = new double[5];
        public double[] sensorDistances = new double[5];
        public boolean[] ballPresent = new boolean[5];
    

        public IndexerState state = IndexerState.STATE_0;

        public boolean wantDump = false;
        public boolean wantBall = false;

    }
    }

