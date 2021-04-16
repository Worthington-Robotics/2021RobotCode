package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.lib.loops.ILooper;
import frc.lib.loops.Loop;
import frc.lib.util.Util;
import frc.robot.Constants;

public class Climber extends Subsystem {
    private DoubleSolenoid unfoldSolenoid, climbSolenoid;

    private Value unfoldIntendedState = Value.kReverse, climbIntendedState = Value.kForward;
  
    

    private double talonRealeseDemand;

    private TalonFX talonRealease;




    private Climber() {
        unfoldSolenoid = new DoubleSolenoid(Constants.UNFOLD_LOW_ID, Constants.UNFOLD_HIGH_ID);
        climbSolenoid = new DoubleSolenoid(Constants.CLIMB_LOW_ID, Constants.CLIMB_HIGH_ID);
        reset();

        talonRealease = new TalonFX(Constants.SOLIDNOID);
        //SmartDashboard.putNumber("turretsim", 0);
    }

    private static Climber mClimber = new Climber();

    public static Climber getInstance() {
        return mClimber;
    }

    @Override
    public void readPeriodicInputs() {
        
    }

    public void registerEnabledLoops(ILooper enabledLooper) {
        enabledLooper.register(new Loop() {

            @Override
            public void onStart(double timestamp) {
                // TODO Auto-generated method stub

            }

            @Override
            public void onLoop(double timestamp) {
                               
            }

            @Override
            public void onStop(double timestamp) {
                // TODO Auto-generated method stub

            }

        });
    }

    
    public void setFirstPiston(boolean raised) {
        if(raised) {
            climbIntendedState = Value.kForward;
        }
         else {
            climbIntendedState = Value.kReverse;
            }
    }

    public void setRealease() {
        climbIntendedState = Value.kReverse;
    }

    public void setMotorPower(double d) {
        talonRealeseDemand = d;
    }

    @Override
    public void writePeriodicOutputs() {
        climbSolenoid.set(climbIntendedState);
        unfoldSolenoid.set(unfoldIntendedState);
        talonRealease.set(ControlMode.PercentOutput, talonRealeseDemand);
    }


    @Override
    public void outputTelemetry() {
    }

    @Override
    public void reset() {
        unfoldSolenoid.set(Value.kReverse);
        climbSolenoid.set(Value.kForward);
    }
}
