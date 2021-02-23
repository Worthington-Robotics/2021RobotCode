package frc.robot.subsystems;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color;
import frc.lib.loops.ILooper;
import frc.lib.loops.Loop;
import frc.robot.Constants;

public class Lights extends Subsystem {
    private AddressableLED mled;
    private AddressableLEDBuffer mLEDBuffer;
    private int numberOfBalls;
    private boolean targeted, intakeDown, enabled, FMSOn;
    private Color allianceColor, colorWheelColor;
    private int state = 0;
    private lightModes currentLightMode = lightModes.beforeStart;
    private Color testColor;

    private Lights() {
        mled = new AddressableLED(Constants.LED_PORT);
        mLEDBuffer = new AddressableLEDBuffer(Constants.LED_LENGTH);
        reset();
    }

    private static Lights m_lightsInstance = new Lights();

    public static Lights getInstance() {
        return m_lightsInstance;
    }

    @Override
    public void readPeriodicInputs() {
        // intakeState = Superstructure.getInstance().intakeDown();
        FMSOn = DriverStation.getInstance().isFMSAttached();
        enabled = DriverStation.getInstance().isEnabled();
        numberOfBalls = Superstructure.getInstance().getNumberOfBalls();
        targeted = Shooter.getInstance().onTarget();
        
        // Testing
        //currentLightMode = lightModes.targeting;
        //targeted = SmartDashboard.getBoolean("Lights/targetedTest", true);
        //testColor = Color.kBlue;
        //numberOfBalls = 5;

        /*if (intakeState == Value.kForward) {
            intakeDown = true;
        } else {
            intakeDown = false;
        }*/
        
        if (enabled) {
            if (numberOfBalls == -2 || numberOfBalls == 5) {
                currentLightMode = lightModes.targeting;
            } else {
                currentLightMode = lightModes.indexNum;
            }
        } else {
            if (FMSOn == true) {
                currentLightMode = lightModes.allianceColor;
                if (DriverStation.getInstance().getAlliance() == Alliance.Blue) {
                    allianceColor = Color.kFirstBlue;
                } else if (DriverStation.getInstance().getAlliance() == Alliance.Red) {
                    allianceColor = Color.kDarkRed;
                } else {
                    allianceColor = Color.kChocolate;
                } 
            } else {
                currentLightMode = lightModes.beforeStart;
            }
        } 

        //Assigning the  LEDs
        switch (currentLightMode) {
            case Testing:
                for (var i = 0; i < (int)(mLEDBuffer.getLength() * (state / 7)); i++) {
                mLEDBuffer.setLED(i, testColor);
            }
            case targeting:
                if (targeted) {
                    for (var i = 0; i < mLEDBuffer.getLength(); i++) {
                        mLEDBuffer.setRGB(i, 0, 150, 0);
                    }
                } else {
                    for (var i = 0; i < mLEDBuffer.getLength(); i++) {
                        mLEDBuffer.setRGB(i, 150, 0, 0);
                    }
                }
                // System.out.println("Lights Set");
                break;
            case indexNum:
                if (numberOfBalls >= 0 && numberOfBalls <= 4) {
                    for (var i = 0; i < mLEDBuffer.getLength(); i++) {
                        mLEDBuffer.setLED(i, Color.kBlack);
                    }
                    for (var i = 0; i < (mLEDBuffer.getLength() * (.2 * numberOfBalls)); i++) {
                        mLEDBuffer.setLED(i, Color.kYellow);
                    }
                } else {
                    for (var i = 0; i < (mLEDBuffer.getLength() * (.2 * numberOfBalls)); i++) {
                        mLEDBuffer.setHSV(i, i * (239 / mLEDBuffer.getLength()), 226, 62);
                    }
                }
                // System.out.println("Lights Set");
                break;
            case allianceColor:
                for (var i = 0; i < mLEDBuffer.getLength(); i++) {
                    mLEDBuffer.setLED(i, allianceColor);
                }
                // System.out.println("Lights Set");
                break;
            case beforeStart:
                for (var i = 0; i < (mLEDBuffer.getLength() * (.2 * numberOfBalls)); i++) {
                    mLEDBuffer.setHSV(i, i * ((239 / mLEDBuffer.getLength())), 150, 75);
                }
                break;
            default:
                for (var i = 0; i < mLEDBuffer.getLength(); i++) {
                    mLEDBuffer.setLED(i, Color.kPurple);
                }
                // System.out.println("Lights Set");
                break;
            }
    }

    @Override
    public void writePeriodicOutputs() {
        mled.setData(mLEDBuffer);
    }

    public void testLights(int state)
    {
        this.state = state;
        currentLightMode = lightModes.Testing;
    }

    @Override
    public void reset() {
        mLEDBuffer = new AddressableLEDBuffer(Constants.LED_LENGTH);
        mled.setLength(mLEDBuffer.getLength());
        mled.setData(mLEDBuffer);
        mled.start();
        currentLightMode = lightModes.allianceColor;

    }

    @Override
    public void outputTelemetry() {
        SmartDashboard.putNumber("Lights/Length of String", mLEDBuffer.getLength());
        SmartDashboard.putString("Lights/LightMode", currentLightMode.toString());
    }

    enum lightModes {
        targeting, indexNum, allianceColor, beforeStart, Testing;
    }

}
