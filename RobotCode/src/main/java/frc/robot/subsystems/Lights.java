package frc.robot.subsystems;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color;
import frc.robot.Constants;

public class Lights extends Subsystem {
    private final double RAINBOW_BREATHE_SPEED = 2;

    private AddressableLED mled;
    private AddressableLEDBuffer mLEDBuffer;
    private int numberOfBalls;
    private boolean targeted, targeting, enabled, RobotConnectedToFMS;
    private Color allianceColor;
    private int state = 0;
    private LightMode currentLightMode = LightMode.RAINBOW;
    private Color testColor;
    private double rainbowint = 0;
    private int lightMode = 0;

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
        RobotConnectedToFMS = DriverStation.getInstance().isFMSAttached();
        enabled = DriverStation.getInstance().isEnabled();
        numberOfBalls = Superstructure.getInstance().getNumberOfBalls();
        targeting = Shooter.getInstance().getUsingLimelight();
        targeted = Shooter.getInstance().onTarget();
        
        // Testing
        //currentLightMode = lightModes.targeting;
        //targeted = SmartDashboard.getBoolean("Lights/targetedTest", true);
        //testColor = Color.kBlue;
        if (DriverStation.getInstance().getAlliance() == Alliance.Blue) {
            allianceColor = Color.kFirstBlue;
        } else if (DriverStation.getInstance().getAlliance() == Alliance.Red) {
            allianceColor = Color.kDarkRed;
        } else {
            allianceColor = Color.kChocolate;
        }

        if(lightMode == 1) {
            currentLightMode = LightMode.BATTERY;
        } else if(lightMode == 2){
            currentLightMode = LightMode.TEMPERATURE;
        } else {
               
            if (enabled) {
                if (targeting) {
                    currentLightMode = LightMode.TARGETING;
                } else {
                    currentLightMode = LightMode.RAINBOW;
                }
            } else {
                if (RobotConnectedToFMS) {
                    currentLightMode = LightMode.ALLIANCE_COLOR;
                    
                } else {
                    currentLightMode = LightMode.RAINBOW_BREATHE;
                }
            } 
        }
        
        //currentLightMode = lightModes.rainbow;
        //if (Constants.DEBUG) {currentLightMode = lightModes.allianceColor;}

        //Assigning the  LEDs
        switch (currentLightMode) {
            case TESTING: {
                for (int i = 0; i < (int)(mLEDBuffer.getLength() * (state / 7)); i++) {
                    mLEDBuffer.setLED(i, testColor);
                }
            }
            case TARGETING: {
                if (targeted) {
                    for (int i = 0; i < mLEDBuffer.getLength(); i++) {
                        mLEDBuffer.setRGB(i, 0, 150, 0);
                    }
                } else {
                    for (int i = 0; i < mLEDBuffer.getLength(); i++) {
                        mLEDBuffer.setRGB(i, 150, 0, 0);
                    }
                }
                // System.out.println("Lights Set");
                break;
            }
            case INDEX_NUM: {
                if (numberOfBalls >= 0 && numberOfBalls <= 4) {
                    for (int i = 0; i < mLEDBuffer.getLength(); i++) {
                        mLEDBuffer.setLED(i, Color.kBlack);
                    }
                    for (int i = 0; i < (mLEDBuffer.getLength() * (.2 * numberOfBalls)); i++) {
                        mLEDBuffer.setLED(i, Color.kYellow);
                    }
                } else {
                    for (int i = 0; i < (mLEDBuffer.getLength() * (.2 * numberOfBalls)); i++) {
                        mLEDBuffer.setHSV(i, i * (300 / mLEDBuffer.getLength()), 226, 62);
                    }
                }
                // System.out.println("Lights Set");
                break;
            }
            case ALLIANCE_COLOR: {
                for (int i = 0; i < mLEDBuffer.getLength(); i++) {
                    mLEDBuffer.setLED(i, allianceColor);
                }
                // System.out.println("Lights Set");
                break;
            }
            case RAINBOW: {
                int ledlen = mLEDBuffer.getLength();
                for (int i = 0; i < (ledlen); i ++) {
                    mLEDBuffer.setHSV((i + (int)(rainbowint)) % ledlen, i * ((180 / ledlen)), 255, 200);
                }
                rainbowint += Math.abs(Drive.getInstance().getLinearVelocity()*.7);
                break;
            }
            case BATTERY: {
                for (int i = 0; i < Math.min(mLEDBuffer.getLength(), (mLEDBuffer.getLength() * ((RobotController.getBatteryVoltage()-10)/3))); i++) {
                    mLEDBuffer.setHSV(i, (int)(70 * ((RobotController.getBatteryVoltage()-10)/3) + 165) % 180, 255, 200);
                }
                for (int i = Math.max((int)(mLEDBuffer.getLength() * ((RobotController.getBatteryVoltage()-10)/3)), 0); i < mLEDBuffer.getLength(); i++) {
                    mLEDBuffer.setLED(i, new Color(0,0,0));
                }  
                break;
            }
            case TEMPERATURE: {
                for (int i = 0; i < Math.min(mLEDBuffer.getLength(), (mLEDBuffer.getLength() * ((Drive.getInstance().getTemperature()-30)/40))); i++) {
                    mLEDBuffer.setHSV(i, (int)(90 + 110 * ((Drive.getInstance().getTemperature()-30)/40)) % 180, 255, 200);
                }
                for (int i = Math.max((int)(mLEDBuffer.getLength() * ((Drive.getInstance().getTemperature()-30)/40)), 0); i < mLEDBuffer.getLength(); i++) {
                    mLEDBuffer.setLED(i, new Color(0,0,0));
                }

                break;
            }
            case RAINBOW_BREATHE: {
                int ledlen = mLEDBuffer.getLength();
                for (int i = 0; i < (ledlen); i ++) {
                    mLEDBuffer.setHSV((i + (int)(rainbowint)) % ledlen, i * ((180 / ledlen)), 255, 200);
                }
                rainbowint += Math.abs(RAINBOW_BREATHE_SPEED*.7); // Rainbow changing constant

                break;
            }   
            default: {
                for (var i = 0; i < mLEDBuffer.getLength(); i++) {
                    mLEDBuffer.setLED(i, Color.kPurple);
                }
                // System.out.println("Lights Set");
                break;
            }
        }
    }

    public void nextLight() {
        lightMode ++;
        lightMode %= 3;
    }

    @Override
    public void writePeriodicOutputs() {
        mled.setData(mLEDBuffer);
    }

    public void testLights(int state)
    {
        this.state = state;
        currentLightMode = LightMode.TESTING;
    }

    @Override
    public void reset() {
        mLEDBuffer = new AddressableLEDBuffer(Constants.LED_LENGTH);
        mled.setLength(mLEDBuffer.getLength());
        mled.setData(mLEDBuffer);
        mled.start();
        currentLightMode = LightMode.ALLIANCE_COLOR;

    }

    @Override
    public void outputTelemetry() {
        SmartDashboard.putNumber("Lights/Length of String", mLEDBuffer.getLength());
        SmartDashboard.putString("Lights/LightMode", currentLightMode.toString());
        SmartDashboard.putNumber("Lights/LightNum", lightMode);
    }

    enum LightMode {
        TARGETING, INDEX_NUM, ALLIANCE_COLOR, RAINBOW, RAINBOW_BREATHE, TESTING, BATTERY, TEMPERATURE;
    }

}
