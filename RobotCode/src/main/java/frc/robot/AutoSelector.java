package frc.robot;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.lib.statemachine.StateMachineDescriptor;
import frc.robot.autoactiongroups.*;
import frc.robot.subsystems.JetsonAILink;

/**
 * @author Cole Tucker This enum encompasses all user selectable autonomous
 * commands.
 * <p>
 * It has two parameters, a visible name and an associated ID number
 * from 1 to n.
 */
enum UserSelection {
    Auto1("ShootyAuto", 1),
    Auto2("G3SG1-All", 2),
    Auto3("Ski", 3),
    Auto4("Barrel", 4),
    Auto5("Bounce", 5),
    Auto6("Three By Three", 6),
    Auto20("Remote Operation", 20);

    private String name;
    private int num;

    UserSelection(String name, int num) {
        this.name = name;
        this.num = num;
    }

    @Override
    public String toString() {
        return name;
    }

    public int getNum() {
        return num;
    }

}

public class AutoSelector {

    /**
     * Method to get an array of names for all selections
     *
     * @return string array of enum names
     */
    public static String[] buildArray() {
        String[] out = new String[UserSelection.values().length];
        for (int i = 0; i < UserSelection.values().length; i++) {
            out[i] = UserSelection.values()[i].toString();
        }
        return out;
    }

    /**
     * Gets the enum object that matches the fed string.
     *
     * @param name checked against all possible enum name parameters
     * @return The internally used enum for auto calculations
     */
    private static UserSelection getSelFromStr(String name) {
        for (UserSelection sel : UserSelection.values()) {
            if (sel.toString().equalsIgnoreCase(name)) {
                return sel;
            }
        }
        // return the last possible enum by default
        return UserSelection.values()[UserSelection.values().length - 1];
    }


    /**
     * This method determines the Auto mode based on the fed game data and the
     * dashboard data.
     *
     * @param selection - a string with the name of the selected autonomous.
     * @return the proper auto command to run. It should include all movements in
     * one command
     */
    public static StateMachineDescriptor autoSelect(String selection) {
        UserSelection usrAuto = getSelFromStr(selection);
        SmartDashboard.putString("Final Auto Choice", usrAuto.toString());
        
        switch (usrAuto) {
            case Auto1:
                return new EightBallAuto();
            case Auto2:
                switch (JetsonAILink.getInstance().getAuto()) {
                    case RED_1:
                        return new GalacticSearchA();
                    case RED_2:
                        return new GalacticSearchB();
                    case BLUE_1:
                        return new GalacticSearchC();
                    case BLUE_2:
                        return new GalacticSearchD();
                    default:
                        return null;
                }
            case Auto3:
                return new NoSkew();
            case Auto4:
                return new Barrel();
            case Auto5:
                return new BoingBoing();
            default:
                return null;
        }
    }
}