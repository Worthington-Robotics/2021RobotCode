package frc.lib.util;

import edu.wpi.first.wpilibj.Filesystem;
import frc.lib.exceptions.DirectoryNotFoundException;

import java.io.File;
import java.io.FileNotFoundException;
import java.lang.reflect.Field;
import java.nio.file.Files;
import java.nio.file.Path;
import java.nio.file.Paths;

public class TestLogger<T> extends ReflectingLogger<T> {

    /**
     * A reflection based logger to use in unit tests, it uses the same formatting
     * as the Reflecting logger but can only intake a single data class unlike its
     * bigger brother. It also stores the captured logs into the src/main/sim/test
     * directory for manipulation purposes
     * 
     * @param dataClass the data class to intake
     * @param testName  the name of the test when saving the file
     */
    public TestLogger(T dataClass, String testName) {
        // generate map of fields
        for (Field field : dataClass.getClass().getFields()) {
            classFieldMap.put(field, dataClass);
        }

        // create the base file and header
        try {
            generateHeader(getMount(testName), false);
        } catch (DirectoryNotFoundException e) {
            e.printStackTrace();
        }

    }

    /**
     * Function that writes the next update to the log file.
     * It reads a list of data classes and dumps their contents into the opened file
     * 
     * @param dataClasses the list of data classes to log
     * @param fpgaTimestamp the current FPGA time of the system to record
     */
    public void update(T dataClass, double fpgaTimestamp) {

        //generate map of fields
        for (Field field : dataClass.getClass().getFields()) {
            classFieldMap.put(field, dataClass);
        }

        logMap(fpgaTimestamp);
    }

    /**
     * a function to get acccess to the simulation directory
     * 
     * @param fileName general name of the file to generate with
     * @return a file reference to the created logging file in the usb drive's
     *         logging folder
     * @throws FileNotFoundException if a logging directory is not found a
     *                               DirectoryNotFoundException will be thrown
     */
    public static File getMount(String fileName) throws DirectoryNotFoundException {
        Path path = Paths.get(Filesystem.getLaunchDirectory() + File.separator + "src" + File.separator + "main"
                + File.separator + "sim" + File.separator + "test");
        if (!Files.exists(path)) {
            try {
                Files.createDirectories(path);
            } catch (Exception e) {
                throw new DirectoryNotFoundException(path.toString());
            }

        }
        return new File(path.toString() + File.separator + fileName + ".csv");
    }

}