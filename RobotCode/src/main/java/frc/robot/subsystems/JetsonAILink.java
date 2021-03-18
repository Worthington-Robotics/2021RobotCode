package frc.robot.subsystems;

import frc.lib.geometry.Pose2d;
import frc.lib.geometry.Rotation2d;
import frc.lib.loops.ILooper;
import frc.lib.loops.Loop;
import frc.lib.socket.table.server.SocketTableData;
import frc.lib.socket.table.server.SocketTableServer;

import java.util.*;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class JetsonAILink extends Subsystem {
    private int SERVER_PORT = 5800;

    private final List<Pose2d> points = new ArrayList<>();
    private final SocketTableServer server = new SocketTableServer(SERVER_PORT);
    private final SocketTableData socketData;
    private final PoseComparator translationComparator = new PoseComparator();

    private static final JetsonAILink instance = new JetsonAILink();

    public static JetsonAILink getInstance() {
        return instance;
    }

    private JetsonAILink() {
        server.start();
        socketData = server.getData();

        SmartDashboard.putString("JetsonAILink/Data", "");
        SmartDashboard.putNumber("JetsonAILink/X", 0);
        SmartDashboard.putNumber("JetsonAILink/Y", 0);
    }

    /**
     * Updates all periodic variables and sensors
     */
    public void readPeriodicInputs() {}

    /**
     * Required for the subsystem's looper to be registered to the state machine
     * not required for subsystems that do not use looper
     *
     * @param enabledLooper the subsystem's Looper
     */
    public void registerEnabledLoops(ILooper enabledLooper) {
        enabledLooper.register(new Loop() {
            @Override public void onLoop(double timestamp) {
                points.clear();

                // Parse data
                String data = socketData.getString("balls", "");

                // Un-serialize data....
                // Will be in the format {x-coord}x{y-coord}y{x-coord}x{y-coord}y{x-coord}x{y-coord}y
                if (data.contains("x")) {
                    for (String coord : data.split("y")) {
                        String[] split = coord.split("x");
                        double x = Double.parseDouble(split[0]);
                        double y = Double.parseDouble(split[1]);

                        points.add(new Pose2d(x, y, Rotation2d.fromDegrees(Math.atan2(y, x))));
                    }
                }

                // Sort list
                points.sort(translationComparator);
            }

            @Override public void onStart(double timestamp) {}
            @Override public void onStop(double timestamp) {}
        });
    }

    /**
     * Writes the periodic outputs to actuators (motors and ect...)
     */
    public void writePeriodicOutputs() {

    }

    /**
     * Outputs all logging information to the SmartDashboard
     */
    public void outputTelemetry() {
        SmartDashboard.putString("JetsonAILink/Data", socketData.getString("balls", ""));

        Pose2d initialPose = PoseEstimator.getInstance().getLatestFieldToVehicle().getValue();
        Pose2d firstBallPose = getFirst().transformBy(initialPose);

        SmartDashboard.putNumber("JetsonAILink/X", firstBallPose.getTranslation().x());
        SmartDashboard.putNumber("JetsonAILink/Y", firstBallPose.getTranslation().y());
    }

    /**
     * Called to reset and configure the subsystem
     */
    public void reset() {

    }

    /**
     * Called to stop the autonomous functions of the subsystem and place it in open loop
     */
    public void onStop() {

    }

    public Pose2d getFirst() {
        // Return default if nothing detected on action run
        return points.isEmpty() ? new Pose2d(0, 0, Rotation2d.fromDegrees(0)) : points.get(0);
    }

    public LogData getLogger() {
        return null;
    }

    /**
     * Inheritable data class for capuring log data from the subsystems
     */
    public class AIIO extends Subsystem.PeriodicIO {

    }

    private static class PoseComparator implements Comparator<Pose2d> {
        @Override public int compare(Pose2d o1, Pose2d o2) {
            return Double.compare(o1.getTranslation().x(), o2.getTranslation().x());
        }
    }
}
