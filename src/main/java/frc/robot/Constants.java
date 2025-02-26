package frc.robot;
import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;

public class Constants {
    // Claw Constants
    public static class Claw {
        public static final double CLAW_PITCH_START = 0.0;
        public static final double CLAW_PITCH_INTAKE = 0.0;
        public static final double CLAW_PITCH_REEF_2 = 0.0;
        public static final double CLAW_PITCH_REEF_3 = 0.0;
        public static final double CLAW_PITCH_REEF_4 = 0.0;
        public static final double CLAW_PITCH_BARGE = 0.0;
        
    }

    // Elevator Constants
    public static class Elevator {
        public static final double ELEVATOR_START = 0.0;
        public static final double ELEVATOR_INTAKE = 0.0;
        public static final double ELEVATOR_REEF_2 = 0.0;
        public static final double ELEVATOR_REEF_3 = 0.0;
        public static final double ELEVATOR_REEF_4 = 0.0;
        public static final double ELEVATOR_BARGE = 0.0;
    }

    public static class Vision{
    // The layout of the AprilTags on the field
    public static final AprilTagFieldLayout aprilTagFieldLayout =AprilTagFieldLayout.loadField(AprilTagFields.k2025Reefscape);
    }



    
}