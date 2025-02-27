package frc.robot;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;

public class Constants {
    // Claw Constants
    public static class Claw {

        public static final double CLAW_PITCH_START = 0.0;
        public static final double CLAW_PITCH_INTAKE = 0.0;
        public static final double CLAW_PITCH_REEF_2 = 0.05;
        public static final double CLAW_PITCH_REEF_3 = 0.05;
        public static final double CLAW_PITCH_REEF_4 = 0.17;
        public static final double CLAW_PITCH_GETBALL = 0.54;
        public static final double CLAW_PITCH_BARGE = 0.3;
        public static final double CLAW_PITCH_ACCEPT_ERROR = 0.5;

        public static final double GET_REFF_SPEED =-20.0;
        public static final double SHOOT_REFF_SPEED =-25.0;

        public static final double GET_BALL_PIPEWHEEL_SPEED =20.0;
        public static final double GET_BALL_WHEEL_SPEED =-20.0;

        public static final double HOLD_BALL_PIPEWHEEL_SPEED =15.0;
        public static final double HOLD_BALL_WHEEL_SPEED =-15.0;

        public static final double SHOOT__BALL_PIPEWHEEL_SPEED =-100.0;
        public static final double SHOOT__BALL_WHEEL_SPEED =100.0;
    }

    // Elevator Constants
    public static class Elevator {

        public enum State {
            START, INTAKE, REEF_2, REEF_3, REEF_4, GETBALL1, GETBALL2,BARGE;
        };

        public static final double ELEVATOR_START  = -0.3;
        public static final double ELEVATOR_INTAKE = -0.3;
        public static final double ELEVATOR_REEF_2 = 3.0;
        public static final double ELEVATOR_REEF_3 = 11.0;
        public static final double ELEVATOR_REEF_4 = 25.0;
        public static final double ELEVATOR_GETBALL1 = 10.8;
        public static final double ELEVATOR_GETBALL2 = 18.8;
        public static final double ELEVATOR_BARGE  = 27.8;

        public static final double ELEVATOR_ACCEPT_ERROR = 1.0;
    }

    public static class Vision {
        // The layout of the AprilTags on the field
        public static final AprilTagFieldLayout aprilTagFieldLayout = AprilTagFieldLayout
                .loadField(AprilTagFields.k2025Reefscape);
    }

}