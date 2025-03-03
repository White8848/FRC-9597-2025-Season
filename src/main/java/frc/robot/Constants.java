package frc.robot;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.wpilibj.XboxController;

public class Constants {
    //candel Constants
    public static class CANDLE{

        public static final int CANdleID1 = 1;
        public static final int CANdleID2 = 2;
        public static final int JoystickId = 0;
        public static final int IncrementAnimButton = XboxController.Button.kRightBumper.value;
        public static final int DecrementAnimButton = XboxController.Button.kLeftBumper.value;
        public static final int BlockButton = XboxController.Button.kStart.value;
        public static final int MaxBrightnessAngle = 90;
        public static final int MidBrightnessAngle = 180;
        public static final int ZeroBrightnessAngle = 270;
        public static final int VbatButton = XboxController.Button.kA.value;
        public static final int V5Button = XboxController.Button.kB.value;
        public static final int CurrentButton = XboxController.Button.kX.value;
        public static final int TemperatureButton = XboxController.Button.kY.value;

    }
    
    // Claw Constants
    public static class Claw {

        //position
        public static final double CLAW_PITCH_START = 0.0;
        public static final double CLAW_PITCH_REEF_2 = 0.05;
        public static final double CLAW_PITCH_REEF_3 = 0.05;
        public static final double CLAW_PITCH_REEF_4 = 0.17;
        public static final double CLAW_PITCH_GETBALL = 0.54;
        public static final double CLAW_PITCH_BARGE = 0.3;
        public static final double CLAW_PITCH_AVOID_COLLISION1 = 0.03;//from start to any position,advoid collision
        public static final double CLAW_PITCH_AVOID_COLLISION2 = 0.15;//from barge to start,advoid collision
        public static final double CLAW_PITCH_ACCEPT_ERROR = 0.05;


        //speed
        public static final double GET_REFF_SPEED =10.0;
        public static final double SHOOT_REFF_SPEED =25.0;
        public static final double GET_BALL_PIPEWHEEL_SPEED =-50.0;
        public static final double GET_BALL_WHEEL_SPEED =50.0;
        public static final double HOLD_BALL_PIPEWHEEL_SPEED =-50.0;
        public static final double HOLD_BALL_WHEEL_SPEED =50.0;
        public static final double SHOOT__BALL_PIPEWHEEL_SPEED =+100.0;
        public static final double SHOOT__BALL_WHEEL_SPEED =-100.0;
    }

    // Elevator Constants
    public static class Elevator {

        public enum State {
            START, INTAKE, REEF_2, REEF_3, REEF_4, GETBALL1, GETBALL2, BARGE, FLOW;
        };

        public static final double ELEVATOR_START  = -0.3;
        public static final double ELEVATOR_REEF_2 = 5.0;
        public static final double ELEVATOR_REEF_3 = 13.0;
        public static final double ELEVATOR_REEF_4 = 26.0;
        public static final double ELEVATOR_GETBALL1 = 10.8;
        public static final double ELEVATOR_GETBALL2 = 18.8;
        public static final double ELEVATOR_BARGE  = 28.2;
        public static final double ELEVATOR_ShALLOWCAGE_UP  = 10.0;
        public static final double ELEVATOR_ShALLOWCAGE_DOWN  = 2.0;

        public static final double ELEVATOR_ACCEPT_ERROR = 2.0;
    }

    public static class DEEPCAGE {
        public static final double INTAKE_BALL_POSITION1=45.0;
        public static final double INTAKE_BALL_POSITION2=25.0;
        public static final double INTAKE_BALL_SPEED=8.0;
        public static final double HOLD_BALL_SPEED=5.0;
        public static final double EJECT_BALL_SPEED=-20.0;

        public static final double PITCH_UP_SPEED=5.0;
        public static final double PITCH_DOWN_SPEED=-5.0;

    }


    public static class Vision {
        // The layout of the AprilTags on the field
        public static final AprilTagFieldLayout aprilTagFieldLayout = AprilTagFieldLayout
                .loadField(AprilTagFields.k2025Reefscape);
    }

}