package frc.robot.Subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.Command;

import static edu.wpi.first.units.Units.RotationsPerSecond;
import static edu.wpi.first.units.Units.RotationsPerSecondPerSecond;
import static edu.wpi.first.units.Units.Second;

import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.CANrangeConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicTorqueCurrentFOC;
import com.ctre.phoenix6.controls.VelocityTorqueCurrentFOC;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.CANrange;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.GravityTypeValue;

public class Claw extends SubsystemBase {
    private final TalonFX m_clawPitch = new TalonFX(9, "rio");
    private final TalonFX m_clawPipeWheel = new TalonFX(10, "rio");
    private final TalonFX m_clawWheel = new TalonFX(11, "rio");

    // Test controller
    private final MotionMagicTorqueCurrentFOC m_clawPitchPositionRequest = new MotionMagicTorqueCurrentFOC(0.0)
            .withSlot(0);
    private final VelocityTorqueCurrentFOC m_clawPipeWheelVelocityRequest = new VelocityTorqueCurrentFOC(0.0)
            .withSlot(0);
    private final VelocityTorqueCurrentFOC m_clawWheelVelocityRequest = new VelocityTorqueCurrentFOC(0.0).withSlot(0);

    private final CANcoder m_clawPitchEncoder = new CANcoder(9, "rio");
    private final CANrange m_canRange = new CANrange(10, "rio");

    public Claw() {
        // claw pipe wheel configs
        var clawPipeWheelConfigs = new TalonFXConfiguration();

        clawPipeWheelConfigs.Slot0.kS = 0;// Static Friction (Velocity)
        clawPipeWheelConfigs.Slot0.kV = 0;
        clawPipeWheelConfigs.Slot0.kA = 0;
        clawPipeWheelConfigs.Slot0.kP = 0;
        clawPipeWheelConfigs.Slot0.kI = 0;
        clawPipeWheelConfigs.Slot0.kD = 0;

        m_clawPipeWheel.getConfigurator().apply(clawPipeWheelConfigs);

        // claw wheel configs
        var clawWheelConfigs = new TalonFXConfiguration();

        clawWheelConfigs.Slot0.kS = 0;// Static Friction (Velocity)
        clawWheelConfigs.Slot0.kV = 0;
        clawWheelConfigs.Slot0.kA = 0;
        clawWheelConfigs.Slot0.kP = 0;
        clawWheelConfigs.Slot0.kI = 0;
        clawWheelConfigs.Slot0.kD = 0;

        m_clawWheel.getConfigurator().apply(clawWheelConfigs);

        // claw pitch configs
        var clawPitchConfigs = new TalonFXConfiguration();

        clawPitchConfigs.Slot0.kG = 0; // Gravity
        clawPitchConfigs.Slot0.kS = 0; // Static Friction
        clawPitchConfigs.Slot0.kV = 0;
        clawPitchConfigs.Slot0.kA = 0;
        clawPitchConfigs.Slot0.kP = 0;
        clawPitchConfigs.Slot0.kI = 0;
        clawPitchConfigs.Slot0.kD = 0;
        clawPitchConfigs.Slot0.GravityType = GravityTypeValue.Arm_Cosine; // Use arm cosine gravity

        // set Motion Magic settings
        // 5 (mechanism) rotations per second cruise
        clawPitchConfigs.MotionMagic.withMotionMagicCruiseVelocity(RotationsPerSecond.of(5))
                // Take approximately 0.5 seconds to reach max vel
                .withMotionMagicAcceleration(RotationsPerSecondPerSecond.of(10))
                // Take approximately 0.1 seconds to reach max accel
                .withMotionMagicJerk(RotationsPerSecondPerSecond.per(Second).of(100));

        // set Feedback configs
        clawPitchConfigs.Feedback.FeedbackRemoteSensorID = m_clawPitchEncoder.getDeviceID();
        clawPitchConfigs.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.FusedCANcoder;
        clawPitchConfigs.Feedback.RotorToSensorRatio = 2.5; // 30:12

        m_clawPitch.getConfigurator().apply(clawPitchConfigs);

        // set CANrange configs as default
        m_canRange.getConfigurator().apply(new CANrangeConfiguration());

        // set CANcoder configs as default
        m_clawPitchEncoder.getConfigurator().apply(new CANcoderConfiguration());

    }

    /**
     * Sets the claw pitch position.
     * 
     * @param position
     */
    public void setClawPitchPosition(double position) {
        m_clawPitch.setControl(m_clawPitchPositionRequest.withPosition(position));
    }

    /**
     * Sets the claw pipe wheel velocity.
     * 
     * @param velocity
     */
    public void setClawPipeWheelVelocity(double velocity) {
        m_clawPipeWheel.setControl(m_clawPipeWheelVelocityRequest.withVelocity(velocity));
    }

    /**
     * Sets the claw wheel velocity.
     * 
     * @param velocity
     */
    public void setClawWheelVelocity(double velocity) {
        m_clawWheel.setControl(m_clawWheelVelocityRequest.withVelocity(velocity));
    }

    /**
     * Runs the claw wheel intake.
     * @return 
     */
    public Command clawWheelIntake() {
        return runEnd(() -> {
            // If the ball is not detected, run the claw wheel
            // (need to test the proximity parameter)
            if (m_canRange.getIsDetected().getValue() == false) {
                setClawPipeWheelVelocity(1.0);// work
            }
            setClawPipeWheelVelocity(0.0);// stop
        },
                () -> setClawPipeWheelVelocity(0.0));
    }

    /**
     * Runs the claw wheel outtake.
     * @return
     */
    public Command clawWheelOuttake() {
        return runEnd(() -> setClawPipeWheelVelocity(-1.0),
                () -> setClawPipeWheelVelocity(0.0));
    }

    /**
     * Runs the claw pipe wheel intake.
     * @return
     */
    public Command getBall() {
        return runEnd(() -> {
            setClawPipeWheelVelocity(0.0);
            setClawWheelVelocity(0.0);
        },
                () -> {
                    setClawPipeWheelVelocity(0.0);
                    setClawWheelVelocity(0.0);
                });
    }

    /**
     * Runs the claw wheel outtake.
     * @return
     */
    public Command shootBall() {
        return runEnd(() -> {
            setClawPipeWheelVelocity(0.0);
            setClawWheelVelocity(0.0);
        },
                () -> {
                    setClawPipeWheelVelocity(0.0);
                    setClawWheelVelocity(0.0);
                });
    }
}
