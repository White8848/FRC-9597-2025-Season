package frc.robot.Subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.Command;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicTorqueCurrentFOC;
import com.ctre.phoenix6.controls.VelocityTorqueCurrentFOC;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.CANrange;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;

import frc.robot.Constants;

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

    private int m_clawPitchPositionFlag = 0;

    public Claw() {
        // claw pipe wheel configs
        var clawPipeWheelConfigs = new TalonFXConfiguration();

        clawPipeWheelConfigs.Slot0.kS = 0;
        clawPipeWheelConfigs.Slot0.kV = 0;
        clawPipeWheelConfigs.Slot0.kA = 0;
        clawPipeWheelConfigs.Slot0.kP = 0;
        clawPipeWheelConfigs.Slot0.kI = 0;
        clawPipeWheelConfigs.Slot0.kD = 0;

        m_clawPipeWheel.getConfigurator().apply(clawPipeWheelConfigs);

        // claw wheel configs
        var clawWheelConfigs = new TalonFXConfiguration();

        clawWheelConfigs.Slot0.kS = 0;
        clawWheelConfigs.Slot0.kV = 0;
        clawWheelConfigs.Slot0.kA = 0;
        clawWheelConfigs.Slot0.kP = 0;
        clawWheelConfigs.Slot0.kI = 0;
        clawWheelConfigs.Slot0.kD = 0;

        m_clawWheel.getConfigurator().apply(clawWheelConfigs);

        // claw pitch configs
        var clawPitchConfigs = new TalonFXConfiguration();

        clawPitchConfigs.Slot0.kS = 0;
        clawPitchConfigs.Slot0.kV = 0;
        clawPitchConfigs.Slot0.kA = 0;
        clawPitchConfigs.Slot0.kP = 0;
        clawPitchConfigs.Slot0.kI = 0;
        clawPitchConfigs.Slot0.kD = 0;

        // set Motion Magic Expo settings
        clawPitchConfigs.MotionMagic.MotionMagicCruiseVelocity = 0; // Unlimited cruise velocity
        clawPitchConfigs.MotionMagic.MotionMagicExpo_kV = 0.0; // kV is around 0.12 V/rps
        clawPitchConfigs.MotionMagic.MotionMagicExpo_kA = 0.0; // Use a slower kA of 0.1 V/(rps/s)

        // set Encoder configs
        clawPitchConfigs.Feedback.FeedbackRemoteSensorID = m_clawPitchEncoder.getDeviceID();
        clawPitchConfigs.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.RemoteCANcoder;

        m_clawPitch.getConfigurator().apply(clawPitchConfigs);
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
     * Moves the claw pitch up.
     * 
     * @return
     */
    public Command clawPitchUp() {
        return runOnce(() -> {
            m_clawPitchPositionFlag++;
            if (m_clawPitchPositionFlag > 5)
                m_clawPitchPositionFlag = 5;
            switch (m_clawPitchPositionFlag) {
                case 1:
                    setClawPitchPosition(Constants.CLAW_PITCH_INTAKE);
                    m_clawPitchEncoder.getAbsolutePosition();
                    break;
                case 2:
                    setClawPitchPosition(Constants.CLAW_PITCH_REEF_2);
                    break;
                case 3:
                    setClawPitchPosition(Constants.CLAW_PITCH_REEF_3);
                    break;
                case 4:
                    setClawPitchPosition(Constants.CLAW_PITCH_REEF_4);
                    break;
                case 5:
                    setClawPitchPosition(Constants.CLAW_PITCH_BARGE);
                    break;
            }
        });
    }

    /**
     * Moves the claw pitch down.
     * 
     * @return
     */
    public Command clawPitchDown() {
        return runOnce(() -> {
            m_clawPitchPositionFlag--;
            if (m_clawPitchPositionFlag < 0)
                m_clawPitchPositionFlag = 0;
            switch (m_clawPitchPositionFlag) {
                case 0:
                    setClawPitchPosition(Constants.CLAW_PITCH_START);
                    break;
                case 1:
                    setClawPitchPosition(Constants.CLAW_PITCH_INTAKE);
                    break;
                case 2:
                    setClawPitchPosition(Constants.CLAW_PITCH_REEF_2);
                    break;
                case 3:
                    setClawPitchPosition(Constants.CLAW_PITCH_REEF_3);
                    break;
                case 4:
                    setClawPitchPosition(Constants.CLAW_PITCH_REEF_4);
                    break;
            }
        });
    }

    public Command clawWheelIntake() {
        return runEnd(() -> setClawPipeWheelVelocity(0.0),
                () -> setClawPipeWheelVelocity(0.0));
    }

    public Command clawWheelOuttake() {
        return runEnd(() -> setClawPipeWheelVelocity(0.0),
                () -> setClawPipeWheelVelocity(0.0));
    }

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
