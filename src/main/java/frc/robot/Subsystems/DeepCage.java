package frc.robot.Subsystems;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.PositionTorqueCurrentFOC;
import com.ctre.phoenix6.controls.VelocityTorqueCurrentFOC;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class DeepCage extends SubsystemBase {
    private final TalonFX m_intakeWheel = new TalonFX(14, "rio");
    private final TalonFX m_deepCagePitch = new TalonFX(15, "rio");

    private final VelocityTorqueCurrentFOC m_intakeWheelVelocityRequest = new VelocityTorqueCurrentFOC(0.0)
            .withSlot(0);
    private final VelocityTorqueCurrentFOC m_deepCagePitchVelocityRequest = new VelocityTorqueCurrentFOC(0.0)
            .withSlot(0);
    private final PositionTorqueCurrentFOC m_deepCagePitchPositionRequest = new PositionTorqueCurrentFOC(0.0)
            .withSlot(1);

    public DeepCage() {
        var intakeWheelConfigs = new TalonFXConfiguration();

        intakeWheelConfigs.Slot0.kS = 0;
        intakeWheelConfigs.Slot0.kV = 0;
        intakeWheelConfigs.Slot0.kA = 0;
        intakeWheelConfigs.Slot0.kP = 0;
        intakeWheelConfigs.Slot0.kI = 0;
        intakeWheelConfigs.Slot0.kD = 0;

        m_intakeWheel.getConfigurator().apply(intakeWheelConfigs);

        var deepCagePitchConfigs = new TalonFXConfiguration();
        //Velocity Torque Current FOC
        deepCagePitchConfigs.Slot0.kS = 0;
        deepCagePitchConfigs.Slot0.kV = 0;
        deepCagePitchConfigs.Slot0.kA = 0;
        deepCagePitchConfigs.Slot0.kP = 0;
        deepCagePitchConfigs.Slot0.kI = 0;
        deepCagePitchConfigs.Slot0.kD = 0;
        //Position Torque Current FOC
        deepCagePitchConfigs.Slot1.kS = 0;
        deepCagePitchConfigs.Slot1.kV = 0;
        deepCagePitchConfigs.Slot1.kA = 0;
        deepCagePitchConfigs.Slot1.kP = 0;
        deepCagePitchConfigs.Slot1.kI = 0;
        deepCagePitchConfigs.Slot1.kD = 0;

        m_deepCagePitch.getConfigurator().apply(deepCagePitchConfigs);
    }

    /**
     * Sets the intake wheel velocity
     * 
     * @param velocity
     */
    public void setIntakeWheelVelocity(double velocity) {
        m_intakeWheel.setControl(m_intakeWheelVelocityRequest.withVelocity(velocity));
    }

    /**
     * Sets the deep cage pitch velocity
     * 
     * @param velocity
     */
    public void setDeepCagePitchVelocity(double velocity) {
        m_deepCagePitch.setControl(m_deepCagePitchVelocityRequest.withVelocity(velocity));
    }

    /**
     * Sets the deep cage pitch position
     * 
     * @param position
     */
    public void setDeepCagePitchPosition(double position) {
        m_deepCagePitch.setControl(m_deepCagePitchPositionRequest.withPosition(position));
    }

    public Command intakeBall() {
        return runEnd(() -> {
            setIntakeWheelVelocity(0.5);
        }, () -> {
            setIntakeWheelVelocity(0);
        });
    }

    public Command ejectBall() {
        return runEnd(() -> {
            setIntakeWheelVelocity(-0.5);
        }, () -> {
            setIntakeWheelVelocity(0);
        });
    }

    public Command deepCagePitchUp() {
        return runEnd(() -> {
            setDeepCagePitchVelocity(0.5);
        }, () -> {
            setDeepCagePitchVelocity(0);
            double currentPosition = m_deepCagePitch.getPosition().getValueAsDouble();
            setDeepCagePitchPosition(currentPosition);
        });
    }

    public Command deepCagePitchDown() {
        return runEnd(() -> {
            setDeepCagePitchVelocity(-0.5);
        }, () -> {
            setDeepCagePitchVelocity(0);
            double currentPosition = m_deepCagePitch.getPosition().getValueAsDouble();
            setDeepCagePitchPosition(currentPosition);
        });
    }
    

}
