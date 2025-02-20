package frc.robot.Subsystems;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.MotionMagicTorqueCurrentFOC;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.GravityTypeValue;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Elevator extends SubsystemBase {
    private final TalonFX m_elevatorLeft = new TalonFX(12, "rio");
    private final TalonFX m_elevatorRight = new TalonFX(13, "rio");

    private final MotionMagicTorqueCurrentFOC m_elevatorPositionRequest = new MotionMagicTorqueCurrentFOC(0.0)
            .withSlot(0);

    public Elevator() {
        var elevatorConfigs = new TalonFXConfiguration();

        elevatorConfigs.Slot0.kG = 0;
        elevatorConfigs.Slot0.kS = 0;
        elevatorConfigs.Slot0.kV = 0;
        elevatorConfigs.Slot0.kA = 0;
        elevatorConfigs.Slot0.kP = 0;
        elevatorConfigs.Slot0.kI = 0;
        elevatorConfigs.Slot0.kD = 0;
        elevatorConfigs.Slot0.GravityType = GravityTypeValue.Elevator_Static; 

        // set Motion Magic Expo settings
        elevatorConfigs.MotionMagic.MotionMagicCruiseVelocity = 0; // Unlimited cruise velocity
        elevatorConfigs.MotionMagic.MotionMagicExpo_kV = 0.0; // kV is around 0.12 V/rps
        elevatorConfigs.MotionMagic.MotionMagicExpo_kA = 0.0; // Use a slower kA of 0.1 V/(rps/s)

        m_elevatorLeft.getConfigurator().apply(elevatorConfigs);

        // Set the right motor to follow the left motor  
        m_elevatorRight.setControl(new Follower(m_elevatorLeft.getDeviceID(), false));
    }

    public void setElevatorPosition(double position) {
        m_elevatorLeft.setControl(m_elevatorPositionRequest.withPosition(position));
    }

}
