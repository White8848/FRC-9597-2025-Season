package frc.robot.Subsystems;

import com.ctre.phoenix6.SignalLogger;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.signals.InvertedValue;

import static edu.wpi.first.units.Units.Second;
import static edu.wpi.first.units.Units.Volts;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;

public class Elevator extends SubsystemBase{ 

    private final TalonFX m_elevatorLeft = new TalonFX(12, "rio");
    private final TalonFX m_elevatorRight = new TalonFX(13, "rio");

    //CONTROL MODE
    private final MotionMagicVoltage m_elevatorPositionRequest = new MotionMagicVoltage(0.0)
    .withSlot(0);

    //sysidroutine need voltageout mode
    private final VoltageOut m_elevator_leftVoltageRequest_sysid = new VoltageOut(0.0);
    private final VoltageOut m_elevator_rightVoltageRequest_sysid = new VoltageOut(0.0);
   
    /* SysId routine for characterizing clawpitch. This is used to find PID gains for the motors. */
    private final SysIdRoutine m_sysIdRoutineElevator = new SysIdRoutine(
        new SysIdRoutine.Config(
            Volts.per(Second).of(0.5),
            //null,        // Use default ramp rate (1 V/s)
            Volts.of(1.2), // Reduce dynamic step voltage to 4 V to prevent brownout
            null,        // Use default timeout (10 s)
            // Log state with SignalLogger class
            state -> SignalLogger.writeString("SysIdElevator_State", state.toString())
        ),
        new SysIdRoutine.Mechanism(
            output -> {m_elevatorLeft.setControl(m_elevator_leftVoltageRequest_sysid.withOutput(output));
                        m_elevatorRight.setControl(m_elevator_rightVoltageRequest_sysid.withOutput(output));},
            null,
            this
        )
    );


    public Elevator() {
        var elevatorConfigs = new TalonFXConfiguration();

        elevatorConfigs.Slot0.kG = 0.48;
        elevatorConfigs.Slot0.kS = 0;
        elevatorConfigs.Slot0.kV = 0;
        elevatorConfigs.Slot0.kA = 0;
        elevatorConfigs.Slot0.kP = 2.0;
        elevatorConfigs.Slot0.kI = 0;
        elevatorConfigs.Slot0.kD = 0;
        elevatorConfigs.Slot0.GravityType = GravityTypeValue.Elevator_Static; 
        elevatorConfigs.MotorOutput.Inverted =InvertedValue.CounterClockwise_Positive;

        // set Motion Magic Expo settings
        elevatorConfigs.MotionMagic.MotionMagicAcceleration = 40; 
        elevatorConfigs.MotionMagic.MotionMagicCruiseVelocity = 20; 
        elevatorConfigs.MotionMagic.MotionMagicExpo_kV = 0.12; 
        elevatorConfigs.MotionMagic.MotionMagicExpo_kA = 0.1; 
        elevatorConfigs.MotionMagic.MotionMagicJerk=0;

        m_elevatorLeft.getConfigurator().apply(elevatorConfigs);

        // Set the right motor to follow the left motor  
        m_elevatorRight.setControl(new Follower(m_elevatorLeft.getDeviceID(), false));
    }

    public void setElevatorPosition(double position) {
        m_elevatorLeft.setControl(m_elevatorPositionRequest.withPosition(position));
    }

    public Command MoveToLevel(double position)
    {
      return run(() -> this.setElevatorPosition(position));
    }



    /**
     * Runs the SysId Quasistatic test in the given direction for the routine
     * specified by {@link #m_sysIdRoutineToApply}.
     *
     * @param direction Direction of the SysId Quasistatic test
     * @return Command to run
     */
    public Command sysIdQuasistatic(SysIdRoutine.Direction direction) {
        return m_sysIdRoutineElevator.quasistatic(direction);
    }

    /**
     * Runs the SysId Dynamic test in the given direction for the routine
     * specified by {@link #m_sysIdRoutineToApply}.
     *
     * @param direction Direction of the SysId Dynamic test
     * @return Command to run
     */
    public Command sysIdDynamic(SysIdRoutine.Direction direction) {
        return m_sysIdRoutineElevator.dynamic(direction);
    }


}
