package frc.robot.Subsystems;

import com.ctre.phoenix6.SignalLogger;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVelocityVoltage;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.VelocityTorqueCurrentFOC;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.GravityTypeValue;

import static edu.wpi.first.units.Units.Volts;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;

public class DeepCage extends SubsystemBase {
    private final TalonFX m_intakeWheel = new TalonFX(14, "rio");
    private final TalonFX m_deepCagePitch = new TalonFX(15, "rio");

    private final MotionMagicVelocityVoltage m_intakeWheelVelocityRequest = new MotionMagicVelocityVoltage(0.0)
            .withSlot(0);
    private final MotionMagicVelocityVoltage m_deepCagePitchVelocityRequest = new MotionMagicVelocityVoltage(0.0)
            .withSlot(0);
    private final MotionMagicVoltage m_deepCagePitchPositionRequest = new MotionMagicVoltage(0.0)
            .withSlot(1);

    //sysidoutine need voltageout mode
    private final VoltageOut m_intakeWheelVoltageRequest_sysid = new VoltageOut(0.0);
    private final VoltageOut m_deepCagePitchVoltageRequest_sysid = new VoltageOut(0.0);
 
    /* SysId routine for characterizing intakewheel. This is used to find PID gains for the motors. */
    private final SysIdRoutine m_sysIdRoutineIntakeWheel = new SysIdRoutine(
        new SysIdRoutine.Config(
            null,        // Use default ramp rate (1 V/s)
            Volts.of(4), // Reduce dynamic step voltage to 4 V to prevent brownout
            null,        // Use default timeout (10 s)
            // Log state with SignalLogger class
            state -> SignalLogger.writeString("SysIdIntakeWheel_State", state.toString())
        ),
        new SysIdRoutine.Mechanism(
            output -> m_intakeWheel.setControl(m_intakeWheelVoltageRequest_sysid.withOutput(output)),
            null,
            this
        )
    );

    /* SysId routine for characterizing deepCagePitch. This is used to find PID gains for the  motors. */
    private final SysIdRoutine m_sysIdRoutineDeepCagePitch = new SysIdRoutine(
        new SysIdRoutine.Config(
            null,        // Use default ramp rate (1 V/s)
            Volts.of(4), // Reduce dynamic step voltage to 4 V to prevent brownout
            null,        // Use default timeout (10 s)
            // Log state with SignalLogger class
            state -> SignalLogger.writeString("SysIdDeepCagePitch_State", state.toString())
        ),
        new SysIdRoutine.Mechanism(
            output -> m_deepCagePitch.setControl(m_deepCagePitchVoltageRequest_sysid.withOutput(output)),
            null,
            this
        )
    );

    private SysIdRoutine m_sysIdRoutineToApply = m_sysIdRoutineIntakeWheel;


    public DeepCage() {
        //IntakeWheel configs
        var intakeWheelConfigs = new TalonFXConfiguration();

        intakeWheelConfigs.Slot0.kS = 0.48;
        intakeWheelConfigs.Slot0.kV = 0.1;
        intakeWheelConfigs.Slot0.kA = 0;
        intakeWheelConfigs.Slot0.kP = 0.5;
        intakeWheelConfigs.Slot0.kI = 0;
        intakeWheelConfigs.Slot0.kD = 0;


        // set Motion Magic Expo settings
        intakeWheelConfigs.MotionMagic.MotionMagicAcceleration = 20; // Acceleration is around 40 rps/s
        intakeWheelConfigs.MotionMagic.MotionMagicCruiseVelocity = 40; // Unlimited cruise velocity
        intakeWheelConfigs.MotionMagic.MotionMagicExpo_kV = 0.12; // kV is around 0.12 V/rps
        intakeWheelConfigs.MotionMagic.MotionMagicExpo_kA = 0.1; // Use a slower kA of 0.1 V/(rps/s)
        intakeWheelConfigs.MotionMagic.MotionMagicJerk = 0; // Jerk is around 0

        m_intakeWheel.getConfigurator().apply(intakeWheelConfigs);

        //DeepCagePitch configs
        var deepCagePitchConfigs = new TalonFXConfiguration();

        deepCagePitchConfigs.Slot0.kS = 0.5;
        deepCagePitchConfigs.Slot0.kV = 0.5;
        deepCagePitchConfigs.Slot0.kA = 0;
        deepCagePitchConfigs.Slot0.kP = 20;
        deepCagePitchConfigs.Slot0.kI = 0;
        deepCagePitchConfigs.Slot0.kD = 0;
        deepCagePitchConfigs.Slot0.kG = 0.25;
        deepCagePitchConfigs.Slot0.withGravityType(GravityTypeValue.Arm_Cosine);

        // set Motion Magic Expo settings
        deepCagePitchConfigs.MotionMagic.MotionMagicAcceleration = 20; // Acceleration is around 40 rps/s
        deepCagePitchConfigs.MotionMagic.MotionMagicCruiseVelocity = 40; // Unlimited cruise velocity
        deepCagePitchConfigs.MotionMagic.MotionMagicExpo_kV = 0.15; // kV is around 0.12 V/rps
        deepCagePitchConfigs.MotionMagic.MotionMagicExpo_kA = 0.1; // Use a slower kA of 0.1 V/(rps/s)
        deepCagePitchConfigs.MotionMagic.MotionMagicJerk = 0; // Jerk is around 0

     
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

    
    /**
     * Runs the SysId Quasistatic test in the given direction for the routine
     * specified by {@link #m_sysIdRoutineToApply}.
     *
     * @param direction Direction of the SysId Quasistatic test
     * @return Command to run
     */
    public Command sysIdQuasistatic(SysIdRoutine.Direction direction) {
        return m_sysIdRoutineToApply.quasistatic(direction);
    }

    /**
     * Runs the SysId Dynamic test in the given direction for the routine
     * specified by {@link #m_sysIdRoutineToApply}.
     *
     * @param direction Direction of the SysId Dynamic test
     * @return Command to run
     */
    public Command sysIdDynamic(SysIdRoutine.Direction direction) {
        return m_sysIdRoutineToApply.dynamic(direction);
    }
    

}
