package frc.robot.Subsystems;

import com.ctre.phoenix6.SignalLogger;
import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.CANrangeConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.VelocityTorqueCurrentFOC;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.CANrange;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.SensorDirectionValue;

import static edu.wpi.first.units.Units.Second;
import static edu.wpi.first.units.Units.Volts;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.Constants;

public class Claw extends SubsystemBase {
    private final TalonFX m_clawPitch = new TalonFX(9, "canivore");
    private final TalonFX m_clawPipeWheel = new TalonFX(10, "canivore");
    private final TalonFX m_clawWheel = new TalonFX(11, "canivore");

    private final CANcoder m_clawPitchEncoder = new CANcoder(5, "canivore");
    private final CANrange m_canRange = new CANrange(1, "canivore");

    
    // set control mode for each motor
    private final MotionMagicVoltage m_clawPitchPositionRequest = new MotionMagicVoltage(0.0)
            .withSlot(0);
     
            
    private final VelocityTorqueCurrentFOC m_clawPipeWheelVelocityRequest = new VelocityTorqueCurrentFOC(0.0)
            .withSlot(0);
    private final VelocityTorqueCurrentFOC m_clawWheelVelocityRequest = new VelocityTorqueCurrentFOC(0.0)
            .withSlot(0);

                        
    // private final MotionMagicVelocityVoltage m_clawPipeWheelVelocityRequest = new MotionMagicVelocityVoltage(0.0)
    //     .withSlot(0);
    // private final MotionMagicVelocityVoltage m_clawWheelVelocityRequest = new MotionMagicVelocityVoltage(0.0)
    //     .withSlot(0);


    private int m_clawPitchPositionFlag = 0;

    // sysidroutine need voltageout mode
    private final VoltageOut m_clawPitchlVoltageRequest_sysid = new VoltageOut(0.0);
    private final VoltageOut m_clawPipeWheelVoltageRequest_sysid = new VoltageOut(0.0);
    private final VoltageOut m_clawWheelVoltageRequest_sysid = new VoltageOut(0.0);

    /*
     * SysId routine for characterizing clawpitch. This is used to find PID gains
     * for the motors.
     */
    private final SysIdRoutine m_sysIdRoutineClawPitch = new SysIdRoutine(
            new SysIdRoutine.Config(
                    Volts.per(Second).of(0.5), // Use default ramp rate (1 V/s)
                    // null,
                    Volts.of(1), // Reduce dynamic step voltage to 4 V to prevent brownout
                    null, // Use default timeout (10 s)
                    // Log state with SignalLogger class
                    state -> SignalLogger.writeString("SysIdClawPitch_State", state.toString())),
            new SysIdRoutine.Mechanism(
                    output -> m_clawPitch.setControl(m_clawPitchlVoltageRequest_sysid.withOutput(output)),
                    null,
                    this));

    /*
     * SysId routine for characterizing clawpipewheel. This is used to find PID
     * gains for the motors.
     */
    private final SysIdRoutine m_sysIdRoutineClawPipeWheel = new SysIdRoutine(
            new SysIdRoutine.Config(
                    null, // Use default ramp rate (1 V/s)
                    Volts.of(4), // Reduce dynamic step voltage to 4 V to prevent brownout
                    null, // Use default timeout (10 s)
                    // Log state with SignalLogger class
                    state -> SignalLogger.writeString("SysIdClawPipeWheel_State", state.toString())),
            new SysIdRoutine.Mechanism(
                    output -> m_clawPipeWheel.setControl(m_clawPipeWheelVoltageRequest_sysid.withOutput(output)),
                    null,
                    this));

    /*
     * SysId routine for characterizing calwwheel. This is used to find PID gains
     * for the motors.
     */
    private final SysIdRoutine m_sysIdRoutineClawWheel = new SysIdRoutine(
            new SysIdRoutine.Config(
                    null, // Use default ramp rate (1 V/s)
                    Volts.of(4), // Reduce dynamic step voltage to 4 V to prevent brownout
                    null, // Use default timeout (10 s)
                    // Log state with SignalLogger class
                    state -> SignalLogger.writeString("SysIdClawWheel_State", state.toString())),
            new SysIdRoutine.Mechanism(
                    output -> m_clawWheel.setControl(m_clawWheelVoltageRequest_sysid.withOutput(output)),
                    null,
                    this));

    // define which motor is used for testing
    private SysIdRoutine m_sysIdRoutineToApply = m_sysIdRoutineClawPitch;

    public Claw() {
        // clawPitchEncoder configs
        var clawPitchEncoderConfigs = new CANcoderConfiguration();

        clawPitchEncoderConfigs.MagnetSensor.MagnetOffset=0.642977;//offset
        clawPitchEncoderConfigs.MagnetSensor.AbsoluteSensorDiscontinuityPoint=0.5;
        clawPitchEncoderConfigs.MagnetSensor.SensorDirection=SensorDirectionValue.Clockwise_Positive;

        m_clawPitchEncoder.getConfigurator().apply(clawPitchEncoderConfigs);

        // claw pipe wheel configs
        var clawPipeWheelConfigs = new TalonFXConfiguration();

        clawPipeWheelConfigs.Slot0.kS = 4.7;
        clawPipeWheelConfigs.Slot0.kV = 0.09;
        clawPipeWheelConfigs.Slot0.kA = 1;
        clawPipeWheelConfigs.Slot0.kP = 4;
        clawPipeWheelConfigs.Slot0.kI = 0;
        clawPipeWheelConfigs.Slot0.kD = 0.25;
        clawPipeWheelConfigs.MotionMagic.MotionMagicAcceleration = 100; // Acceleration is around 40 rps/s
        clawPipeWheelConfigs.MotionMagic.MotionMagicCruiseVelocity = 200; // Unlimited cruise velocity
        clawPipeWheelConfigs.MotionMagic.MotionMagicExpo_kV = 0.12; // kV is around 0.12 V/rps
        clawPipeWheelConfigs.MotionMagic.MotionMagicExpo_kA = 0.1; // Use a slower kA of 0.1 V/(rps/s)
        clawPipeWheelConfigs.MotionMagic.MotionMagicJerk = 0; // Jerk is around 0

        // clawPipeWheelConfigs.Slot0.kS = 0.3;
        // clawPipeWheelConfigs.Slot0.kV = 0.12;
        // clawPipeWheelConfigs.Slot0.kA = 0;
        // clawPipeWheelConfigs.Slot0.kP = 0.3;
        // clawPipeWheelConfigs.Slot0.kI = 0;
        // clawPipeWheelConfigs.Slot0.kD = 0;
        // clawPipeWheelConfigs.MotionMagic.MotionMagicAcceleration = 2000; // Acceleration is around 40 rps/s
        // clawPipeWheelConfigs.MotionMagic.MotionMagicCruiseVelocity = 2000; // Unlimited cruise velocity
        // clawPipeWheelConfigs.MotionMagic.MotionMagicExpo_kV = 0.12; // kV is around 0.12 V/rps
        // clawPipeWheelConfigs.MotionMagic.MotionMagicExpo_kA = 0.1; // Use a slower kA of 0.1 V/(rps/s)
        // clawPipeWheelConfigs.MotionMagic.MotionMagicJerk = 0; // Jerk is around 0

        m_clawPipeWheel.getConfigurator().apply(clawPipeWheelConfigs);

        // claw wheel configs
        var clawWheelConfigs = new TalonFXConfiguration();

        clawWheelConfigs.Slot0.kS = 1.5;
        clawWheelConfigs.Slot0.kV = 0.08;
        clawWheelConfigs.Slot0.kA = 1;
        clawWheelConfigs.Slot0.kP = 7;
        clawWheelConfigs.Slot0.kI = 0;
        clawWheelConfigs.Slot0.kD = 0;
        clawWheelConfigs.MotionMagic.MotionMagicAcceleration = 100; // Acceleration is around 40 rps/s
        clawWheelConfigs.MotionMagic.MotionMagicCruiseVelocity = 200; // Unlimited cruise velocity
        clawWheelConfigs.MotionMagic.MotionMagicExpo_kV = 0.12; // kV is around 0.12 V/rps
        clawWheelConfigs.MotionMagic.MotionMagicExpo_kA = 0.1; // Use a slower kA of 0.1 V/(rps/s)
        clawWheelConfigs.MotionMagic.MotionMagicJerk = 0; // Jerk is around 0

        // clawWheelConfigs.Slot0.kS = 0.3;
        // clawWheelConfigs.Slot0.kV = 0.1;
        // clawWheelConfigs.Slot0.kA = 0;
        // clawWheelConfigs.Slot0.kP = 0.3;
        // clawWheelConfigs.Slot0.kI = 0;
        // clawWheelConfigs.Slot0.kD = 0;
        // clawWheelConfigs.MotionMagic.MotionMagicAcceleration = 2000; // Acceleration is around 40 rps/s
        // clawWheelConfigs.MotionMagic.MotionMagicCruiseVelocity = 2000; // Unlimited cruise velocity
        // clawWheelConfigs.MotionMagic.MotionMagicExpo_kV = 0.12; // kV is around 0.12 V/rps
        // clawWheelConfigs.MotionMagic.MotionMagicExpo_kA = 0.1; // Use a slower kA of 0.1 V/(rps/s)
        // clawWheelConfigs.MotionMagic.MotionMagicJerk = 0; // Jerk is around 0


        m_clawWheel.getConfigurator().apply(clawWheelConfigs);

        // claw pitch configs
        var clawPitchConfigs = new TalonFXConfiguration();

        clawPitchConfigs.Slot0.kS = 0.5;
        clawPitchConfigs.Slot0.kV = 0.5;
        clawPitchConfigs.Slot0.kA = 0;
        clawPitchConfigs.Slot0.kP = 30;
        clawPitchConfigs.Slot0.kI = 0;
        clawPitchConfigs.Slot0.kD = 0;
        clawPitchConfigs.Slot0.kG = 0.15;
        clawPitchConfigs.Slot0.withGravityType(GravityTypeValue.Arm_Cosine);
        clawPitchConfigs.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
        

        // set Motion Magic Expo settings
        clawPitchConfigs.MotionMagic.MotionMagicAcceleration = 30; // Acceleration is around 40 rps/s
        clawPitchConfigs.MotionMagic.MotionMagicCruiseVelocity = 60; // Unlimited cruise velocity
        clawPitchConfigs.MotionMagic.MotionMagicExpo_kV = 0.15; // kV is around 0.12 V/rps
        clawPitchConfigs.MotionMagic.MotionMagicExpo_kA = 0.1; // Use a slower kA of 0.1 V/(rps/s)
        clawPitchConfigs.MotionMagic.MotionMagicJerk = 0; // Jerk is around 0

        // set Feedback configs
        clawPitchConfigs.Feedback.FeedbackRemoteSensorID = m_clawPitchEncoder.getDeviceID();
        clawPitchConfigs.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.FusedCANcoder;
        clawPitchConfigs.Feedback.RotorToSensorRatio = 2.5; // 30:12

        m_clawPitch.getConfigurator().apply(clawPitchConfigs);

        // set CANrange configs as default
        m_canRange.getConfigurator().apply(new CANrangeConfiguration());


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
     * Gets the claw pitch position.
     * 
     * @return
     */
    public double getClawPitchPosition() {
        return m_clawPitchEncoder.getPosition().getValueAsDouble();
    }
    /**
     * realese the evelator motor
     * 
     * @return
     */
    public void releaseClawPitch() {
        m_clawPitch.setControl(new VoltageOut(0.0));
    }

    /**
     * Runs the claw wheel intake.
     * 
     * @return
     */

    public Command clawWheelIntake() {
       
        return runEnd(
            () ->   { if(!m_canRange.getIsDetected().getValue())
                        {
                            setClawPipeWheelVelocity(Constants.Claw.GET_REFF_SPEED);
                        }
                    },
  
            () -> new WaitCommand(0.1).andThen(() -> setClawPipeWheelVelocity(0)).schedule()
                //setClawPipeWheelVelocity(0);
                //System.out.println("Ball is detected: ");
        

        ).until(() -> m_canRange.getIsDetected().getValue());

    }

    /**
     * Runs the claw wheel outtake.
     * 
     * @return
     */
    public Command clawWheelOuttake() {

        return startEnd(
                () -> setClawPipeWheelVelocity(Constants.Claw.SHOOT_REFF_SPEED),

                () -> setClawPipeWheelVelocity(0)

        );

    }
    /**
     * Runs the claw wheel outtake during automation.
     * 
     * @return
     */
    public Command Auto_clawWheelOuttake() {

        return runOnce(
                () -> setClawPipeWheelVelocity(Constants.Claw.SHOOT_REFF_SPEED));

    }
    /**
     * back a little when coral sildes down
     * 
     * @return
     */
    public Command clawWheelbacklittle() {

        return startEnd(
                () -> setClawPipeWheelVelocity(Constants.Claw.BACK_REFF_SPEED),

                () -> setClawPipeWheelVelocity(0)

        );

    }
    /**
     * stop the pipewheel motor during automation
     * 
     * @return
     */
    public Command clawPipeWheelStop() {
        return runOnce(() -> 
            setClawPipeWheelVelocity(0)  

        );
    }

    /**
     * Runs the claw  pipewheel and wheel to intake the ball.
     * 
     * @return
     */
    public Command getBall() {
        return runEnd(() -> {
            //System.out.println("getBall");
            setClawPipeWheelVelocity(Constants.Claw.GET_BALL_PIPEWHEEL_SPEED);
            setClawWheelVelocity(Constants.Claw.GET_BALL_WHEEL_SPEED);
        },
                () -> {
                    setClawPipeWheelVelocity(Constants.Claw.HOLD_BALL_PIPEWHEEL_SPEED);
                    setClawWheelVelocity(Constants.Claw.HOLD_BALL_WHEEL_SPEED);
                });
    }

    /**
     * Runs the claw  pipewheel and wheel to outtake the ball.
     * 
     * @return
     */
    public Command shootBall() {
        return runEnd(() -> {
            setClawPipeWheelVelocity(Constants.Claw.SHOOT_BALL_PIPEWHEEL_SPEED);
            setClawWheelVelocity(Constants.Claw.SHOOT_BALL_WHEEL_SPEED);
        },
                () -> {
                    setClawPipeWheelVelocity(0.0);
                    setClawWheelVelocity(0.0);
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