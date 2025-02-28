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
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.Constants;

public class Elevator extends SubsystemBase {

    private final TalonFX m_elevatorLeft = new TalonFX(12, "rio");
    private final TalonFX m_elevatorRight = new TalonFX(13, "rio");

    // CONTROL MODE
    private final MotionMagicVoltage m_elevatorPositionRequest = new MotionMagicVoltage(0.0)
            .withSlot(0);

    // sysidroutine need voltageout mode
    private final VoltageOut m_elevator_leftVoltageRequest_sysid = new VoltageOut(0.0);
    private final VoltageOut m_elevator_rightVoltageRequest_sysid = new VoltageOut(0.0);

    private final Claw m_claw;

    private Constants.Elevator.State m_current_state = Constants.Elevator.State.START;// describe the current state of
                                                                                      // the elevator

    private boolean m_elevatorClawIsMoving = false;// describe the elevator is moving or not
    private double m_elevatorPosition = 0.0;// describe the elevator position
    private double m_clawPosition = 0.0;// describe the claw position

    /*
     * SysId routine for characterizing clawpitch. This is used to find PID gains
     * for the motors.
     */
    private final SysIdRoutine m_sysIdRoutineElevator = new SysIdRoutine(
            new SysIdRoutine.Config(
                    Volts.per(Second).of(0.5),
                    // null, // Use default ramp rate (1 V/s)
                    Volts.of(1.2), // Reduce dynamic step voltage to 4 V to prevent brownout
                    null, // Use default timeout (10 s)
                    // Log state with SignalLogger class
                    state -> SignalLogger.writeString("SysIdElevator_State", state.toString())),
            new SysIdRoutine.Mechanism(
                    output -> {
                        m_elevatorLeft.setControl(m_elevator_leftVoltageRequest_sysid.withOutput(output));
                        m_elevatorRight.setControl(m_elevator_rightVoltageRequest_sysid.withOutput(output));
                    },
                    null,
                    this));

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
        elevatorConfigs.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;

        // set Motion Magic Expo settings
        elevatorConfigs.MotionMagic.MotionMagicAcceleration = 40;
        elevatorConfigs.MotionMagic.MotionMagicCruiseVelocity = 20;
        elevatorConfigs.MotionMagic.MotionMagicExpo_kV = 0.12;
        elevatorConfigs.MotionMagic.MotionMagicExpo_kA = 0.1;
        elevatorConfigs.MotionMagic.MotionMagicJerk = 0;

        m_elevatorLeft.getConfigurator().apply(elevatorConfigs);

        // Set the right motor to follow the left motor
        m_elevatorRight.setControl(new Follower(m_elevatorLeft.getDeviceID(), false));

        m_claw = new Claw();

        m_elevatorPosition = getM_elevatorPosition();
        m_clawPosition = m_claw.getClawPitchPosition();
    }

    public void setElevatorPosition(double position) {
        m_elevatorLeft.setControl(m_elevatorPositionRequest.withPosition(position));
    }

    public double getM_elevatorPosition() {
        return m_elevatorLeft.getPosition().getValueAsDouble();
    }

    public Command MoveToLevel(double position) {
        return run(() -> this.setElevatorPosition(position));
    }

    // realese the evelator motor
    public void releaseElevator() {
        m_elevatorLeft.setControl(new VoltageOut(0.0));
    }

    /**
     * Adjusts the elevator and claw positions based on the desired state.
     * Moves the claw and elevator sequentially, ensuring the proper order of
     * operations
     * for each state transition, including special handling when transitioning
     * to/from REEF_4.
     *
     * @param m_desired_state The target state for the elevator and claw.
     * @return Command to execute the position adjustments.
     */
    public Command ElevatorClawMove(Constants.Elevator.State m_desired_state) {
        System.out.println("start ElevatorClawMove");
        return run(() -> {
            System.out.println("start run");
            if (m_elevatorClawIsMoving == false) {
                m_elevatorPosition = getM_elevatorPosition();
                m_clawPosition = m_claw.getClawPitchPosition();
                m_elevatorClawIsMoving = true;
            }

            switch (m_desired_state) {
                case START:
                    m_clawPosition = Constants.Claw.CLAW_PITCH_START;
                    m_elevatorPosition = Constants.Elevator.ELEVATOR_START;
                    break;
                case INTAKE:
                    m_clawPosition = Constants.Claw.CLAW_PITCH_INTAKE;
                    m_elevatorPosition = Constants.Elevator.ELEVATOR_INTAKE;
                    break;
                case REEF_2:
                    m_clawPosition = Constants.Claw.CLAW_PITCH_REEF_2;
                    m_elevatorPosition = Constants.Elevator.ELEVATOR_REEF_2;
                    break;
                case REEF_3:
                    m_clawPosition = Constants.Claw.CLAW_PITCH_REEF_3;
                    m_elevatorPosition = Constants.Elevator.ELEVATOR_REEF_3;
                    break;
                case REEF_4:
                    m_clawPosition = Constants.Claw.CLAW_PITCH_REEF_4;
                    m_elevatorPosition = Constants.Elevator.ELEVATOR_REEF_4;
                    break;
                case GETBALL1:
                    m_clawPosition = Constants.Claw.CLAW_PITCH_GETBALL;
                    m_elevatorPosition = Constants.Elevator.ELEVATOR_GETBALL1;
                    break;
                case GETBALL2:
                    m_clawPosition = Constants.Claw.CLAW_PITCH_GETBALL;
                    m_elevatorPosition = Constants.Elevator.ELEVATOR_GETBALL2;
                    break;
                case BARGE:
                    m_clawPosition = Constants.Claw.CLAW_PITCH_BARGE;
                    m_elevatorPosition = Constants.Elevator.ELEVATOR_BARGE;
                    break;
            }

            // From any position to 4 or from 4 to any position
            // First arrive at the unblocked position, then move the elevator, and then move
            // the claw
            if ((m_desired_state == Constants.Elevator.State.REEF_4)
                    || (m_current_state == Constants.Elevator.State.REEF_4)) {
                m_claw.setClawPitchPosition(0.05);// move the claw avoid collision
                if (Math.abs(m_claw.getClawPitchPosition() - m_clawPosition) < Constants.Claw.CLAW_PITCH_ACCEPT_ERROR) {
                    // Wait for the claw to reach the desired position
                    setElevatorPosition(m_elevatorPosition);// move the elevator
                } else if (Math
                        .abs(getM_elevatorPosition() - m_elevatorPosition) < Constants.Elevator.ELEVATOR_ACCEPT_ERROR) {
                    m_claw.setClawPitchPosition(m_clawPosition);// move the claw
                }

            }

            // Other positions to 0
            // Move the elevator first, then move the claw
            else if (m_desired_state == Constants.Elevator.State.START
                    && m_current_state != Constants.Elevator.State.REEF_4) {
                setElevatorPosition(m_elevatorPosition);// move the elevator
                if (Math.abs(getM_elevatorPosition() - m_elevatorPosition) < Constants.Elevator.ELEVATOR_ACCEPT_ERROR) {
                    // Wait for the elevator to reach the desired position
                    m_claw.setClawPitchPosition(m_clawPosition);// move the claw
                }
            }

            // other circumstances
            // Move claw and elevator at the same time
            else {
                m_claw.setClawPitchPosition(m_clawPosition);// move the claw
                setElevatorPosition(m_elevatorPosition);// move the elevator

            }

        }).until(() -> {
            // Check if the elevator and claw have reached their target positions
            return (Math.abs(getM_elevatorPosition() - m_elevatorPosition) < Constants.Elevator.ELEVATOR_ACCEPT_ERROR
                    &&
                    Math.abs(m_claw.getClawPitchPosition() - m_clawPosition) < Constants.Claw.CLAW_PITCH_ACCEPT_ERROR);
        }).andThen(() -> {
            // Set the moving flag to false after the move is complete
            m_elevatorClawIsMoving = false;
            m_current_state = m_desired_state;// update the current state of the elevator for the next move
            System.out.println("Elevator and claw have reached their target positions.");
            System.out.println("Current State: " + m_current_state);
            // Release the motor
            if (m_current_state == Constants.Elevator.State.START) {
                new WaitCommand(1.0).andThen(() -> {
                    releaseElevator(); // Release the elevator motor
                    m_claw.releaseClawPitch(); // Release the claw pitch motor
                }).schedule();
            }
        }).withTimeout(5.0);// set a timeout for the command
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
