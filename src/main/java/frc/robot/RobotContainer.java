// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;

import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.RotationsPerSecond;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Subsystems.CANdleSystem;
import frc.robot.Subsystems.Claw;
import frc.robot.Subsystems.CommandSwerveDrivetrain;
import frc.robot.Subsystems.DeepCage;
import frc.robot.Subsystems.Elevator;
import frc.robot.generated.TunerConstants;

//import frc.robot.Commands.Getballcommand;


public class RobotContainer {

    //choose which subsyystem to test
    public enum TestMode {
        DRIVETRAIN,
        CLAW,
        ELEVATOR,
        DEEPCAGE,
        INTAKE
    }    
    private TestMode currentTestMode = TestMode.ELEVATOR; // FLAG
    

    private double MaxSpeed = TunerConstants.kSpeedAt12Volts.in(MetersPerSecond); // kSpeedAt12Volts desired top speed
    private double MaxAngularRate = RotationsPerSecond.of(0.75).in(RadiansPerSecond); // 3/4 of a rotation per second max angular velocity

    /* Setting up bindings for necessary control of the swerve drive platform */
    private final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
            .withDeadband(MaxSpeed * 0.1).withRotationalDeadband(MaxAngularRate * 0.1) // Add a 10% deadband
            .withDriveRequestType(DriveRequestType.OpenLoopVoltage); // Use open-loop control for drive motors
    private final SwerveRequest.SwerveDriveBrake brake = new SwerveRequest.SwerveDriveBrake();
    private final SwerveRequest.PointWheelsAt point = new SwerveRequest.PointWheelsAt();

    private final Telemetry logger = new Telemetry(MaxSpeed);

    //private final CommandXboxController joystick = new CommandXboxController(0);
    private final CommandXboxController m_driverJoystick = new CommandXboxController(0);//two xbox controller
    private final CommandXboxController m_operatorJoystick = new CommandXboxController(1);

    public final CommandSwerveDrivetrain drivetrain = TunerConstants.createDrivetrain();

    public final DeepCage deepcage = new DeepCage();

    public final Elevator elevator = new Elevator();

    public final CANdleSystem candle = new CANdleSystem(elevator);
    
    public final Claw claw = new Claw();



    public RobotContainer() {
        configureBindings();//operate mode bingings
    }

    private void configureBindings() {
        //************************************************************ (driver) *******************************************************

        // Note that X is defined as forward according to WPILib convention,
        // and Y is defined as to the left according to WPILib convention.
        drivetrain.setDefaultCommand(
            // Drivetrain will execute this command periodically
            drivetrain.applyRequest(() ->
                // drive.withVelocityX(Utility.quadraticCurve(-m_driverJoystick.getLeftY()) * MaxSpeed)// Drive forward with negative Y (forward)
                // .withVelocityY(Utility.quadraticCurve(-m_driverJoystick.getLeftX() )* MaxSpeed)// Drive left with negative X (left)
                // .withRotationalRate(Utility.quadraticCurve(-m_driverJoystick.getRightX()) * MaxAngularRate)// Drive counterclockwise with negative X (left)

                drive.withVelocityX((-m_driverJoystick.getLeftY()) * MaxSpeed)// Drive forward with negative Y (forward)
                .withVelocityY((-m_driverJoystick.getLeftX() )* MaxSpeed)// Drive left with negative X (left)
                .withRotationalRate((-m_driverJoystick.getRightX()) * MaxAngularRate)// Drive counterclockwise with negative X (left)
            )
        );

        // m_driverJoystick.a().whileTrue(drivetrain.applyRequest(() -> brake));
        // m_driverJoystick.b().whileTrue(drivetrain.applyRequest(() ->
        //     point.withModuleDirection(new Rotation2d(-m_driverJoystick.getLeftY(), -m_driverJoystick.getLeftX()))
        // ));

        // reset the field-centric heading on left bumper press
        drivetrain.registerTelemetry(logger::telemeterize);

        //set the direction of heading
        m_driverJoystick.leftBumper().onTrue(drivetrain.runOnce(() -> drivetrain.seedFieldCentric()));

        //press the button to change the color state to showoff
        m_driverJoystick.a().onTrue(new RunCommand(candle::incrementAnimation, candle));

        //intake reef
        m_driverJoystick.rightBumper().whileTrue(claw.clawWheelIntake());
        //shoot reef
        m_driverJoystick.rightTrigger().whileTrue(claw.clawWheelOuttake());

        // //************************************************************ DEEPCAGE(operator) ****************************************************
        //m_driverJoystick.leftTrigger().whileTrue(deepcage.intakeBall());
        //m_driverJoystick.rightTrigger().whileTrue(deepcage.ejectBall());


        //************************************************************ (operator) ********************************************************
        //back to start
        m_operatorJoystick.x().onTrue(elevator.ElevatorClawMove(Constants.Elevator.State.START)
                                    .andThen((new InstantCommand(() -> candle.Changecolor(Constants.Elevator.State.START), candle))));
        //reef2
        m_operatorJoystick.povLeft().onTrue(elevator.ElevatorClawMove(Constants.Elevator.State.REEF_2)
                                    .andThen((new InstantCommand(() -> candle.Changecolor(Constants.Elevator.State.REEF_2), candle))));
        //reef3
        m_operatorJoystick.povRight().onTrue(elevator.ElevatorClawMove(Constants.Elevator.State.REEF_3)
                                    .andThen((new InstantCommand(() -> candle.Changecolor(Constants.Elevator.State.REEF_3), candle))));
        //reef4
        m_operatorJoystick.povUp().onTrue(elevator.ElevatorClawMove(Constants.Elevator.State.REEF_4)
                                    .andThen((new InstantCommand(() -> candle.Changecolor(Constants.Elevator.State.REEF_4), candle))));
        //get ball position 1
   
        m_operatorJoystick.a().whileTrue(
                                    Commands.parallel(
                                        elevator.ElevatorClawMove(Constants.Elevator.State.GETBALL1), // 让电梯和爪子运动
                                        claw.getBall() // 获取球的命令                                  
                                    ).andThen(new InstantCommand(() -> candle.Changecolor(Constants.Elevator.State.GETBALL1), candle))); // 颜色变化
        //get ball position 2
        m_operatorJoystick.y().whileTrue(
                                    Commands.parallel(
                                        elevator.ElevatorClawMove(Constants.Elevator.State.GETBALL2), // 让电梯和爪子运动
                                        claw.getBall() // 获取球的命令   
                                    ).andThen(new InstantCommand(() -> candle.Changecolor(Constants.Elevator.State.GETBALL2), candle))); // 颜色变化

        //barge                              
        m_operatorJoystick.b().onTrue(elevator.ElevatorClawMove(Constants.Elevator.State.BARGE)
                                    .andThen((new InstantCommand(() -> candle.Changecolor(Constants.Elevator.State.BARGE), candle))));
        //shoot ball
        m_operatorJoystick.rightTrigger().whileTrue(claw.shootBall());



        // //************************************************************ DEEPCAGE(operator) ****************************************************
        // m_operatorJoystick.leftTrigger().whileTrue(deepcage.deepCagePitchUp());
        // m_operatorJoystick.rightTrigger().whileTrue(deepcage.deepCagePitchDown());


        // //********************************************************** Sysidroutine ******************************************************
        // // Run SysId routines when holding back/start and X/Y.
        // // Note that each routine should be run exactly once in a single log.

        // /*drivetrain test */
        // if (currentTestMode == TestMode.DRIVETRAIN) {
        //     joystick.back().and(joystick.y()).whileTrue(drivetrain.sysIdDynamic(Direction.kForward));
        //     joystick.back().and(joystick.x()).whileTrue(drivetrain.sysIdDynamic(Direction.kReverse));
        //     joystick.start().and(joystick.y()).whileTrue(drivetrain.sysIdQuasistatic(Direction.kForward));
        //     joystick.start().and(joystick.x()).whileTrue(drivetrain.sysIdQuasistatic(Direction.kReverse));
        // }
        // /*elevator test */
        // else if (currentTestMode == TestMode.ELEVATOR) {
        //     joystick.back().and(joystick.y()).whileTrue(elevator.sysIdDynamic(Direction.kForward));
        //     joystick.back().and(joystick.x()).whileTrue(elevator.sysIdDynamic(Direction.kReverse));
        //     joystick.start().and(joystick.y()).whileTrue(elevator.sysIdQuasistatic(Direction.kForward));
        //     joystick.start().and(joystick.x()).whileTrue(elevator.sysIdQuasistatic(Direction.kReverse));
        // }

        // /*deepcage test */
        // else if (currentTestMode == TestMode.DEEPCAGE) {
        //     joystick.back().and(joystick.y()).whileTrue(deepcage.sysIdDynamic(Direction.kForward));
        //     joystick.back().and(joystick.x()).whileTrue(deepcage.sysIdDynamic(Direction.kReverse));
        //     joystick.start().and(joystick.y()).whileTrue(deepcage.sysIdQuasistatic(Direction.kForward));
        //     joystick.start().and(joystick.x()).whileTrue(deepcage.sysIdQuasistatic(Direction.kReverse));
        // }

        // /*claw test */
        // else if (currentTestMode == TestMode.CLAW) {
        //     joystick.back().and(joystick.y()).whileTrue(claw.sysIdDynamic(Direction.kForward));
        //     joystick.back().and(joystick.x()).whileTrue(claw.sysIdDynamic(Direction.kReverse));
        //     joystick.start().and(joystick.y()).whileTrue(claw.sysIdQuasistatic(Direction.kForward));
        //     joystick.start().and(joystick.x()).whileTrue(claw.sysIdQuasistatic(Direction.kReverse));
        // }
    }

    public Command getAutonomousCommand() {
        return Commands.print("No autonomous command configured");
    }
}
