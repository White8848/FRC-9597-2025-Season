// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;

import edu.wpi.first.math.geometry.Rotation2d;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.RotationsPerSecond;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;
import frc.robot.Subsystems.Claw;
import frc.robot.Subsystems.CommandSwerveDrivetrain;
import frc.robot.Subsystems.DeepCage;
import frc.robot.Subsystems.Elevator;
import frc.robot.generated.TunerConstants;


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

    private final CommandXboxController joystick = new CommandXboxController(0);

    public final CommandSwerveDrivetrain drivetrain = TunerConstants.createDrivetrain();

    public final DeepCage deepcage = new DeepCage();

    public final Elevator elevator = new Elevator();

    public final Claw claw = new Claw();


    public RobotContainer() {
        configureBindings();//operate mode bingings
    }

    private void configureBindings() {
        // Note that X is defined as forward according to WPILib convention,
        // and Y is defined as to the left according to WPILib convention.
        drivetrain.setDefaultCommand(
            // Drivetrain will execute this command periodically
            drivetrain.applyRequest(() ->
                drive.withVelocityX(-joystick.getLeftY() * MaxSpeed) // Drive forward with negative Y (forward)
                    .withVelocityY(-joystick.getLeftX() * MaxSpeed) // Drive left with negative X (left)
                    .withRotationalRate(-joystick.getRightX() * MaxAngularRate) // Drive counterclockwise with negative X (left)
            )
        );

        joystick.a().whileTrue(drivetrain.applyRequest(() -> brake));
        joystick.b().whileTrue(drivetrain.applyRequest(() ->
            point.withModuleDirection(new Rotation2d(-joystick.getLeftY(), -joystick.getLeftX()))
        ));

        // reset the field-centric heading on left bumper press
        joystick.leftBumper().onTrue(drivetrain.runOnce(() -> drivetrain.seedFieldCentric()));

        //test elevator to a desired position
        joystick.rightBumper().onTrue(elevator.MoveToLevel(10.0));

        drivetrain.registerTelemetry(logger::telemeterize);

        // Run SysId routines when holding back/start and X/Y.
        // Note that each routine should be run exactly once in a single log.

        /*drivetrain test */
        if (currentTestMode == TestMode.DRIVETRAIN) {
            joystick.back().and(joystick.y()).whileTrue(drivetrain.sysIdDynamic(Direction.kForward));
            joystick.back().and(joystick.x()).whileTrue(drivetrain.sysIdDynamic(Direction.kReverse));
            joystick.start().and(joystick.y()).whileTrue(drivetrain.sysIdQuasistatic(Direction.kForward));
            joystick.start().and(joystick.x()).whileTrue(drivetrain.sysIdQuasistatic(Direction.kReverse));
        }
        /*elevator test */
        else if (currentTestMode == TestMode.ELEVATOR) {
            joystick.back().and(joystick.y()).whileTrue(elevator.sysIdDynamic(Direction.kForward));
            joystick.back().and(joystick.x()).whileTrue(elevator.sysIdDynamic(Direction.kReverse));
            joystick.start().and(joystick.y()).whileTrue(elevator.sysIdQuasistatic(Direction.kForward));
            joystick.start().and(joystick.x()).whileTrue(elevator.sysIdQuasistatic(Direction.kReverse));
        }

        /*deepcage test */
        else if (currentTestMode == TestMode.DEEPCAGE) {
            joystick.back().and(joystick.y()).whileTrue(deepcage.sysIdDynamic(Direction.kForward));
            joystick.back().and(joystick.x()).whileTrue(deepcage.sysIdDynamic(Direction.kReverse));
            joystick.start().and(joystick.y()).whileTrue(deepcage.sysIdQuasistatic(Direction.kForward));
            joystick.start().and(joystick.x()).whileTrue(deepcage.sysIdQuasistatic(Direction.kReverse));
        }

        /*claw test */
        else if (currentTestMode == TestMode.CLAW) {
            joystick.back().and(joystick.y()).whileTrue(claw.sysIdDynamic(Direction.kForward));
            joystick.back().and(joystick.x()).whileTrue(claw.sysIdDynamic(Direction.kReverse));
            joystick.start().and(joystick.y()).whileTrue(claw.sysIdQuasistatic(Direction.kForward));
            joystick.start().and(joystick.x()).whileTrue(claw.sysIdQuasistatic(Direction.kReverse));
        }
    }

    public Command getAutonomousCommand() {
        return Commands.print("No autonomous command configured");
    }
}
