package frc.robot.Subsystems;

import java.util.Arrays;
import java.util.Comparator;
import java.util.HashMap;
import java.util.List;
import java.util.Map;
import java.util.Optional;
import java.util.function.Supplier;

import org.littletonrobotics.junction.Logger;
import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.MultiTargetPNPResult;;
import org.photonvision.targeting.PhotonTrackedTarget;

import com.ctre.phoenix6.SignalLogger;
import com.ctre.phoenix6.Utils;
import com.ctre.phoenix6.swerve.SwerveDrivetrain.SwerveDriveState;
import com.ctre.phoenix6.swerve.SwerveDrivetrainConstants;
import com.ctre.phoenix6.swerve.SwerveModuleConstants;
import com.ctre.phoenix6.swerve.SwerveRequest;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;
import static edu.wpi.first.units.Units.Second;
import static edu.wpi.first.units.Units.Volts;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Mass;
import edu.wpi.first.units.measure.MomentOfInertia;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.Notifier;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Subsystem;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.Constants;
import frc.robot.generated.TunerConstants;
import frc.robot.generated.TunerConstants.TunerSwerveDrivetrain;


import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;
import com.pathplanner.lib.config.*;
import com.ctre.phoenix6.swerve.SwerveModuleConstants;
import com.pathplanner.lib.util.swerve.SwerveSetpointGenerator;

import static edu.wpi.first.units.Units.Kilogram;
import static edu.wpi.first.units.Units.KilogramSquareMeters;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.RotationsPerSecond;


/**
 * Class that extends the Phoenix 6 SwerveDrivetrain class and implements
 * Subsystem so it can easily be used in command-based projects.
 */
public class CommandSwerveDrivetrain extends TunerSwerveDrivetrain implements Subsystem {
    private static final double kSimLoopPeriod = 0.005; // 5 ms
    private Notifier m_simNotifier = null;
    private double m_lastSimTime;

    /* Blue alliance sees forward as 0 degrees (toward red alliance wall) */
    private static final Rotation2d kBlueAlliancePerspectiveRotation = Rotation2d.kZero;
    /* Red alliance sees forward as 180 degrees (toward blue alliance wall) */
    private static final Rotation2d kRedAlliancePerspectiveRotation = Rotation2d.k180deg;
    /* Keep track if we've ever applied the operator perspective before or not */
    private boolean m_hasAppliedOperatorPerspective = false;

    /* Swerve requests to apply during SysId characterization */
    private final SwerveRequest.SysIdSwerveTranslation m_translationCharacterization = new SwerveRequest.SysIdSwerveTranslation();
    private final SwerveRequest.SysIdSwerveSteerGains m_steerCharacterization = new SwerveRequest.SysIdSwerveSteerGains();
    private final SwerveRequest.SysIdSwerveRotation m_rotationCharacterization = new SwerveRequest.SysIdSwerveRotation();

    //vision
    public  boolean kUseVision = true;
    private final Map<PhotonCamera, PhotonPoseEstimator> cameraEstimators = new HashMap<>();
    private final AprilTagFieldLayout aprilTagFieldLayout = AprilTagFieldLayout.loadField(AprilTagFields.kDefaultField);

    //Auto
    // PathPlanner config constants
    private static final Mass ROBOT_MASS = Kilogram.of(74);
    private static final MomentOfInertia ROBOT_MOI =KilogramSquareMeters.of(6.8);
    private static final double WHEEL_COF = 1.2;
    public static final SwerveModuleConstants SWERVE_MODULE_CONSTANTS = TunerConstants.FrontLeft;
    public static final Translation2d[] SWERVE_MODULE_OFFSETS =
    new Translation2d[] {
      new Translation2d(TunerConstants.FrontLeft.LocationX, TunerConstants.FrontLeft.LocationY),
      new Translation2d(TunerConstants.FrontRight.LocationX, TunerConstants.FrontRight.LocationY),
      new Translation2d(TunerConstants.BackLeft.LocationX, TunerConstants.BackLeft.LocationY),
      new Translation2d(TunerConstants.BackRight.LocationX, TunerConstants.BackRight.LocationY)
    };

    /** Swerve request to apply during robot-centric path following */
    private final SwerveRequest.ApplyRobotSpeeds m_pathApplyRobotSpeeds =new SwerveRequest.ApplyRobotSpeeds();

    
    /* SysId routine for characterizing translation. This is used to find PID gains for the drive motors. */
    private final SysIdRoutine m_sysIdRoutineTranslation = new SysIdRoutine(
        new SysIdRoutine.Config(
            null,        // Use default ramp rate (1 V/s)
            Volts.of(4), // Reduce dynamic step voltage to 4 V to prevent brownout
            null,        // Use default timeout (10 s)
            // Log state with SignalLogger class
            state -> SignalLogger.writeString("SysIdTranslation_State", state.toString())
        ),
        new SysIdRoutine.Mechanism(
            output -> setControl(m_translationCharacterization.withVolts(output)),
            null,
            this
        )
    );

    /* SysId routine for characterizing steer. This is used to find PID gains for the steer motors. */
    private final SysIdRoutine m_sysIdRoutineSteer = new SysIdRoutine(
        new SysIdRoutine.Config(
            null,        // Use default ramp rate (1 V/s)
            Volts.of(7), // Use dynamic voltage of 7 V
            null,        // Use default timeout (10 s)
            // Log state with SignalLogger class
            state -> SignalLogger.writeString("SysIdSteer_State", state.toString())
        ),
        new SysIdRoutine.Mechanism(
            volts -> setControl(m_steerCharacterization.withVolts(volts)),
            null,
            this
        )
    );

    /*
     * SysId routine for characterizing rotation.
     * This is used to find PID gains for the FieldCentricFacingAngle HeadingController.
     * See the documentation of SwerveRequest.SysIdSwerveRotation for info on importing the log to SysId.
     */
    private final SysIdRoutine m_sysIdRoutineRotation = new SysIdRoutine(
        new SysIdRoutine.Config(
            /* This is in radians per second², but SysId only supports "volts per second" */
            Volts.of(Math.PI / 6).per(Second),
            /* This is in radians per second, but SysId only supports "volts" */
            Volts.of(Math.PI),
            null, // Use default timeout (10 s)
            // Log state with SignalLogger class
            state -> SignalLogger.writeString("SysIdRotation_State", state.toString())
        ),
        new SysIdRoutine.Mechanism(
            output -> {
                /* output is actually radians per second, but SysId only supports "volts" */
                setControl(m_rotationCharacterization.withRotationalRate(output.in(Volts)));
                /* also log the requested output for SysId */
                SignalLogger.writeDouble("Rotational_Rate", output.in(Volts));
            },
            null,
            this
        )
    );


    /* The SysId routine to test */
    private SysIdRoutine m_sysIdRoutineToApply = m_sysIdRoutineTranslation;

    /**
     * Constructs a CTRE SwerveDrivetrain using the specified constants .
     * <p>
     * This constructs the underlying hardware devices, so users should not construct
     * the devices themselves. If they need the devices, they can access them through
     * getters in the classes.
     *
     * @param drivetrainConstants   Drivetrain-wide constants for the swerve drive
     * @param modules               Constants for each specific module
     */
    public CommandSwerveDrivetrain(
        SwerveDrivetrainConstants drivetrainConstants,
        SwerveModuleConstants<?, ?, ?>... modules
    ) {
        super(drivetrainConstants, modules);
        if (Utils.isSimulation()) {
            startSimThread();
        }

        // initialize camera system
        cameraEstimators.put(
            new PhotonCamera("Camera_left"),
            new PhotonPoseEstimator(
                aprilTagFieldLayout,
                PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR,
                new Transform3d(
                    new Translation3d(0.32, 0.32, 0.5),//z to elevator
                    new Rotation3d(     
                    0,
                    Units.degreesToRadians(0),//pitch
                    Units.degreesToRadians(30))//yaw
                )
            )
        );
        cameraEstimators.put(
            new PhotonCamera("Camera_right"), 
            new PhotonPoseEstimator(
                aprilTagFieldLayout,
                PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR,
                new Transform3d(
                    new Translation3d(0.32, -0.32, 0.5),
                    new Rotation3d(     
                        0,
                        Units.degreesToRadians(0),//pitch
                        Units.degreesToRadians(330))//yaw
                )
            )
        );
        System.out.println("finish camera initialization");

        // Robot config
        RobotConfig robotConfig = null; // Initialize with null in case of exception
        try {
            robotConfig =
                    RobotConfig.fromGUISettings(); // Takes config from Robot Config on Pathplanner
            // Settings
        } catch (Exception e) {
            e.printStackTrace(); // Fallback to a default configuration
        }

        // Configure AutoBuilder last
        AutoBuilder.configure(
                this::getPose, // Robot pose supplier
                this::resetPose, // Method to reset odometry (will be called if your auto has a starting pose)
                this::getRobotRelativeSpeeds, // ChassisSpeeds supplier. MUST BE ROBOT RELATIVE
                (speeds, feedforwards) -> this.setControl(
                    m_pathApplyRobotSpeeds
                        .withSpeeds(speeds)
                        .withWheelForceFeedforwardsX(feedforwards.robotRelativeForcesXNewtons())
                        .withWheelForceFeedforwardsY(feedforwards.robotRelativeForcesYNewtons())), // Method that will drive the robot given ROBOT RELATIVE ChassisSpeeds. Also optionally outputs individual module feedforwards
                new PPHolonomicDriveController( // PPHolonomicController is the built in path following controller for holonomic drive trains
                        new PIDConstants(5.0, 0.0, 0.0), // Translation PID constants
                        new PIDConstants(5.0, 0.0, 0.0) // Rotation PID constants
                ),
                robotConfig,//load the parameters from the gui
                () -> {

                // Boolean supplier that controls when the path will be mirrored for the red alliance
                // This will flip the path being followed to the red side of the field.
                // THE ORIGIN WILL REMAIN ON THE BLUE SIDE

                var alliance = DriverStation.getAlliance();
                if (alliance.isPresent()) {
                    return alliance.get() == DriverStation.Alliance.Red;
                }
                return false;
                },
                this // Reference to this subsystem to set requirements
        );
    }



    /**
     * Returns a command that applies the specified control request to this swerve drivetrain.
     *
     * @param request Function returning the request to apply
     * @return Command to run
     */
    public Command applyRequest(Supplier<SwerveRequest> requestSupplier) {
        return run(() -> this.setControl(requestSupplier.get()));
    }

    
    @Override
    public void periodic() {
        /*
         * Periodically try to apply the operator perspective.
         * If we haven't applied the operator perspective before, then we should apply it regardless of DS state.
         * This allows us to correct the perspective in case the robot code restarts mid-match.
         * Otherwise, only check and apply the operator perspective if the DS is disabled.
         * This ensures driving behavior doesn't change until an explicit disable event occurs during testing.
         */
        if (!m_hasAppliedOperatorPerspective || DriverStation.isDisabled()) {
            DriverStation.getAlliance().ifPresent(allianceColor -> {
                setOperatorPerspectiveForward(
                    allianceColor == Alliance.Red
                        ? kRedAlliancePerspectiveRotation
                        : kBlueAlliancePerspectiveRotation
                );
                m_hasAppliedOperatorPerspective = true;
            });
        }

        //vision processing 
        processVisionData();
    }

/***************************************************************** process  vision data   ***********************************************************/
    private void processVisionData() {
        //System.out.println("vision status "+kUseVision);
        SwerveDriveState driveState = getState();
        Pose2d odometryPose = driveState.Pose;

        // report the pose of drivetrain
        SmartDashboard.putNumberArray("Chassis/Pose", 
            new double[] {
                odometryPose.getX(),
                odometryPose.getY(),
                odometryPose.getRotation().getDegrees()
            });

        cameraEstimators.forEach((camera, estimator) -> {
            List<PhotonPipelineResult> cameraResults = camera.getAllUnreadResults();
            if (cameraResults.isEmpty()) return;

            PhotonPipelineResult latestResult = cameraResults.get(cameraResults.size() - 1);
            if (!latestResult.hasTargets()) return;
            //System.out.println("size"+latestResult.targets.size());

            estimator.setReferencePose(odometryPose);
            Optional<EstimatedRobotPose> estimatedPose = estimator.update(latestResult);
            
            estimatedPose.ifPresent(pose -> {
                Matrix<N3, N1> stdDevs = calculateAdaptiveStdDevs(
                    latestResult.targets.size(),
                    calculateAverageDistance(latestResult.targets),
                    driveState.Speeds
                );
              
                if(kUseVision){//if vision is enabled
                    addVisionMeasurement(
                        pose.estimatedPose.toPose2d(),
                        latestResult.getTimestampSeconds(),
                        stdDevs
                    );
                }

                // report vision data
                Pose2d visionPose = pose.estimatedPose.toPose2d();
                String cameraName = camera.getName();
                SmartDashboard.putNumberArray("Vision/Pose/" + cameraName, 
                    new double[] {
                        visionPose.getX(),
                        visionPose.getY(),
                        visionPose.getRotation().getDegrees()
                    });
                
                // report ID
                int[] targetIds = latestResult.targets.stream()
                    .mapToInt(PhotonTrackedTarget::getFiducialId)
                    .toArray();
                SmartDashboard.putNumberArray("Vision/Targets/" + cameraName, 
                    Arrays.stream(targetIds).asDoubleStream().toArray());

                Logger.recordOutput("Vision/" + cameraName + "/Pose", pose.estimatedPose);
            });
        });
    }


    // New standard deviation calculation method
    private Matrix<N3, N1> calculateAdaptiveStdDevs(int tagCount, double avgDistance, ChassisSpeeds speeds) {
        double baseXY, baseTheta;
        
        // Set the base value based on the number of tags
        if (tagCount >= 2) {
            baseXY = 0.5;  
            baseTheta = Math.toRadians(5); 
        } else {
            baseXY = 1.5 + avgDistance * 0.2; 
            baseTheta = Math.toRadians(10 + avgDistance * 3); 
        }

        // Adjust dynamically based on speed
        double speedFactor = Math.hypot(speeds.vxMetersPerSecond, speeds.vyMetersPerSecond) / 4.0; // suppose the speed max=4m/s
        double rotationFactor = Math.abs(speeds.omegaRadiansPerSecond) / Math.PI; // suppose the angular_speed=π rad/s
        
        return VecBuilder.fill(
            baseXY * (1 + speedFactor),       // X standard deviation
            baseXY * (1 + speedFactor),       // Y standard deviation
            baseTheta * (1 + rotationFactor)  // θ standard deviation
        );
    }

    // calculateAverageDistance
    private double calculateAverageDistance(List<PhotonTrackedTarget> targets) {
        return targets.stream()
            .mapToDouble(t -> t.getBestCameraToTarget().getTranslation().getNorm())
            .average()
            .orElse(0.0);
    }

/********************************************************************** vision end *********************************************************** */


    private void startSimThread() {
        m_lastSimTime = Utils.getCurrentTimeSeconds();

        /* Run simulation at a faster rate so PID gains behave more reasonably */
        m_simNotifier = new Notifier(() -> {
            final double currentTime = Utils.getCurrentTimeSeconds();
            double deltaTime = currentTime - m_lastSimTime;
            m_lastSimTime = currentTime;

            /* use the measured time delta, get battery voltage from WPILib */
            updateSimState(deltaTime, RobotController.getBatteryVoltage());
        });
        m_simNotifier.startPeriodic(kSimLoopPeriod);
    }

    /**
     * Adds a vision measurement to the Kalman Filter. This will correct the odometry pose estimate
     * while still accounting for measurement noise.
     *
     * @param visionRobotPoseMeters The pose of the robot as measured by the vision camera.
     * @param timestampSeconds The timestamp of the vision measurement in seconds.
     */
    @Override
    public void addVisionMeasurement(Pose2d visionRobotPoseMeters, double timestampSeconds) {
        super.addVisionMeasurement(visionRobotPoseMeters, Utils.fpgaToCurrentTime(timestampSeconds));
    }

    /**
     * Adds a vision measurement to the Kalman Filter. This will correct the odometry pose estimate
     * while still accounting for measurement noise.
     * <p>
     * Note that the vision measurement standard deviations passed into this method
     * will continue to apply to future measurements until a subsequent call to
     * {@link #setVisionMeasurementStdDevs(Matrix)} or this method.
     *
     * @param visionRobotPoseMeters The pose of the robot as measured by the vision camera.
     * @param timestampSeconds The timestamp of the vision measurement in seconds.
     * @param visionMeasurementStdDevs Standard deviations of the vision pose measurement
     *     in the form [x, y, theta]ᵀ, with units in meters and radians.
     */
    @Override
    public void addVisionMeasurement(
        Pose2d visionRobotPoseMeters,
        double timestampSeconds,
        Matrix<N3, N1> visionMeasurementStdDevs
    ) {
        super.addVisionMeasurement(visionRobotPoseMeters, Utils.fpgaToCurrentTime(timestampSeconds), visionMeasurementStdDevs);
    }

    //get the pose of the robot
    public Pose2d getPose() {
        return this.getState().Pose;
    }

    //reset the pose of the robot
    public void resetPose(Pose2d newPose) {
        this.seedFieldCentric();
    }

    //get the robot relative speeds
    public ChassisSpeeds getRobotRelativeSpeeds() {
        return this.getKinematics().toChassisSpeeds(this.getState().ModuleStates);
    }

    //get the robot state auto or operated
    public Constants.Elevator.State Get_Auto_State(){
        if(kUseVision){
            return Constants.Elevator.State.START_AUTO;
        }
        else{
            return Constants.Elevator.State.START_OPERATED;
        }
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

    public Command ChangeVisionDataStatus(){
        return  runOnce(()->{
            //System.out.println("vision status"+kUseVision);
            kUseVision = !kUseVision;
        });
    }

        
}

