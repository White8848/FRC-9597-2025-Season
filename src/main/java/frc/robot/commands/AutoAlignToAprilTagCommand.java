// package frc.robot.commands;
// import edu.wpi.first.math.controller.PIDController;
// import edu.wpi.first.math.geometry.Pose2d;
// import edu.wpi.first.math.geometry.Rotation2d;
// import edu.wpi.first.math.geometry.Transform2d;
// import edu.wpi.first.math.geometry.Transform3d;
// import edu.wpi.first.math.geometry.Translation2d;
// import edu.wpi.first.wpilibj2.command.Command;
// import edu.wpi.first.wpilibj2.command.Commands;
// import frc.robot.Constants;

// import java.util.AbstractMap;
// import java.util.Comparator;
// import java.util.List;
// import java.util.Optional;
// import java.util.stream.Stream;

// import org.photonvision.PhotonCamera;
// import org.photonvision.PhotonPoseEstimator;
// import org.photonvision.targeting.PhotonPipelineResult;
// import org.photonvision.targeting.PhotonTrackedTarget;

// import com.ctre.phoenix6.swerve.SwerveRequest;

// import frc.robot.Subsystems.CommandSwerveDrivetrain;


// public class AutoAlignToAprilTagCommand extends Command {
//     private final CommandSwerveDrivetrain drivetrain;
//     private final PIDController xController = new PIDController(2, 0, 0);
//     private final PIDController yController = new PIDController(2, 0, 0);
//     private final PIDController thetaController = new PIDController(0.5, 0, 0);
    
//     private Pose2d targetPose;
//     private int targetID = -1;
//     private final double positionTolerance = 0.15; // meters
//     private final double angleTolerance = 10.0; // degrees

//     public AutoAlignToAprilTagCommand(CommandSwerveDrivetrain drivetrain) {
//         this.drivetrain = drivetrain;
//         thetaController.enableContinuousInput(-180, 180);
//         addRequirements(drivetrain);
//     }

//     @Override
//     public void initialize() {
//         // 重置PID控制器
//         xController.reset();
//         yController.reset();
//         thetaController.reset();
        
//         // 设置目标位置
//         updateTarget();
//     }

//     private void updateTarget() {
//         System.out.println("updateTarget");
//         Optional<PhotonTrackedTarget> closestTarget = findClosestTarget();
//         closestTarget.ifPresent(target -> {
//             targetID = target.getFiducialId();
//             // 获取Tag的全局位置并设置目标位置（在Tag前方0.5米）
//             Pose2d tagPose = Constants.Vision.aprilTagFieldLayout.getTagPose(targetID)
//                 .get().toPose2d();
//                 System.out.println("tagPose " + tagPose);
            
//             // 计算目标位置（保持当前朝向，停在Tag前方0.5米）
//             Translation2d offset = new Translation2d(0.5, 0)
//                 .rotateBy(tagPose.getRotation());
//             targetPose = tagPose.plus(new Transform2d(offset, new Rotation2d()));
//         });
//     }

// private Optional<PhotonTrackedTarget> findClosestTarget() {
//     System.out.println("调用 findClosestTarget() 方法");

//     return drivetrain.cameraEstimators.entrySet().stream()
//         .flatMap(entry -> {
//             PhotonCamera camera = entry.getKey();
//             PhotonPoseEstimator estimator = entry.getValue();

//             System.out.println("检查摄像头: " + camera.getName());

//             // 获取所有未读结果
//             List<PhotonPipelineResult> results = camera.getAllUnreadResults();
//             System.out.println("未读结果数量: " + results.size());

//             if (results.isEmpty()) return Stream.empty();
            
//             // 取最新一帧
//             PhotonPipelineResult latestResult = results.get(results.size() - 1);
//             System.out.println("最新结果包含目标: " + latestResult.hasTargets());

//             if (!latestResult.hasTargets()) return Stream.empty();
            
//             Transform3d robotToCamera = estimator.getRobotToCameraTransform();
//             System.out.println("机器人到摄像头变换: " + robotToCamera);

//             double timestamp = latestResult.getTimestampSeconds();
//             System.out.println("时间戳: " + timestamp);

//             return latestResult.getTargets().stream()
//                 .peek(target -> System.out.println("发现目标 ID: " + target.getFiducialId()))
//                 .filter(target -> {
//                     boolean isValid = Constants.Vision.validTargetIDs.contains(target.getFiducialId());
//                     System.out.println("目标 ID " + target.getFiducialId() + " 是否有效: " + isValid);
//                     return isValid;
//                 })
//                 .map(target -> {
//                     Transform3d cameraToTarget = target.getBestCameraToTarget();
//                     Transform3d robotToTarget = cameraToTarget.plus(robotToCamera);
//                     double distance = robotToTarget.getTranslation().getNorm();
                    
//                     System.out.println("目标 ID " + target.getFiducialId() + " 到机器人的距离: " + distance);
                    
//                     return new TargetData(target, distance, timestamp);
//                 });
//         })
//         .min(Comparator.comparingDouble(TargetData::distance))
//         .map(TargetData::target);
// }


//     // 辅助数据记录类
//     private record TargetData(
//         PhotonTrackedTarget target,  // 目标对象
//         double distance,             // 到目标的距离（米）
//         double timestamp            // 时间戳（秒）
//     ) {}

//     @Override
//     public void execute() {
//         //System.out.println("targetID " + targetID);
//         if (targetID == -1) {
//             updateTarget();
//             return;
//         }

//         Pose2d currentPose = drivetrain.getPose();
//         double xSpeed = xController.calculate(currentPose.getX(), targetPose.getX());
//         double ySpeed = yController.calculate(currentPose.getY(), targetPose.getY());
//         double rotSpeed = thetaController.calculate(
//             currentPose.getRotation().getDegrees(), 
//             targetPose.getRotation().getDegrees()
//         );

//         drivetrain.setControl(new SwerveRequest.FieldCentric()
//             .withVelocityX(xSpeed)
//             .withVelocityY(ySpeed)
//             .withRotationalRate(rotSpeed));
//     }

//     @Override
//     public boolean isFinished() {
//         if (targetID == -1) return true;
        
//         Pose2d currentPose = drivetrain.getPose();
//         return Math.abs(currentPose.getX() - targetPose.getX()) < positionTolerance
//             && Math.abs(currentPose.getY() - targetPose.getY()) < positionTolerance
//             && Math.abs(currentPose.getRotation().minus(targetPose.getRotation()).getDegrees()) < angleTolerance;
//     }

//     @Override
//     public void end(boolean interrupted) {
//         drivetrain.setControl(new SwerveRequest.Idle());
//     }
// }