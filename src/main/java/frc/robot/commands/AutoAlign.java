package frc.robot.commands;

import java.util.Set;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.PathConstraints;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;

import frc.robot.Constants.AutoAlign.Side;
import frc.robot.Subsystems.CommandSwerveDrivetrain;

public class AutoAlign {
    private CommandSwerveDrivetrain drivetrain;
    // Create the constraints to use while pathfinding
    private PathConstraints constraints = new PathConstraints(
            2.5, 4.0,
            Units.degreesToRadians(540), Units.degreesToRadians(720));

    public AutoAlign(CommandSwerveDrivetrain drivetrain) {
        this.drivetrain = drivetrain;
    }

    public Pose2d getAlignTarget(Side side) {
        Pose2d currentPose = drivetrain.getState().Pose;
        Pose2d targetPose = currentPose;
        Alliance alliance = DriverStation.getAlliance().orElse(Alliance.Blue);
        int sector = -1;

        if (alliance == Alliance.Red) {
            sector = getHexSector(currentPose.getX(), currentPose.getY(), 13, 4);
            switch (sector) {
                case 0:
                    if (side == Side.LEFT) {
                        // id 7 left
                        targetPose = new Pose2d(14.334, 3.857, Rotation2d.fromDegrees(180));
                    } else {
                        // id 7 right
                        targetPose = new Pose2d(14.334, 4.178, Rotation2d.fromDegrees(180));
                    }
                    break;
                case 1:
                    if (side == Side.LEFT) {
                        // id 8 left
                        targetPose = new Pose2d(13.849, 5.029, Rotation2d.fromDegrees(-120));
                    } else {
                        // id 8 right
                        targetPose = new Pose2d(13.551, 13.551, Rotation2d.fromDegrees(-120));
                    }
                    break;
                case 2:
                    if (side == Side.LEFT) {
                        // id 9 left
                        targetPose = new Pose2d(12.595, 5.193, Rotation2d.fromDegrees(-60));
                    } else {
                        // id 9 right
                        targetPose = new Pose2d(12.297, 5.029, Rotation2d.fromDegrees(-60));
                    }
                    break;
                case 3:
                    if (side == Side.LEFT) {
                        // id 10 left
                        targetPose = new Pose2d(11.819, 4.193, Rotation2d.fromDegrees(0));
                    } else {
                        // id 10 right
                        targetPose = new Pose2d(11.819, 3.842, Rotation2d.fromDegrees(0));
                    }
                    break;
                case 4:
                    if (side == Side.LEFT) {
                        // id 11 left
                        targetPose = new Pose2d(12.289, 3.029, Rotation2d.fromDegrees(60));
                    } else {
                        // id 11 right
                        targetPose = new Pose2d(12.580, 2.850, Rotation2d.fromDegrees(60));
                    }
                    break;
                case 5:
                    if (side == Side.LEFT) {
                        // id 6 left
                        targetPose = new Pose2d(13.543, 2.872, Rotation2d.fromDegrees(120));
                    } else {
                        // id 6 right
                        targetPose = new Pose2d(13.812, 3.029, Rotation2d.fromDegrees(120));
                    }
                    break;
                default:
                    targetPose = currentPose;
                    break;
            }
        } else if (alliance == Alliance.Blue) {
            sector = getHexSector(currentPose.getX(), currentPose.getY(), 4.5, 4);
            switch (sector) {
                case 0:
                    if (side == Side.LEFT) {
                        // id 6 left
                        targetPose = new Pose2d(5.731, 3.865, Rotation2d.fromDegrees(180));
                    } else {
                        // id 6 right
                        targetPose = new Pose2d(5.731, 4.185, Rotation2d.fromDegrees(180));
                    }
                    break;
                case 1:
                    if (side == Side.LEFT) {
                        // id 6 left
                        targetPose = new Pose2d(5.268, 5.021, Rotation2d.fromDegrees(-120));
                    } else {
                        // id 6 right
                        targetPose = new Pose2d(4.977, 5.185, Rotation2d.fromDegrees(-120));
                    }
                    break;
                case 2:
                    if (side == Side.LEFT) {
                        // id 6 left
                        targetPose = new Pose2d(4.014, 5.185, Rotation2d.fromDegrees(-60));
                    } else {
                        // id 6 right
                        targetPose = new Pose2d(3.731, 5.021, Rotation2d.fromDegrees(-60));
                    }
                    break;
                case 3:
                    if (side == Side.LEFT) {
                        targetPose = new Pose2d(3.216, 4.178, Rotation2d.fromDegrees(0));
                    } else {
                        targetPose = new Pose2d(3.216, 3.865, Rotation2d.fromDegrees(0));
                    }
                    break;
                case 4:
                    if (side == Side.LEFT) {
                        // id 6 left
                        targetPose = new Pose2d(3.723, 3.014, Rotation2d.fromDegrees(60));
                    } else {
                        // id 6 right
                        targetPose = new Pose2d(4.007, 2.872, Rotation2d.fromDegrees(60));
                    }
                    break;
                case 5:
                    if (side == Side.LEFT) {
                        // id 6 left
                        targetPose = new Pose2d(4.970, 2.850, Rotation2d.fromDegrees(120));
                    } else {
                        // id 6 right
                        targetPose = new Pose2d(5.253, 3.036, Rotation2d.fromDegrees(120));
                    }
                    break;
                default:
                    targetPose = currentPose;
                    break;
            }
        }
        SmartDashboard.putNumber("autoSector", sector);

        return targetPose;
    }

    public Command getAutoCommand(Side side) {
        return Commands.defer(()->{
            Pose2d targetPose = getAlignTarget(side);
            return AutoBuilder.pathfindToPose(targetPose, constraints, 0.0);
        },Set.of(drivetrain));
    }

    /**
     * 获取六边形区域编号
     * 
     * @param x  机器人x坐标
     * @param y  机器人y坐标
     * @param cx 六边形中心x坐标
     * @param cy 六边形中心y坐标
     * @return 区域编号 0~5
     */
    public int getHexSector(double x, double y, double cx, double cy) {
        double dx = x - cx;
        double dy = y - cy;

        // 得到角度 [-180, 180]
        double angleDeg = Math.toDegrees(Math.atan2(dy, dx));
        if (angleDeg < 0)
            angleDeg += 360; // 转为 [0, 360)

        // 将起点平移到 -30°
        double shiftedAngle = (angleDeg + 30) % 360;

        int sector = (int) Math.floor(shiftedAngle / 60.0); // 每 60° 一区

        SmartDashboard.putNumber("dx", dx);
        SmartDashboard.putNumber("dy", dy);

        return sector; // 0~5
    }

}