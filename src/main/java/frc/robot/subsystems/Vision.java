package frc.robot.subsystems;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructPublisher;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.LimelightHelper;
import frc.util.Util;

public class Vision extends SubsystemBase {

    StructPublisher<Pose2d> publisher = NetworkTableInstance.getDefault().getStructTopic("Robot Pose", Pose2d.struct).publish();

    public Vision() {}

    @Override
    public void periodic() {
        publisher.set(getPose2d());
        debug();
    }

    /**
     * @return {xCoord, yCoord, timestamp}
     */
    public Translation2d getTranslation2d() {
        LimelightHelper.PoseEstimate estimate;
        if (LimelightHelper.getTV("limelight")) {
            if (DriverStation.getAlliance().isPresent() && DriverStation.getAlliance().get() == Alliance.Blue) {
                estimate = LimelightHelper.getBotPoseEstimate_wpiBlue_MegaTag2("limelight");
            }
            else {
                estimate = LimelightHelper.getBotPoseEstimate_wpiRed_MegaTag2("limelight");
            }
            return new Translation2d(estimate.pose.getX(), estimate.pose.getY());
        } else {
            return new Translation2d(0.0, 0.0);
        }
    }

    public Pose2d getPose2d() {
        LimelightHelper.PoseEstimate estimate;

        if (LimelightHelper.getTV("limelight")) {
            if (DriverStation.getAlliance().isPresent() && DriverStation.getAlliance().get() == Alliance.Blue) {
                estimate = LimelightHelper.getBotPoseEstimate_wpiBlue_MegaTag2("limelight");
            }
            else {
                estimate = LimelightHelper.getBotPoseEstimate_wpiRed_MegaTag2("limelight");
            }

            return estimate.pose;
        } else {
            return new Pose2d();
        }
    }

    public boolean hasTargets() {
        return LimelightHelper.getTV("limelight");
    }

    public double getXOffset() {
        if (hasTargets()){
            return LimelightHelper.getTX("limelight");
        } else {
            return 0.0;
        }

    }


    private void debug() {
        SmartDashboard.putNumber("Robot Pose X [VI]", getTranslation2d().getX());
        SmartDashboard.putNumber("Robot Pose Y [VI]", getTranslation2d().getY());

     }

}
