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
import frc.robot.LimelightHelper.PoseEstimate;
import frc.util.Util;

public class Vision extends SubsystemBase {

    // Translation2d targetSpeaker = new Translation2d(0.0, 0.0);
    StructPublisher<Pose2d> publisher = NetworkTableInstance.getDefault()
    .getStructTopic("Robot Pose", Pose2d.struct).publish();

    // int[] validIDs = {4, 7};

    public Vision() {}

    public Translation2d getTranslation2d() {
        // ask why poseestimate
        PoseEstimate poseEstimate;
        if (LimelightHelper.getTV("limelight")) {
            // ask why we use MegaTag and how they work
            poseEstimate = LimelightHelper.getBotPoseEstimate_wpiBlue_MegaTag2("limelight");

            return new Translation2d(poseEstimate.pose.getX(), poseEstimate.pose.getY());
        } else {
            return new Translation2d(0, 0);
        }
    }

    public boolean hasTargets() {
        if (LimelightHelper.getTV("limelight")) {
            if (getTranslation2d().getX() != 0 && getTranslation2d().getY() != 0) return true;
            else return false;
        } else return false;
    }

}
