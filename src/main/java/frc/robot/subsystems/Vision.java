package frc.robot.subsystems;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.estimator.PoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructPublisher;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.util.LimelightHelper;
import frc.util.Util;
import frc.util.LimelightHelper.PoseEstimate;

public class Vision extends SubsystemBase {

    StructPublisher<Pose2d> publisher = NetworkTableInstance.getDefault()
    .getStructTopic("Robot Pose", Pose2d.struct).publish();

    public Vision() {}

    @Override
    public void periodic() {
        publisher.set(getPose2d());
    }

    public Pose2d getPose2d() {
        LimelightHelper.PoseEstimate estimate;

        if (LimelightHelper.getTV("limelight")) {
                estimate = LimelightHelper.getBotPoseEstimate_wpiBlue_MegaTag2("limelight");
            return estimate.pose;
        } else {
            return new Pose2d();
        }
    }

    public boolean hasTargets() {
        return LimelightHelper.getTV("limelight");
    }

    public boolean hasTargets(Translation2d translation) {
        if (translation.getX() != 0.0 && translation.getY() != 0.0) {
            return true;
        } else {
            return false;
        }
    }

    /**
     * @return Offset of limelight crosshair center to fiducials in DEGREES
     */     
    public double getXOffset() {
        if (LimelightHelper.getTV("limelight")) {
            if (LimelightHelper.getFiducialID("limelight") == 7 || LimelightHelper.getFiducialID("limelight") == 4) {
                return LimelightHelper.getTX("limelight"); 
            }
            else {
                return 0.0;
            }
        } else {
            return 0.0;
        }
    }
}