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
import frc.util.LimelightHelper;
import frc.util.Util;

public class Vision extends SubsystemBase {

    StructPublisher<Pose2d> publisher = NetworkTableInstance.getDefault().getStructTopic("Robot Pose", Pose2d.struct).publish();

    private int[] blueReefAprilTag = {17, 18, 19, 20, 21, 22};
    private int[] redReefAprilTag = {6, 7, 8, 9, 10, 11};

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
        if (LimelightHelper.getTV("")) {
            if (DriverStation.getAlliance().isPresent() && DriverStation.getAlliance().get() == Alliance.Blue) {
                estimate = LimelightHelper.getBotPoseEstimate_wpiBlue_MegaTag2("");
            }
            else {
                estimate = LimelightHelper.getBotPoseEstimate_wpiRed_MegaTag2("");
            }
            return new Translation2d(estimate.pose.getX(), estimate.pose.getY());
        } else {
            return new Translation2d(0.0, 0.0);
        }
    }

    public Pose2d getPose2d() {
        LimelightHelper.PoseEstimate estimate;

        if (LimelightHelper.getTV("")) {
            if (DriverStation.getAlliance().isPresent() && DriverStation.getAlliance().get() == Alliance.Blue) {
                estimate = LimelightHelper.getBotPoseEstimate_wpiBlue_MegaTag2("");
            }
            else {
                estimate = LimelightHelper.getBotPoseEstimate_wpiRed_MegaTag2("");
            }

            return estimate.pose;
        } else {
            return new Pose2d();
        }
    }

    public boolean hasTargets() {
        return LimelightHelper.getTV("");
    }

    public double getXOffset() {
        if (hasTargets()){
            return LimelightHelper.getTX("");
        } else {
            return 0.0;
        }
    }

    public double getYOffset() {
        if (hasTargets()) {
            return LimelightHelper.getTY("");
        } else {
            return 0.0;
        }
    }
     
    public boolean hasReefTargets(){
        if(LimelightHelper.getTV("") ){
            for (int reefID : redReefAprilTag) {
                if (LimelightHelper.getFiducialID("") == reefID) return true;
            }
            for (int reefID : blueReefAprilTag) {
                if (LimelightHelper.getFiducialID("") == reefID) return true;
            }
        } 
        return false;
    }
    

    private void debug() {
        // SmartDashboard.putNumber("Robot Pose X [VI]", getTranslation2d().getX());
        // SmartDashboard.putNumber("Robot Pose Y [VI]", getTranslation2d().getY());
        // SmartDashboard.putBoolean("Vision Target [VI]", hasTargets());
        // SmartDashboard.putNumber("TX Offset [VI]", getXOffset());
        // SmartDashboard.putNumber("TY Offset [VI]", getYOffset());
        SmartDashboard.putBoolean("Has Reef Target [VI]", hasReefTargets());
     }

}
