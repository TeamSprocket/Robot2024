package frc.robot.subsystems;

import java.util.List;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.commands.FollowPathCommand;
import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;
import com.pathplanner.lib.path.GoalEndState;
import com.pathplanner.lib.path.IdealStartingState;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.path.Waypoint;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructPublisher;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.subsystems.swerve.CommandSwerveDrivetrain;
import frc.util.LimelightHelper;
import frc.util.Util;

public class Vision extends SubsystemBase {
    StructPublisher<Pose2d> publisher = NetworkTableInstance.getDefault().getStructTopic("Endpoint", Pose2d.struct).publish();

    private int[] blueReefAprilTag = {17, 18, 19, 20, 21, 22};
    private int[] redReefAprilTag = {6, 7, 8, 9, 10, 11};

    CommandSwerveDrivetrain drivetrain;

    String name = "limelight-front";

    Pose2d lastPose = new Pose2d();

    LimelightHelper.PoseEstimate estimate;

    public Vision(CommandSwerveDrivetrain drive) {
        drivetrain = drive;
    }

    @Override
    public void periodic() {
        publisher.set(drivetrain.getAutoBuilderPose());
        debug();
        updatePose();
    }

    /**
     * @return {xCoord, yCoord, timestamp}
     */
    public Translation2d getTranslation2d() {
        LimelightHelper.PoseEstimate estimate;
        if (LimelightHelper.getTV(name)) {
            if (DriverStation.getAlliance().isPresent() && DriverStation.getAlliance().get() == Alliance.Blue) {
                estimate = LimelightHelper.getBotPoseEstimate_wpiBlue_MegaTag2(name);
            }
            else {
                estimate = LimelightHelper.getBotPoseEstimate_wpiRed_MegaTag2(name);
            }
            return new Translation2d(estimate.pose.getX(), estimate.pose.getY());
        } else {
            return new Translation2d(0.0, 0.0);
        }
    }

    public Pose2d getPose2d() {
        if (LimelightHelper.getTV("limelight-front")) {
            estimate = LimelightHelper.getBotPoseEstimate_wpiBlue("limelight-front");
            lastPose = estimate.pose;
            return estimate.pose;
        } else {
            return lastPose;
        }
    }

    public boolean hasTargets() {
        return LimelightHelper.getTV(name);
    }

    public double getXOffset() {
        if (hasTargets()){
            return LimelightHelper.getTX(name);
        } else {
            return 0.0;
        }
    }

    public double getYOffset() {
        if (hasTargets()) {
            return LimelightHelper.getTY(name);
        } else {
            return 0.0;
        }
    }
     
    public boolean hasReefTargets(){
        if(LimelightHelper.getTV(name) ){
            for (int reefID : redReefAprilTag) {
                if (LimelightHelper.getFiducialID(name) == reefID) return true;
            }
            for (int reefID : blueReefAprilTag) {
                if (LimelightHelper.getFiducialID(name) == reefID) return true;
            }
        } 
        return false;
    }

    public PathPlannerPath getAlignPathLeft() {
        double fiducialID = LimelightHelper.getFiducialID(name);
        Pose2d endpoint = new Pose2d();
        switch ((int)fiducialID) {
            case 17:
                endpoint = Constants.Vision.poseAlignBlueLeft17;
            case 18:
                endpoint = Constants.Vision.poseAlignBlueLeft18;
            case 19:
                endpoint = Constants.Vision.poseAlignBlueLeft19;
            case 20:
                endpoint = Constants.Vision.poseAlignBlueLeft20;
            case 21:
                endpoint = Constants.Vision.poseAlignBlueLeft21;
            case 22:
                endpoint = Constants.Vision.poseAlignBlueLeft22;
        }
        

        List<Waypoint> waypoints = PathPlannerPath.waypointsFromPoses(
            getPose2d(),
            endpoint
        );

        PathPlannerPath path = new PathPlannerPath(
            waypoints,
            new PathConstraints(4, 2, 4, 2),
            null,
            new GoalEndState(0.0, endpoint.getRotation())
        );
        return path;
    }

    public Command getAlignPathRight() {
        double fiducialID = LimelightHelper.getFiducialID(name);
        Pose2d endpoint = new Pose2d();
        switch ((int)fiducialID) {
            case 17:
                endpoint = Constants.Vision.poseAlignBlueRight17;
                break;
            case 18:
                endpoint = Constants.Vision.poseAlignBlueRight18;
                break;
            case 19:
                endpoint = Constants.Vision.poseAlignBlueRight19;
                break;
            case 20:
                endpoint = Constants.Vision.poseAlignBlueRight20;
                break;
            case 21:
                endpoint = Constants.Vision.poseAlignBlueRight21;
                break;
            case 22:
                endpoint = Constants.Vision.poseAlignBlueRight22;
                break;
            default:
                endpoint = drivetrain.getAutoBuilderPose();
                break;
        }
    
        // endpoint = Constants.Vision.testPose;
        // System.out.println("XXXXXXXXXXXXXXXXXXXXXXXXXXX:" + endpoint.getX());

        Command path = AutoBuilder.pathfindToPose(
            endpoint,
            new PathConstraints(4, 2, 4, 2), 
            0.0
        );

        return path;
    }

    public Pose2d updatePose() {
        if (hasTargets()) {
            Pose2d pose = getPose2d();
            drivetrain.updateOdometry(pose);
            return pose;
        } else {
            return getPose2d();
        }
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
