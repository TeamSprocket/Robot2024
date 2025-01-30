package frc.robot.subsystems;

import java.util.List;

import com.fasterxml.jackson.databind.node.POJONode;
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
import edu.wpi.first.math.geometry.Transform2d;
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

    Pose2d endpointL = new Pose2d();
    Pose2d endpointR = new Pose2d();
    Pose2d endpoint = new Pose2d();

    Command pathL;
    Command pathR;
    Command path;

    double distToAprilLeft = 0.0;
    double distToAprilRight = 0.0;
    boolean updateFirst = true;

    double fiducialID;

    public Vision(CommandSwerveDrivetrain drive) {
        drivetrain = drive;
    }

    @Override
    public void periodic() {
        publisher.set(drivetrain.getAutoBuilderPose());
        debug();
        if (hasTargets() && updateFirst) {
            updatePose();
            updateFirst = false;
        } else if (!hasTargets() && !updateFirst) {
            updateFirst = true;
        }
        
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

    // TEST LATER

    public Command getAlignCommand() {
        
        fiducialID = LimelightHelper.getFiducialID(name);
        endpoint = new Pose2d();
        switch ((int)fiducialID) {
            case 17:
                endpoint = getClosestEndpoint(Constants.Vision.poseAlignBlueLeft17, Constants.Vision.poseAlignBlueLeft17);
                break;
            case 18:
                endpoint = getClosestEndpoint(Constants.Vision.poseAlignBlueLeft18, Constants.Vision.poseAlignBlueLeft18);
                break;
            case 19:
                endpoint = getClosestEndpoint(Constants.Vision.poseAlignBlueLeft19, Constants.Vision.poseAlignBlueLeft19);
                break;
            case 20:
                endpoint = getClosestEndpoint(Constants.Vision.poseAlignBlueLeft20, Constants.Vision.poseAlignBlueLeft20);
                break;
            case 21:
                endpoint = getClosestEndpoint(Constants.Vision.poseAlignBlueLeft21, Constants.Vision.poseAlignBlueLeft21);
                break;
            case 22:
                endpoint = getClosestEndpoint(Constants.Vision.poseAlignBlueLeft22, Constants.Vision.poseAlignBlueLeft22);
                break;
            case -1:
                endpoint = drivetrain.getAutoBuilderPose();
                break;
        }

        path = AutoBuilder.pathfindToPose(
            endpoint,
            new PathConstraints(3, 2, 4, 2), 
            0.0
        );

        return path;
    }

    public Pose2d getClosestEndpoint(Pose2d endpointLeft, Pose2d endpointRight) {
        Transform2d transformToAprilTagLeft = new Transform2d(drivetrain.getAutoBuilderPose(), endpointLeft);
        Transform2d transformToAprilTagRight = new Transform2d(drivetrain.getAutoBuilderPose(), endpointRight);

        distToAprilLeft = Math.hypot(transformToAprilTagLeft.getX(), transformToAprilTagLeft.getY());
        distToAprilRight = Math.hypot(transformToAprilTagRight.getX(), transformToAprilTagRight.getY());

        if (distToAprilLeft > distToAprilRight) {
            return endpointRight;
        } else if (distToAprilLeft < distToAprilRight) {
            return endpointLeft;
        } else {
            return endpointLeft;
        }
    }

    // TEST LATER ^^

    public Command getAlignPathLeft() {
        updatePose();
        fiducialID = LimelightHelper.getFiducialID(name);
        endpointL = new Pose2d();
        switch ((int)fiducialID) {
            case 17:
                endpointL = Constants.Vision.poseAlignBlueLeft17;
                break;
            case 18:
                endpointL = Constants.Vision.poseAlignBlueLeft18;
                break;
            case 19:
                endpointL = Constants.Vision.poseAlignBlueLeft19;
                break;
            case 20:
                endpointL = Constants.Vision.poseAlignBlueLeft20;
                break;
            case 21:
                endpointL = Constants.Vision.poseAlignBlueLeft21;
                break;
            case 22:
                endpointL = Constants.Vision.poseAlignBlueLeft22;
                break;
            case -1:
                endpointL = drivetrain.getAutoBuilderPose();
                break;
        }
    

        pathL = AutoBuilder.pathfindToPose(
            endpointL,
            new PathConstraints(3, 2, 4, 2), 
            0.0
        );

        return pathL;
    }

    public Command getAlignPathRight() {
        updatePose();
        fiducialID = LimelightHelper.getFiducialID(name);
        endpointR = new Pose2d();
        switch ((int)fiducialID) {
            case 17:
                endpointR = Constants.Vision.poseAlignBlueRight17;
                break;
            case 18:
                endpointR = Constants.Vision.poseAlignBlueRight18;
                break;
            case 19:
                endpointR = Constants.Vision.poseAlignBlueRight19;
                break;
            case 20:
                endpointR = Constants.Vision.poseAlignBlueRight20;
                break;
            case 21:
                endpointR = Constants.Vision.poseAlignBlueRight21;
                break;
            case 22:
                endpointR = Constants.Vision.poseAlignBlueRight22;
                break;
            case -1:
                endpointR = drivetrain.getAutoBuilderPose();
                break;
        }

        pathR = AutoBuilder.pathfindToPose(
            endpointR,
            new PathConstraints(3, 2, 4, 2), 
            0.0
        );

        return pathR;
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
        SmartDashboard.putBoolean("Has Reef Target [VI]", hasReefTargets());
        SmartDashboard.putNumber("FUDICIAL ID", LimelightHelper.getFiducialID(name));
        SmartDashboard.putNumber("END XL",endpointL.getX());
        SmartDashboard.putNumber("END YL",endpointL.getY());
        SmartDashboard.putNumber("END XR",endpointR.getX());
        SmartDashboard.putNumber("END YR",endpointR.getY());
        SmartDashboard.putNumber("dist to left", distToAprilLeft);
        SmartDashboard.putNumber("dist to right", distToAprilRight);
     }

}
