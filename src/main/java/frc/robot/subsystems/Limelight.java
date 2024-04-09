package frc.robot.subsystems;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructPublisher;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.LimelightHelpers;

public class Limelight extends SubsystemBase {

    Translation2d targetSpeaker = new Translation2d(0.0, 0.0);
    StructPublisher<Pose2d> publisher = NetworkTableInstance.getDefault()
    .getStructTopic("Robot Pose", Pose2d.struct).publish();

    public Limelight() {}

    @Override
    public void periodic() {
        publisher.set(getPose2d());
        debug();
    }

    /**
     * @return {xCoord, yCoord, timestamp}
     */
    public Translation2d getTranslation2d() { // TODO: check if we only want coords from blue side
        LimelightHelpers.PoseEstimate estimate;

        if (LimelightHelpers.getTV("limelight")) {
            // get pose estimate using megatag2 localization
            if (DriverStation.getAlliance().isPresent() && DriverStation.getAlliance().get() == Alliance.Blue) {
                estimate = LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2("limelight");
            }
            else {
                estimate = LimelightHelpers.getBotPoseEstimate_wpiRed_MegaTag2("limelight");
            }
            return new Translation2d(estimate.pose.getX(), estimate.pose.getY());
        } else {
            return new Translation2d(0.0, 0.0);
        }
    }

    public Pose2d getPose2d() { // DEBUG ONLY
        LimelightHelpers.PoseEstimate estimate;

        if (LimelightHelpers.getTV("limelight")) {
            // get pose estimate using megatag2 localization
            if (DriverStation.getAlliance().isPresent() && DriverStation.getAlliance().get() == Alliance.Blue) {
                estimate = LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2("limelight");
            }
            else {
                estimate = LimelightHelpers.getBotPoseEstimate_wpiRed_MegaTag2("limelight");
            }

            return estimate.pose;
        } else {
            return new Pose2d();
        }
    }

    public boolean hasTargets() {
        return LimelightHelpers.getTV("limelight");
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
        if (LimelightHelpers.getTV("limelight")) {
            if (LimelightHelpers.getFiducialID("limelight") == 7 || LimelightHelpers.getFiducialID("limelight") == 4) {
                return LimelightHelpers.getTX("limelight");
            }
            else {
                return 0.0;
            }
        } else {
            return 0.0;
        }
    }


    // public double getSpeakerAngleOffset() {
    //     Translation2d speakerPoint = targetSpeaker;
    //     Translation2d currentPos = getTranslation2d();

    //     // Driver POV NOT FIELD POV
    //     double speakerX = targetSpeaker.getX();
    //     double speakerY = targetSpeaker.getY();
    //     double botX = targetSpeaker.getX();
    //     double speakerY = targetSpeaker.getY();

    // }


    public void getTargetSpeaker() {

        if (DriverStation.getAlliance().isPresent() && DriverStation.getAlliance().get() == Alliance.Blue) {
            this.targetSpeaker = new Translation2d(0.0, Constants.FieldConstants.kSpeakerY);
        }
        else {
            this.targetSpeaker = new Translation2d(Constants.FieldConstants.kFieldLength, Constants.FieldConstants.kSpeakerY);
        }
    }

    public double getDistanceToTarget(Translation2d robotTranslation) { // TODO: find distance offset + add filter if needed
        getTargetSpeaker();
        return targetSpeaker.getDistance(robotTranslation);
    }

    public double getDistToTarget() { // TODO: check which one is more accurate
        return Math.hypot(getTranslationRobotToGoal().getX(), getTranslationRobotToGoal().getY());
    }

    private Translation2d getTranslationRobotToGoal() {
        getTargetSpeaker();
        Translation2d robotToGoal;

        robotToGoal = targetSpeaker.minus(getTranslation2d());

        return robotToGoal;
    }

    private void debug() {
        SmartDashboard.putNumber("Robot Pose X [LL]", getTranslation2d().getX());
        SmartDashboard.putNumber("Robot Pose Y [LL]", getTranslation2d().getY());
        // SmartDashboard.putBoolean("Has Targets [LL]", hasTargets(getTranslation2d()));
        // SmartDashboard.putNumber("Translation X Robot To Target [LL]", getTranslationRobotToGoal().getX());
        // SmartDashboard.putNumber("Translation Y Robot To Target [LL]", getTranslationRobotToGoal().getY());
        SmartDashboard.putNumber("Target X [LL]", targetSpeaker.getX());
        SmartDashboard.putNumber("Target Y [LL]", targetSpeaker.getY());
        SmartDashboard.putNumber("Dist [LL]", getDistToTarget());
    }

    // public Translation2d getTranslationRobotToGoal() {
        // double x = 0.0;
        // double y = 0.0;

        // if (DriverStation.getAlliance().isPresent() && DriverStation.getAlliance().get() == Alliance.Blue) {
        //     x = Math.abs(Constants.Limelight.speakerBlue.getX() - getTranslation2d().getX()); 
        //     y = Math.abs(Constants.Limelight.speakerBlue.getY() - getTranslation2d().getY());
        // }
        // else {
        //     x = Math.abs(Constants.Limelight.speakerRed.getX() - getTranslation2d().getX());
        //     y = Math.abs(Constants.Limelight.speakerRed.getY() - getTranslation2d().getY());
        // }

        // return Math.atan(y/x);
    // }
}
