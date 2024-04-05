package frc.robot.subsystems;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.LimelightHelpers;

public class Limelight extends SubsystemBase {

    Translation2d targetSpeaker;

    public Limelight() {
        // TODO: make sure the target is correct using practice mode on driver station

        // changes speaker side based on color of alliance
        if (DriverStation.getAlliance().isPresent() && DriverStation.getAlliance().get() == Alliance.Blue) {
            targetSpeaker = new Translation2d(0.0, Constants.FieldConstants.kSpeakerY);
        }
        else {
            targetSpeaker = new Translation2d(Constants.FieldConstants.kFieldLength, Constants.FieldConstants.kSpeakerY);
        }
    }

    @Override
    public void periodic() {
        debug();
    }

    // TODO: make sure all of these functions are correct

    /**
     * @return {xCoord, yCoord, timestamp}
     */
    public Translation2d getTranslation2d() { // TODO: make this better i beg you
        LimelightHelpers.PoseEstimate estimate;

        // get pose estimate using megatag2 localization
        if (DriverStation.getAlliance().isPresent() && DriverStation.getAlliance().get() == Alliance.Blue) {
            estimate = LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2("limelight-shooter");
        }
        else {
            estimate = LimelightHelpers.getBotPoseEstimate_wpiRed_MegaTag2("limelight-shooter");
        }

        return estimate.pose.getTranslation();
    }

    public boolean hasTargets(Translation2d translation) {
        if (translation.getX() != 0.0 && translation.getY() != 0.0) {
            return true;
        } else {
            return false;
        }
    }

    public double getXOffset() {
        return LimelightHelpers.getTX("limelight-shooter");
    }

    public Translation2d getTranslationRobotToGoal() {
        Translation2d robotToGoal;

        robotToGoal = targetSpeaker.minus(getTranslation2d());

        return robotToGoal;
    }

    public double getDistanceToTarget(Translation2d robotTranslation) {
        return targetSpeaker.getDistance(robotTranslation);
    }

    // public double getDistToTarget() {
        // Translation2d robotToGoal = getTranslationRobotToGoal();
        // return Math.hypot(robotToGoal.getX(), robotToGoal.getY());
    // }

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

    // public Translation2d getTranslation2d() {
    //     double[] botPose;

    //     if (DriverStation.getAlliance().isPresent() && DriverStation.getAlliance().get() == Alliance.Blue) {
    //         botPose = tableShooter.getEntry("botpose_wpiblue").getDoubleArray(new double[2]);

    //         LimelightHelpers.PoseEstimate mt1 = LimelightHelpers.getBotPoseEstimate_wpiBlue("limelight");
    //     }
    //     else {
    //         botPose = tableShooter.getEntry("botpose_wpired").getDoubleArray(new double[2]);
    //     }

    //     double xPos = botPose[0];
    //     double yPos = botPose[1];


    //     // double filteredX = filterX.calculate(xPos);
    //     // double filteredY = filterY.calculate(yPos);

    //     // return (new Translation2d(filteredX, filteredY));
    //     return new Translation2d(xPos, yPos);
    // }

    private void debug() {
        // SmartDashboard.putNumber("Robot Pose X [LL]", getTranslation2d().getX());
        // SmartDashboard.putNumber("Robot Pose Y [LL]", getTranslation2d().getY());
        // SmartDashboard.putBoolean("Has Targets [LL]", hasTargets(getTranslation2d()));
        // SmartDashboard.putNumber("Translation X Robot To Target [LL]", getTranslationRobotToGoal().getX());
        // SmartDashboard.putNumber("Translation Y Robot To Target [LL]", getTranslationRobotToGoal().getY());
        // SmartDashboard.putNumber("Target X [LL]", targetSpeaker.getX());
        // SmartDashboard.putNumber("Target Y [LL]", targetSpeaker.getY());
    }
}
