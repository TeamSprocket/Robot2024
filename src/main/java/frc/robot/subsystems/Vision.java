package frc.robot.subsystems;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.estimator.PoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructPublisher;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.LimelightHelper;
import frc.robot.LimelightHelper.PoseEstimate;
import frc.robot.subsystems.swerve.CommandSwerveDrivetrain;
import frc.util.Util;

public class Vision extends SubsystemBase {

    // Translation2d targetSpeaker = new Translation2d(0.0, 0.0);
    StructPublisher<Pose2d> publisher = NetworkTableInstance.getDefault()
    .getStructTopic("Robot Pose", Pose2d.struct).publish();

    // int[] validIDs = {4, 7};
    Pose2d lastPose = new Pose2d();
    Pose2d robotPose = new Pose2d();
    Pose2d trueRobotPose = new Pose2d();
    double timestamp;
    double chassisRotationSpeeds;
    double lastXOffset;

    private PIDController pidHeadingLock = new PIDController(0.1, 0, 0.005);

    CommandSwerveDrivetrain swerve;

    public Vision(CommandSwerveDrivetrain swerve) {
        this.swerve = swerve;
        timestamp = 0;
    }

    @Override
    public void periodic() {
        trueRobotPose = logPose();
        publisher.set(trueRobotPose);
        chassisRotationSpeeds = pidHeadingLock.calculate(getXOffset(), 0);
        debug();
    }

    /**
     * @return {xCoord, yCoord, timestamp}
     */
    public Translation2d getTranslation2d() {
        LimelightHelper.PoseEstimate estimate;

        if (LimelightHelper.getTV("limelight")) {
            estimate = LimelightHelper.getBotPoseEstimate_wpiBlue_MegaTag2("limelight");
            lastPose = estimate.pose;
            
            return new Translation2d(estimate.pose.getX(), estimate.pose.getY());
        } else {
            return new Translation2d(lastPose.getX(), lastPose.getY());
        }
    }

    private Pose2d getPose2d() {
        LimelightHelper.PoseEstimate estimate;

        if (LimelightHelper.getTV("limelight")) {
            estimate = LimelightHelper.getBotPoseEstimate_wpiBlue_MegaTag2("limelight");
            lastPose = estimate.pose;

            return estimate.pose;
        } else {
            return lastPose;
        }
    }

    public Pose2d logPose() {
        if (hasTargets()) {
            robotPose = getPose2d();
            swerve.updateOdometry(robotPose);
        } else {
            robotPose = swerve.getPose();
        }
        Pose2d pose = new Pose2d(robotPose.getTranslation(), swerve.getYaw());
        
        return pose;
    }

    public Pose2d getRobotPose() {
        return trueRobotPose;
    }
    
    public ChassisSpeeds getHeadingLockSpeed() {
        return new ChassisSpeeds(0, 0, chassisRotationSpeeds);
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
                lastXOffset = LimelightHelper.getTX("limelight");
                return lastXOffset;
            }
            else {
                return lastXOffset;
            }
        } else {
            return lastXOffset;
        }
    }

    // public double getSpeakerAngle() {
    //     if (LimelightHelper.getTV("limelight")) {
    //         Translation2d robotToSpeakerPose = targetSpeaker.minus(getTranslation2d());
    //         return robotToSpeakerPose.getAngle().getDegrees();
    //     } else {
    //         return 0.0;
    //     }
    // }

    // public void getTargetSpeaker() {

    //     if (DriverStation.getAlliance().isPresent() && DriverStation.getAlliance().get() == Alliance.Blue) {
    //         this.targetSpeaker = new Translation2d(0.0, Constants.FieldConstants.kSpeakerY);
    //     }
    //     else {
    //         this.targetSpeaker = new Translation2d(Constants.FieldConstants.kFieldLength, Constants.FieldConstants.kSpeakerY);
    //     }
    // }

    // public double getDistanceToTarget(Translation2d robotTranslation) { // TODO: find distance offset + add filter if needed
    //     getTargetSpeaker();
    //     return targetSpeaker.getDistance(robotTranslation);
    // }

    // public double getDistToTarget() { // TODO: check which one is more accurate
    //     return Math.hypot(getTranslationRobotToGoal().getX(), getTranslationRobotToGoal().getY());
    // }

    // private Translation2d getTranslationRobotToGoal() {
    //     getTargetSpeaker();
    //     Translation2d robotToGoal;

    //     robotToGoal = targetSpeaker.minus(getTranslation2d());

    //     return robotToGoal;
    // }

    private void debug() {
        SmartDashboard.putNumber("Robot Pose X [VI]", getTranslation2d().getX());
        SmartDashboard.putNumber("Robot Pose Y [VI]", getTranslation2d().getY());
        SmartDashboard.putNumber("PID Heading Lock Output [VI]", chassisRotationSpeeds);
        SmartDashboard.putNumber("X Offset [VI]", getXOffset());
        // SmartDashboard.putBoolean("Has Targets [LL]", hasTargets(getTranslation2d()));
        // SmartDashboard.putNumber("Translation X Robot To Target [LL]", getTranslationRobotToGoal().getX());
        // SmartDashboard.putNumber("Translation Y Robot To Target [LL]", getTranslationRobotToGoal().getY());
        // SmartDashboard.putNumber("Target X [LL]", targetSpeaker.getX());
        // SmartDashboard.putNumber("Target Y [LL]", targetSpeaker.getY());
        // SmartDashboard.putNumber("Dist [LL]", getDistToTarget());
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
