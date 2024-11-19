package frc.robot.subsystems;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructPublisher;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.swerve.CommandSwerveDrivetrain;
import frc.util.LimelightHelper;

public class Vision extends SubsystemBase {

    StructPublisher<Pose2d> publisher = NetworkTableInstance.getDefault()
    .getStructTopic("Robot Pose", Pose2d.struct).publish();

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
        pidHeadingLock.setTolerance(0.01);
    }

    @Override
    public void periodic() {
        chassisRotationSpeeds = pidHeadingLock.calculate(getXOffset(), 0);
        debug();
    }

    public boolean isAligned() {
        if (pidHeadingLock.atSetpoint()) {
            return true;
        } else {
            return false;
        }
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

    private void debug() {
        SmartDashboard.putNumber("Robot Pose X [VI]", getTranslation2d().getX());
        SmartDashboard.putNumber("Robot Pose Y [VI]", getTranslation2d().getY());
        SmartDashboard.putNumber("PID Heading Lock Output [VI]", chassisRotationSpeeds);
        SmartDashboard.putNumber("X Offset [VI]", getXOffset());

        SmartDashboard.putBoolean("Swerve Align", isAligned());
    }
}