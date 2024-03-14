package frc.robot.subsystems;

import java.util.ArrayList;
import java.util.List;
import java.util.Optional;
import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;
import org.photonvision.PhotonUtils;
import org.photonvision.targeting.PhotonTrackedTarget;
import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.filter.MedianFilter;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.util.Util;

public class Vision extends SubsystemBase {    
    private PhotonCamera shooterLL = new PhotonCamera("shooter-limelight"); // TODO: FIND CAMERA NAME
    // private PhotonCamera intakeLL = new PhotonCamera("CAMERA NAME");

    private PIDController pidController = new PIDController(Constants.Vision.kPlimelight, Constants.Vision.kIlimelight, Constants.Vision.kDlimelight);
    
    private AprilTagFieldLayout aprilTagFieldLayout = AprilTagFields.k2024Crescendo.loadAprilTagLayoutField();
    private Transform3d robotToCam = new Transform3d(new Translation3d(0.5, 0.0, 0.5), new Rotation3d(0,0,0)); //Cam mounted facing forward, half a meter forward of center, half a meter up from center // TODO: change transform based on cam mounting
    private PhotonPoseEstimator photonPoseEstimator = new PhotonPoseEstimator(aprilTagFieldLayout, PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR, shooterLL, robotToCam); // there are different types of pose strategies, using lowest ambiguity for now to get clearest april tag in limelight view
    // used to get robot position on field by looking at april tags

    private List<PhotonTrackedTarget> targets;
    private PhotonTrackedTarget target;
    // private PhotonTrackedTarget note;
    // targets for camera

    private SendableChooser<Boolean> cameraMode = new SendableChooser<Boolean>();

    private MedianFilter filterYaw = new MedianFilter(10);
    private MedianFilter filterPitch = new MedianFilter(10);
    // private MedianFilter filterIntake = new MedianFilter(5);
    // filters for shooter and intake

    private ArrayList<Double> xPoseReadings = new ArrayList<Double>(Constants.Vision.kVolatilitySlidingWindowLen);
    private ArrayList<Double> yPoseReadings = new ArrayList<Double>(Constants.Vision.kVolatilitySlidingWindowLen);

    private double totalX = 0.00;
    private double totalY = 0.00;

    private double averageX = 0.00;
    private double averageY = 0.00;
    // volatility

    public Vision() {
        cameraMode.addOption("driver mode", true);
        cameraMode.addOption("obj detection mode", false);

        // intakeLL.setPipelineIndex(0); // TODO: update pipelines to note
        shooterLL.setPipelineIndex(0); // apriltag

        for (int i = 0; i < Constants.Vision.kVolatilitySlidingWindowLen; i++) {
            xPoseReadings.add(0.0);
            yPoseReadings.add(0.0);
        }
    }

    @Override
    public void periodic() {
        Translation2d currentPose = getTranslation2d();

        targets = shooterLL.getLatestResult().getTargets();
        updateTargets();

        SmartDashboard.putNumber("tX [V]", currentPose.getX());
        SmartDashboard.putNumber("tY [V]", currentPose.getY());
        debug();

        // setDriverMode(cameraMode.getSelected());
        // can change back and forth from driver cam and note detection

        // note = intakeLL.getLatestResult().getBestTarget();
        // gets closest note

        totalX += currentPose.getX();
        totalX -= xPoseReadings.get(0);
        averageX = totalX / Constants.Vision.kVolatilitySlidingWindowLen;

        totalY += currentPose.getY();
        totalY -= yPoseReadings.get(0);
        averageY = totalY / Constants.Vision.kVolatilitySlidingWindowLen;

        xPoseReadings.add(currentPose.getX());
        xPoseReadings.remove(0);

        yPoseReadings.add(currentPose.getY());
        yPoseReadings.remove(0);
        // volatility calculations
    }

    /**
     * @return x and y position of the robot on the field
     */
    public Translation2d getTranslation2d() {
        Optional<EstimatedRobotPose> result = photonPoseEstimator.update();

        if (result.isPresent()) {
            Pose2d pose = result.get().estimatedPose.toPose2d();
            // uses april tags to find robot position on field (we have to convert to pose 2d then translation)

            return new Translation2d(pose.getX(), pose.getY());
        } else {
            return new Translation2d(0.0, 0.0);
        }   
    }

    public double getTspeed(double yaw) {
        pidController.setSetpoint(0);
        double tSpeed = pidController.calculate(yaw);

        return tSpeed;
    }

    /**
     * @return distance from the april tag when the robot is in front of the april tag
     */
    public double getStraightDistanceFromTarget() {
        
        if (target != null) {
            double angleToGoalRadians = Math.toRadians(getPitch() + Constants.Vision.kLimelightPitchAngleDegrees);
            double distanceFromTarget = (Constants.Vision.kLimelightPitchAngleDegrees - Constants.Vision.kLimelightHeightMeters) / Math.tan(angleToGoalRadians);

            distanceFromTarget -= 0.0;
            if (distanceFromTarget < 0) {
                distanceFromTarget = 0.0;
            }

        // height from cam to april tag / tan(angle) = distance from cam to april tag
            return distanceFromTarget - Constants.Vision.kdistanceOffset;
        } else {
            return 0.0;
        }
    }

    /**
     * @return distance from the april tag when the robot is at an angle
     */
    public double getDistanceFromTarget() {
        return PhotonUtils.calculateDistanceToTargetMeters(Constants.Vision.kLimelightHeightMeters, Constants.Vision.kSpeakerAprilTagHeightMeters, Constants.Vision.kLimelightPitchAngleDegrees, Units.degreesToRadians(target.getPitch()));
    }

    /**
     * @return Highest axial volatility reading
     */
    public double getOverallVolatility() {
        double volatilityX = getVolatilityAxis(averageX, xPoseReadings);
        double volatilityY = getVolatilityAxis(averageY, yPoseReadings);
        double overallVolatility = Util.max(volatilityX, volatilityY);
        return overallVolatility;
    }

    // /**
    //  * @param driver true for drivermode, false for note detection
    //  * @return boolean to change camera pipeline
    //  */
    // public String setDriverMode(boolean driver) { // for intake LL
    //     if (driver) {
    //         intakeLL.setDriverMode(true);
    //         return "driver mode";
    //     } else {
    //         intakeLL.setPipelineIndex(0); // TODO: change to note pipeline
    //         return "note detection";
    //     }
    // }

    /**
     * @param translation of the robot on the field
     * @return checks if translation is valid before doing math
     */
    public boolean hasTargets(Translation2d translation) {
        if (translation.getX() != 0.0 && translation.getY() != 0.0) {
            return true;
        } else {
            return false;
        }
    }

    public boolean getIsNotVolatile() {
        return getOverallVolatility() < Constants.Vision.kAcceptableVolatilityThreshold;
    }

    public boolean getDriverMode() {
        return shooterLL.getDriverMode();
    }

    public double getYaw() {
        if (target != null) {
            return filterYaw.calculate(target.getYaw());    
        }
        else {
            return 0.0;
        }
    }

    public double getPitch() {
        if (target != null) {
            return filterPitch.calculate(target.getPitch());  
        }
        else {
            return 0.0;
        }
    }

    public double getSkew() {
        if (target != null) {
            return filterPitch.calculate(target.getSkew());
        }
        else {
            return 0.0;
        }
    }

    // public double getIntakeYaw() {
    //     double intakeNoteReading = note.getYaw();
    //     return filterIntake.calculate(intakeNoteReading);
    // }

    /**
     * sets target april tag to the middle speaker tags for correct alignment
     */
    private void updateTargets() {
        for (PhotonTrackedTarget i : targets) {
            if (i.getFiducialId() == 7 || i.getFiducialId() == 4) {
                this.target = i;
            }
            else {
                this.target = null;
            }
        }
    }

    /**
     * @return axial volatility
     */
    private double getVolatilityAxis(double average, ArrayList<Double> llOdometry){
        double volatility = 0.00;

        for (int i = 0; i < Constants.Vision.kVolatilitySlidingWindowLen; i++){
            volatility += Math.abs(average - (double)llOdometry.get(i));
        }
        return volatility;
    }

    private void debug() {
        
        SmartDashboard.putNumber("Distance from Target [V]", getDistanceFromTarget());
        SmartDashboard.putNumber("Distance from Target TEST [V]", getStraightDistanceFromTarget());
        SmartDashboard.putBoolean("isVolatile [V]", getIsNotVolatile());
        // SmartDashboard.putBoolean("Driver mode", getDriverMode());
        SmartDashboard.putNumber("Tag Yaw [V]", getYaw());
        SmartDashboard.putNumber("Tag Skew [V]", getSkew());
        SmartDashboard.putNumber("Tag Pitch [V]", getPitch());
        SmartDashboard.putBoolean("Has Target", shooterLL.getLatestResult().hasTargets());
        SmartDashboard.putNumber("Volatility [V]", getOverallVolatility());
    }
}
