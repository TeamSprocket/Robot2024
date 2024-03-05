package frc.robot.subsystems;

import java.util.ArrayList;
import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;
import org.photonvision.targeting.PhotonTrackedTarget;
import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.util.Util;

public class Vision extends SubsystemBase {
    
    private PhotonCamera shooterLL = new PhotonCamera("CAMERA NAME"); // TODO: FIND CAMERA NAME
    private PhotonCamera intakeLL = new PhotonCamera("CAMERA NAME");
    
    private AprilTagFieldLayout aprilTagFieldLayout = AprilTagFields.k2024Crescendo.loadAprilTagLayoutField();
    private Transform3d robotToCam = new Transform3d(new Translation3d(0.5, 0.0, 0.5), new Rotation3d(0,0,0)); //Cam mounted facing forward, half a meter forward of center, half a meter up from center.
    private PhotonPoseEstimator photonPoseEstimator = new PhotonPoseEstimator(aprilTagFieldLayout, PoseStrategy.LOWEST_AMBIGUITY, shooterLL, robotToCam);

    private PhotonTrackedTarget target = new PhotonTrackedTarget();
    private SendableChooser<Boolean> cameraMode = new SendableChooser<Boolean>();

    private ArrayList<Double> xPoseReadings = new ArrayList<Double>(50);
    private ArrayList<Double> yPoseReadings = new ArrayList<Double>(50);

    private double totalX = 0.00;
    private double totalY = 0.00;

    private double averageX = 0.00;
    private double averageY = 0.00;

    public Vision() {
        cameraMode.addOption("driver mode", true);
        cameraMode.addOption("obj detection mode", false);

        intakeLL.setPipelineIndex(0); //note
        shooterLL.setPipelineIndex(0); //apriltag
    }

    @Override
    public void periodic() {

        SmartDashboard.putNumber("distance from target", getDistanceFromTarget());
        SmartDashboard.putBoolean("is volatile", getIsNotVolatile());
        SmartDashboard.putBoolean("driver mode", getDriverMode());
        SmartDashboard.putNumber("yaw", getYaw());
        SmartDashboard.putNumber("pitch", getPitch());

        setDriverMode(cameraMode.getSelected());

        Translation2d currentPose = getTranslation2d();
        target = shooterLL.getLatestResult().getBestTarget();

        totalX += currentPose.getX();
        totalX -= xPoseReadings.get(0);
        averageX = totalX / 50;

        totalY += currentPose.getY();
        totalY -= yPoseReadings.get(0);
        averageY = totalY / 50;

        xPoseReadings.add(currentPose.getX());
        xPoseReadings.remove(0);

        yPoseReadings.add(currentPose.getY());
        yPoseReadings.remove(0);
    }

    /**
     * @return x and y position of the robot on the field
     */
    public Translation2d getTranslation2d() {

        EstimatedRobotPose result = photonPoseEstimator.update().get();
        Pose2d pose = result.estimatedPose.toPose2d();

        return new Translation2d(pose.getX(), pose.getY());
    }

    /**
     * @return distance from the april tag
     */
    public double getDistanceFromTarget() {
        double angleToGoalRadians = Math.toRadians(getPitch() + Constants.Limelight.limelightMountAngleDegrees);
        double distanceFromTarget = (Constants.Limelight.goalHeightInches - Constants.Limelight.limelightHeightInches) / Math.tan(angleToGoalRadians);

        return distanceFromTarget;
    }

    /**
     * @return individual volatility for x & y
     */
    public double getVolatilityAxis(double average, ArrayList<Double> llOdometry){
        double volatility = 0.00;

        for (int i = 0; i < 50; i++){
            volatility += Math.abs(average - (double)llOdometry.get(i));
        }
        return volatility;
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

    /**
     * @param driver true for drivermode, false for note detection
     * @return boolean to change camera pipeline
     */
    public String setDriverMode(boolean driver) { // for intake LL
        if (driver) {
            intakeLL.setDriverMode(true);
            return "driver mode";
        } else {
            intakeLL.setPipelineIndex(0); // TODO: change to note pipeline
            return "note detection";
        }
    }

    public boolean hasTargets(Translation2d translation) {
        if (translation.getX() != 0.0 && translation.getY() != 0.0) {
            return true;
        } else {
            return false;
        }
    }

    public boolean getIsNotVolatile() {
        return getOverallVolatility() < Constants.Limelight.kAcceptableVolatilityThreshold;
    }

    public boolean getDriverMode() {
        return shooterLL.getDriverMode();
    }

    public double getYaw() {
        return target.getYaw();
    }

    public double getPitch() {
        return target.getPitch();
    }

}
