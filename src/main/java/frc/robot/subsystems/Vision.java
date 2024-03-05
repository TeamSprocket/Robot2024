package frc.robot.subsystems;

import org.photonvision.PhotonCamera;
import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Vision extends SubsystemBase {
    
    PhotonCamera camera = new PhotonCamera("CAMERA NAME"); // TODO: FIND CAMERA NAME
    private PhotonTrackedTarget target;
    private SendableChooser<Boolean> cameraMode = new SendableChooser<Boolean>();

    public Vision() {
        cameraMode.addOption("driver mode", true);
        cameraMode.addOption("obj detection mode", false);
    }

    @Override
    public void periodic() {
        target = camera.getLatestResult().getBestTarget();

        SmartDashboard.putNumber("distance from target", getDistanceFromTarget());
        SmartDashboard.putBoolean("is volatile", isVolatile());
        SmartDashboard.putBoolean("driver mode", getDriverMode());
        SmartDashboard.putNumber("yaw", getYaw());
        SmartDashboard.putNumber("pitch", getPitch());

        setDriverMode(cameraMode.getSelected());
    }

    public double getDistanceFromTarget() {
        double angleToGoalRadians = Math.toRadians(getPitch() + Constants.Limelight.limelightMountAngleDegrees);
        double distanceFromTarget = (Constants.Limelight.goalHeightInches - Constants.Limelight.limelightHeightInches) / Math.tan(angleToGoalRadians);

        return distanceFromTarget;
    }

    public String setDriverMode(boolean driver) { // for intake LL
        if (driver) {
            camera.setDriverMode(true);
            return "driver mode";
        } else {
            camera.setPipelineIndex(0); // TODO: change to note pipeline
            return "note detection";
        }
    }

    public boolean getDriverMode() {
        return camera.getDriverMode();
    }

    public boolean isVolatile() {
        return target.getPoseAmbiguity() > Constants.Limelight.kAcceptableVolatilityThreshold;
    }

    public double getYaw() {
        return target.getYaw();
    }

    public double getPitch() {
        return target.getPitch();
    }

}
