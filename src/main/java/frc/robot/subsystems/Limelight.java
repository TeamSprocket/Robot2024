package frc.robot.subsystems;

import java.sql.Driver;
import java.util.Optional;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.filter.LinearFilter;
import edu.wpi.first.math.filter.MedianFilter;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.networktables.DoubleArraySubscriber;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Limelight extends SubsystemBase {
    private MedianFilter filterX = new MedianFilter(10);
    private MedianFilter filterY = new MedianFilter(10);

    public Limelight() {

    }

    @Override
    public void periodic() {
        SmartDashboard.putNumber("Filtered X Pos", getTranslation2d().getX());
        SmartDashboard.putNumber("Filtered Y Pos", getTranslation2d().getY());
    }

    /**
     * @return Field relative coordinates of the robot
     */
    public Translation2d getTranslation2d() {

        double[] botPoseInfo;

        if (DriverStation.getAlliance() == Optional.of(Alliance.Blue)) {
            botPoseInfo = NetworkTableInstance.getDefault().getTable("limelight").getEntry("botpose_wpiblue").getDoubleArray(new double[6]);
        } else {
            botPoseInfo = NetworkTableInstance.getDefault().getTable("limelight").getEntry("botpose_wpired").getDoubleArray(new double[6]);
        }

        double xPos = filterX.calculate(botPoseInfo[0]);
        double yPos = filterY.calculate(botPoseInfo[1]);

        return new Translation2d(xPos, yPos);
        
    }
    



}
