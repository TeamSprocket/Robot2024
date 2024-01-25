package frc.robot.subsystems;

import java.util.Optional;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.filter.MedianFilter;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
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

    private NetworkTable table = NetworkTableInstance.getDefault().getTable("limelight");

    private SwerveDrive swerveDrive;
    private SwerveDriveOdometry odometry;
    private PIDController pidController;
    private double pose[];

    private MedianFilter filter;
    private double filteredX;
    private double filteredY;

    public Limelight(SwerveDrive swerveDrive) {

        this.swerveDrive = swerveDrive;

        pidController = new PIDController(Constants.Drivetrain.kLimelightAlignP, Constants.Drivetrain.kLimelightAlignI, Constants.Drivetrain.kLimelightAlignD);
        pidController.setSetpoint(0);
        odometry = new SwerveDriveOdometry(Constants.Drivetrain.kDriveKinematics, new Rotation2d(swerveDrive.getHeading()), swerveDrive.getModulePositions());

        filter.reset();
        filter = new MedianFilter(3);
    }

    @Override
    public void periodic() {
        filterPose();

        SmartDashboard.putNumber("Pose X", table.getEntry("tx").getDouble(0.0));
        SmartDashboard.putNumber("Pose Y", table.getEntry("ty").getDouble(0.0));
        SmartDashboard.putNumber("Filtered X", filteredX);
        SmartDashboard.putNumber("Filtered Y", filteredY);
    }

    public void filterPose() { // for testing purposes!!
        DoubleArraySubscriber botPose = table.getDoubleArrayTopic("botpose").subscribe(new double[] {});
        pose = botPose.get();
        filteredX = filter.calculate(pose[0]);
        filteredY = filter.calculate(pose[1]);
    }

    public Pose2d getPose() {
        DoubleArraySubscriber botPose = table.getDoubleArrayTopic("botpose").subscribe(new double[] {});
        int hasTarget = (int)table.getEntry("tv").getInteger(0);
        Optional<Alliance> color = DriverStation.getAlliance();
        
        if (hasTarget == 0) {

            Pose2d robotPose = odometry.update(new Rotation2d(swerveDrive.getHeading()), swerveDrive.getModulePositions());
            return (new Pose2d(robotPose.getX(), robotPose.getY(), new Rotation2d(robotPose.getRotation().getRadians())));
        } 
        else {

            if (color == Optional.of(Alliance.Blue)) {

                botPose = table.getDoubleArrayTopic("botpose_wpiblue").subscribe(new double[] {});
            } else if (color == Optional.of(Alliance.Red)) {

                botPose = table.getDoubleArrayTopic("botpose_wpired").subscribe(new double[] {});
            }

            pose = botPose.get();

            filteredX = filter.calculate(pose[0]);
            filteredY = filter.calculate(pose[1]);

            return (new Pose2d(filteredX, filteredY, new Rotation2d(swerveDrive.getHeading())));
        }
    }

    public void changeAlliance() {
        Optional<Alliance> color = DriverStation.getAlliance();

        if (color == Optional.of(Alliance.Blue)) {
            NetworkTableInstance.getDefault().getTable("limelight").getEntry("botpose_wpiblue");
        } else if (color == Optional.of(Alliance.Red)) {
            NetworkTableInstance.getDefault().getTable("limelight").getEntry("botpose_wpired");
        }
    }

    public void setAprilTagPipeline() {
        table.getEntry("pipeline").setNumber(0);
    }
}