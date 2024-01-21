package frc.robot.subsystems;

import java.util.Optional;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.networktables.DoubleArraySubscriber;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Limelight extends SubsystemBase {

    private NetworkTable table = NetworkTableInstance.getDefault().getTable("limelight");

    SwerveDrive swerveDrive;
    PIDController pidController;
    double pose[];

    private SwerveDriveOdometry odometry;

    public Limelight(SwerveDrive swerveDrive) {

        this.swerveDrive = swerveDrive;
        this.pidController = new PIDController(Constants.Drivetrain.kLimelightAlignP, Constants.Drivetrain.kLimelightAlignI, Constants.Drivetrain.kLimelightAlignD);
        this.pidController.setSetpoint(0);
        odometry = new SwerveDriveOdometry(Constants.Drivetrain.kDriveKinematics, new Rotation2d(swerveDrive.getHeading()), swerveDrive.getModulePositions());
    }

    @Override
    public void periodic() {}

    public Pose2d getPose() {
        DoubleArraySubscriber botPose = table.getDoubleArrayTopic("botpose").subscribe(new double[] {});
        int hasTarget = (int)table.getEntry("tv").getInteger(0);
        Optional<Alliance> color = DriverStation.getAlliance();
        
        if (hasTarget == 0) {

            Pose2d robotPose = odometry.update(new Rotation2d(swerveDrive.getHeading()), swerveDrive.getModulePositions());
            double x = round(robotPose.getX(), 3);
            double y = round(robotPose.getY(), 3); // Probably unecessary, just add a deadband when aligning
            double radians = round(robotPose.getRotation().getRadians(), 3);

            return (new Pose2d(x, y, new Rotation2d(radians)));
        } 
        else {

            if (color == Optional.of(Alliance.Blue)) {

                botPose = table.getDoubleArrayTopic("botpose_wpiblue").subscribe(new double[] {});
            } else if (color == Optional.of(Alliance.Red)) {

                botPose = table.getDoubleArrayTopic("botpose_wpired").subscribe(new double[] {});
            }

            pose = botPose.get();
            return (new Pose2d(pose[0], pose[1], new Rotation2d(swerveDrive.getHeading())));
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

    public static double round(double num, int places)
    {
        double scale = Math.pow(10, places);
        double roundedNum = Math.round(num * scale) / scale;
        return roundedNum;
    }

    public void setAprilTagPipeline() {
        table.getEntry("pipeline").setNumber(0);
    }

    public double getXoffset() {
        return table.getEntry("tx").getDouble(0.0);
    }
}