package frc.robot.subsystems;

import java.util.Optional;
import edu.wpi.first.math.filter.MedianFilter;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Limelight extends SubsystemBase {

    private NetworkTable table = NetworkTableInstance.getDefault().getTable("limelight");
    private MedianFilter filter = new MedianFilter(10);
    // TODO: test with only one filter because I think we only need one

    public Limelight() {}

    @Override
    public void periodic() {
        SmartDashboard.putNumber("Pose X", table.getEntry("tx").getDouble(0.0));
        SmartDashboard.putNumber("Pose Y", table.getEntry("ty").getDouble(0.0));
        SmartDashboard.putNumber("Filtered X", getTranslation2d().getX());
        SmartDashboard.putNumber("Filtered Y", getTranslation2d().getY());
    }

    public Translation2d getTranslation2d() {
        Optional<Alliance> color = DriverStation.getAlliance();
        double[] botPose;

        if (color == Optional.of(Alliance.Blue)) {

            botPose = table.getEntry("botpose_wpiblue").getDoubleArray(new double[2]);
        }
        else {

            botPose = table.getEntry("botpose_wpired").getDoubleArray(new double[2]);
        }

        double filteredX = filter.calculate(botPose[0]);
        double filteredY = filter.calculate(botPose[1]);

        return (new Translation2d(filteredX, filteredY));
    }
}