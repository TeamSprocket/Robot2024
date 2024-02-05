package frc.robot.subsystems;

import java.util.ArrayList;
import java.util.List;
import java.util.Optional;

import frc.util.Util;

import edu.wpi.first.math.filter.MedianFilter;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.SwerveDrive;
import frc.robot.Constants;

public class Limelight extends SubsystemBase {

    private NetworkTable table = NetworkTableInstance.getDefault().getTable("limelight");
    private MedianFilter filter = new MedianFilter(10);

    private ArrayList<Double> llOdometryX = new ArrayList<Double>(50);
    private ArrayList<Double> llOdometryY = new ArrayList<Double>(50);

    private double totalX = 0.00;
    private double totalY = 0.00;

    private double averageX = 0.00;
    private double averageY = 0.00;

    private final double limeLightDeviation = 0.75;

    SwerveDrive swerveDrive;


    // TODO: test with only one filter because I think we only need one

    public Limelight() {}

    @Override
    public void periodic() {
        SmartDashboard.putNumber("Pose X", table.getEntry("tx").getDouble(0.0));
        SmartDashboard.putNumber("Pose Y", table.getEntry("ty").getDouble(0.0));
        SmartDashboard.putNumber("Filtered X", getTranslation2d().getX());
        SmartDashboard.putNumber("Filtered Y", getTranslation2d().getY());

        Translation2d currentPose = getTranslation2d();

        totalX += currentPose.getX();
        totalX -= llOdometryX.get(0);
        averageX = totalX / 50;

        llOdometryX.add(currentPose.getX());
        llOdometryX.remove(0);

        totalY += currentPose.getY();
        totalY -= llOdometryY.get(0);
        averageY = totalY / 50;

        llOdometryY.add(currentPose.getY());
        llOdometryY.remove(0);
        
    }

    public Translation2d getTranslation2d() {
        Alliance color = DriverStation.getAlliance();
        double[] botPose;

        if (color == new Optional<Alliance.Blue>) {

            botPose = table.getEntry("botpose_wpiblue").getDoubleArray(new double[2]);
        }
        else {

            botPose = table.getEntry("botpose_wpired").getDoubleArray(new double[2]);
        }

        double filteredX = filter.calculate(botPose[0]);
        double filteredY = filter.calculate(botPose[1]);

        return (new Translation2d(filteredX, filteredY));
    }

    
    public double getVolatilityAxis(double average, ArrayList llOdometry){
        double volatility = 0.00;
        for(int i = 0; i < 50; i++){
            volatility += Math.abs(average - (double)llOdometry.get(i));
        }
        return volatility;
    }

    /**
     * @return Highest axial volatility reading
     */
    public double getOverallVolatility() {
        double volatilityX = getVolatilityAxis(averageX, llOdometryX);
        double volatilityY = getVolatilityAxis(averageY, llOdometryY);
        double overallVolatility = Util.max(volatilityX, volatilityY);
        return overallVolatility;
    }


    public boolean getIsNotVolatile() {
        return getOverallVolatility() < Constants.Limelight.kAcceptableVolatilityThreshold;
    }





}
