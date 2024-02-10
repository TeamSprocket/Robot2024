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

    private NetworkTable shooterLLTable = NetworkTableInstance.getDefault().getTable("limelight-shooter");
    private NetworkTable intakeLLTable = NetworkTableInstance.getDefault().getTable("limelight-intake");
    private MedianFilter filterX = new MedianFilter(10);
    private MedianFilter filterY = new MedianFilter(10);
    private MedianFilter filterIntake = new MedianFilter(5);

    private ArrayList<Double> xPoseReadings = new ArrayList<Double>(50);
    private ArrayList<Double> yPoseReadings = new ArrayList<Double>(50);

    private double totalX = 0.00;
    private double totalY = 0.00;

    private double averageX = 0.00;
    private double averageY = 0.00;
    

    public Limelight() {}

    @Override
    public void periodic() {
        SmartDashboard.putNumber("LL Filtered posX", getTranslation2d().getX());
        SmartDashboard.putNumber("LL Filtered posY", getTranslation2d().getY());
        SmartDashboard.putBoolean("LL IsNotVolatile", getIsNotVolatile());

        SmartDashboard.putNumber("LL Intake tX", intakeLLTable.getEntry("tx").getDouble(0.0));
        



        Translation2d currentPose = getTranslation2d();



        totalX += currentPose.getX();
        totalX -= xPoseReadings.get(0);
        averageX = totalX / 50;

        xPoseReadings.add(currentPose.getX());
        xPoseReadings.remove(0);

        totalY += currentPose.getY();
        totalY -= yPoseReadings.get(0);
        averageY = totalY / 50;

        yPoseReadings.add(currentPose.getY());
        yPoseReadings.remove(0);
        
    }

    public Translation2d getTranslation2d() {
        double[] botPose;

        if (DriverStation.getAlliance().isPresent() && DriverStation.getAlliance().get() == Alliance.Blue) {
            botPose = shooterLLTable.getEntry("botpose_wpiblue").getDoubleArray(new double[2]);
        }
        else {
            botPose = shooterLLTable.getEntry("botpose_wpired").getDoubleArray(new double[2]);
        }

        double filteredX = filterX.calculate(botPose[0]);
        double filteredY = filterY.calculate(botPose[1]);

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
        double volatilityX = getVolatilityAxis(averageX, xPoseReadings);
        double volatilityY = getVolatilityAxis(averageY, yPoseReadings);
        double overallVolatility = Util.max(volatilityX, volatilityY);
        return overallVolatility;
    }


    public boolean getIsNotVolatile() {
        return getOverallVolatility() < Constants.Limelight.kAcceptableVolatilityThreshold;
    }

    /**
     * @return tX reading from note detection, 0.0 if undetected 
     */
    public double getIntakeTX() {
        return intakeLLTable.getEntry("tx").getDouble(0.0);
    }





}
