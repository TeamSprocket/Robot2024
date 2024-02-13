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

    private NetworkTable tableShooter = NetworkTableInstance.getDefault().getTable("limelight-shooter");
    private NetworkTable tableIntake = NetworkTableInstance.getDefault().getTable("limelight-intake");

    private MedianFilter filterX = new MedianFilter(5);
    private MedianFilter filterY = new MedianFilter(5);
    private MedianFilter filterIntake = new MedianFilter(5);

    
    

    // private ArrayList<Double> llOdometryX = new ArrayList<Double>(50);
    // private ArrayList<Double> llOdometryY = new ArrayList<Double>(50);

    // private double totalX = 0.00;
    // private double totalY = 0.00;

    // private double averageX = 0.00;
    // private double averageY = 0.00;

    // private final double limeLightDeviation = 0.75;

    SwerveDrive swerveDrive;


    // TODO: test with only one filter because I think we only need one

    public Limelight() {}

    @Override
    public void periodic() {
        SmartDashboard.putNumber("Pose X", tableShooter.getEntry("tx").getDouble(0.0));
        SmartDashboard.putNumber("Pose Y", tableShooter.getEntry("ty").getDouble(0.0));
        SmartDashboard.putNumber("Filtered X", getTranslation2d().getX());
        SmartDashboard.putNumber("Filtered Y", getTranslation2d().getY());

        SmartDashboard.putNumber("Intake Note TX", getNoteTX());


        
        // Translation2d currentPose = getTranslation2d();

        // totalX += currentPose.getX();
        // totalX -= llOdometryX.get(0);
        // averageX = totalX / 50;

        // llOdometryX.add(currentPose.getX());
        // llOdometryX.remove(0);

        // totalY += currentPose.getY();
        // totalY -= llOdometryY.get(0);
        // averageY = totalY / 50;

        // llOdometryY.add(currentPose.getY());
        // llOdometryY.remove(0);
        
    }

    public Translation2d getTranslation2d() {
        double[] botPose;

        if (DriverStation.getAlliance().isPresent() && DriverStation.getAlliance().get() == Alliance.Blue) {
            botPose = tableShooter.getEntry("botpose_wpiblue").getDoubleArray(new double[2]);
        }
        else {
            botPose = tableShooter.getEntry("botpose_wpired").getDoubleArray(new double[2]);
        }

        double filteredX = filterX.calculate(botPose[0]);
        double filteredY = filterY.calculate(botPose[1]);

        return (new Translation2d(filteredX, filteredY));
    }

    public double getNoteTX() {
        return tableIntake.getEntry("tx").getDouble(0.0);
    }

    
    // public double getVolatilityAxis(double average, ArrayList llOdometry){
    //     double volatility = 0.00;
    //     for(int i = 0; i < 50; i++){
    //         volatility += Math.abs(average - (double)llOdometry.get(i));
    //     }
    //     return volatility;
    // }

    // /**
    //  * @return Highest axial volatility reading
    //  */
    // public double getOverallVolatility() {
    //     double volatilityX = getVolatilityAxis(averageX, llOdometryX);
    //     double volatilityY = getVolatilityAxis(averageY, llOdometryY);
    //     double overallVolatility = Util.max(volatilityX, volatilityY);
    //     return overallVolatility;
    // }


    // public boolean getIsNotVolatile() {
    //     return getOverallVolatility() < Constants.Limelight.kAcceptableVolatilityThreshold;
    // }





}
