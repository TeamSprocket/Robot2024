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

    // private MedianFilter filterY = new MedianFilter(3);
    // private MedianFilter filterX = new MedianFilter(3);
    private MedianFilter filterIntake = new MedianFilter(5);

    private ArrayList<Double> distsX = new ArrayList<Double>(Constants.Limelight.kSlidingWindowLen);
    private ArrayList<Double> distsY = new ArrayList<Double>(Constants.Limelight.kSlidingWindowLen);


    SwerveDrive swerveDrive;


    public Limelight() {
        for (int i = 0; i < Constants.Limelight.kSlidingWindowLen; i++) {
            distsX.add(0.0);
            distsY.add(0.0);
        }
    }

    @Override
    public void periodic() {
        SmartDashboard.putNumber("Pose X [LL]", tableShooter.getEntry("botpose").getDoubleArray(new double[2])[0]);
        SmartDashboard.putNumber("Pose Y [LL]", tableShooter.getEntry("botpose").getDoubleArray(new double[2])[1]);
        SmartDashboard.putNumber("Filtered X [LL]", getTranslation2d().getX());
        SmartDashboard.putNumber("Filtered Y [LL]", getTranslation2d().getY());

        SmartDashboard.putNumber("Intake Note TX [LL]", getNoteTX());

        SmartDashboard.putNumber("Pose Volatility [LL]", getPoseVolatility());
        SmartDashboard.putBoolean("Pose Is Not Volatile [LL]", getIsNotVolatile());


        // System.out.println(""+tableShooter.getEntry("botpose").getDoubleArray(new double[2])[0]);
        Translation2d currentPose = getTranslation2d();

        distsX.add(currentPose.getX());
        distsX.remove(0);

        distsY.add(currentPose.getY());
        distsY.remove(0);
        
    }

    public Translation2d getTranslation2d() {
        double[] botPose;

        if (DriverStation.getAlliance().isPresent() && DriverStation.getAlliance().get() == Alliance.Blue) {
            botPose = tableShooter.getEntry("botpose_wpiblue").getDoubleArray(new double[2]);
        }
        else {
            botPose = tableShooter.getEntry("botpose_wpired").getDoubleArray(new double[2]);
        }

        double xPos = botPose[0];
        double yPos = botPose[1];


        // double filteredX = filterX.calculate(xPos);
        // double filteredY = filterY.calculate(yPos);

        // return (new Translation2d(filteredX, filteredY));
        return new Translation2d(xPos, yPos);
    }


    public double getNoteTX() {
        double noteTX = tableIntake.getEntry("tx").getDouble(0.0);
        noteTX = filterIntake.calculate(noteTX);
        return noteTX;
    }


    public boolean getIsNotVolatile() {
        return getPoseVolatility() < Constants.Limelight.kAcceptableVolatilityThreshold;
    }

    /**
     * @return Returns volatility of pose reading 
     */
    public double getPoseVolatility() {
        double volatilityX = getPoseVolatilityAxis(distsX);
        double volatilityY = getPoseVolatilityAxis(distsY);

        return Math.max(volatilityX, volatilityY);
    }

    private double getPoseVolatilityAxis(ArrayList<Double> distances) {
        double average = Util.average(distances);

        double totalDeviation = 0;
        for (double dist : distances) {
            totalDeviation += Math.abs(dist - average);
        }

        return totalDeviation;
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
