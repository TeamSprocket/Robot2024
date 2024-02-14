package frc.robot.subsystems;

import java.util.ArrayList;
import org.ejml.simple.SimpleMatrix;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.Nat;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.estimator.KalmanFilter;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.system.LinearSystem;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import edu.wpi.first.wpilibj.Timer;

public class Limelight extends SubsystemBase {

    private NetworkTable shooterLLTable = NetworkTableInstance.getDefault().getTable("limelight-shooter");
    private NetworkTable intakeLLTable = NetworkTableInstance.getDefault().getTable("limelight-intake");

    private static ArrayList<Double> poseX = new ArrayList<>();
    private static ArrayList<Double> odomPoseX = new ArrayList<>();
    private static ArrayList<Double> poseY = new ArrayList<>();
    private static ArrayList<Double> odomPoseY = new ArrayList<>();

    private static Timer timer = new Timer();

    private static double dtSeconds = 0.02;

    private final static LinearSystem<N1, N1, N1> m_flywheelPlant =
        LinearSystemId.identifyVelocitySystem(Constants.Drivetrain.kMaxSpeed, Constants.Drivetrain.kMaxAccel); // creates a linear system for us
        /*
            return new LinearSystem<>(
                VecBuilder.fill(-kSwerveV / kSwerveAccel), // VecBuilder.fill(double n1) -> returns fillVec(Nat.N1(), n1) **Nat.N1() has to = length of n1 -> returns Vector<>(new SimpleMatrix(n1)) ***n1 has to be of type double[][]
                VecBuilder.fill(1.0 / kSwerveAccel), 
                VecBuilder.fill(1.0),
                VecBuilder.fill(0.0));
         */

    private final static KalmanFilter<N1, N1, N1> m_observerX =
      new KalmanFilter<>(
          Nat.N1(), // Nat representing states of the system
          Nat.N1(), // Nat representing outputs of the system
          m_flywheelPlant, // linear system used for prediction step
          VecBuilder.fill(3.0), // How accurate we think our model is - standard deviations of model states
          VecBuilder.fill(0.01), // How accurate we think our encoder - standard deviations of measurements
          dtSeconds); // seconds

    private final static KalmanFilter<N1, N1, N1> m_observerY =
      new KalmanFilter<>(
          Nat.N1(), // Nat representing states of the system
          Nat.N1(), // Nat representing outputs of the system
          m_flywheelPlant, // linear system used for prediction step
          VecBuilder.fill(3.0), // How accurate we think our model is - standard deviations of model states
          VecBuilder.fill(0.01), // How accurate we think our encoder - standard deviations of measurements
          dtSeconds); // seconds

    public Limelight() {
        m_observerX.reset();
        m_observerY.reset();
        timer.restart();
    }

    @Override
    public void periodic() {

        SmartDashboard.putNumber("LL Filtered posX", getUnfilteredTranslation2d().getX());
        SmartDashboard.putNumber("LL Filtered posY", getUnfilteredTranslation2d().getY());

        SmartDashboard.putNumber("LL Intake tX", intakeLLTable.getEntry("tx").getDouble(0.0));
    }

    public Translation2d getUnfilteredTranslation2d() {
        double[] botPose;

        if (DriverStation.getAlliance().isPresent() && DriverStation.getAlliance().get() == Alliance.Blue) {
            botPose = shooterLLTable.getEntry("botpose_wpiblue").getDoubleArray(new double[2]);
        }
        else {
            botPose = shooterLLTable.getEntry("botpose_wpired").getDoubleArray(new double[2]);
        }
        
        poseX.add(botPose[0]);

        return (new Translation2d(botPose[0], botPose[1]));
    }

    public static Matrix<N1, N1> getMatrix(ArrayList<Double> arraylist) {
        Double[] array = arraylist.toArray(new Double[arraylist.size()]);
        double[] primitiveArray = toPrimitive(array);

        SimpleMatrix simpleMatrix = new SimpleMatrix(primitiveArray);
        Matrix<N1, N1> matrix = new Matrix<>(simpleMatrix);

        return matrix;
    }
    
    public static double[] toPrimitive(Double[] array) {
        double[] primitiveArray = new double[array.length];

        for (int i = 0; i < array.length; i++) {
            primitiveArray[i] = array[i].doubleValue();
        }
        return primitiveArray;
    }

    public static double getFilteredX(double x) {

        odomPoseX.add(x);
    
        updateDTseconds();

        m_observerX.predict(getMatrix(poseX), dtSeconds);
        m_observerX.correct(getMatrix(odomPoseX), getMatrix(poseX));

        return m_observerX.getXhat(0);
    }

    public static double getFilteredY(double y) {

        odomPoseY.add(y);
        
        updateDTseconds();

        m_observerY.predict(getMatrix(poseY), dtSeconds);
        m_observerY.correct(getMatrix(odomPoseY), getMatrix(poseY));

        return m_observerY.getXhat(0);
    }

    public static void updateDTseconds() {
        dtSeconds = timer.get();
    }
}