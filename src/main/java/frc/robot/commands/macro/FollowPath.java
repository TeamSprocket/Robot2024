
// package frc.robot.commands.macro;

// import java.util.ArrayList;
// import java.util.List;

// import com.pathplanner.lib.auto.AutoBuilder;
// import com.pathplanner.lib.path.PathConstraints;
// // import com.pathplanner.lib.Pathplanner;
// import com.pathplanner.lib.path.PathPlannerPath;
// import com.pathplanner.lib.path.PathPlannerTrajectory;
// import com.pathplanner.lib.path.PathPoint;
// import com.pathplanner.lib.path.PathPlannerTrajectory.State;

// import edu.wpi.first.math.geometry.Rotation2d;
// import edu.wpi.first.math.kinematics.ChassisSpeeds;
// import edu.wpi.first.wpilibj.Timer;
// import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
// import edu.wpi.first.wpilibj2.command.CommandBase;

// public class FollowPath extends CommandBase {
//   Timer timer = new Timer();
//   int currentPoint = 0;
//   List<State> states;


//   public FollowPath() {
//     // Load the path you want to follow using its name in the GUI
//     PathPlannerPath path = PathPlanner.loadPath("Left Right 2m Test");
//     states = path.getTrajectory(new ChassisSpeeds(), new Rotation2d(Math.PI)).getStates();
//   }

//   // Called when the command is initially scheduled.
//   @Override
//   public void initialize() {
//     timer.reset();
//     timer.start();
//   }

//   // Called every time the scheduler runs while the command is scheduled.
//   @Override
//   public void execute() {
//     timer.start();
    
//     State point = states.get(currentPoint);
//     double nextTime = point.timeSeconds;
//     if (timer.get() > nextTime) {
//       double xTarget = point.positionMeters.getX();
//       double yTarget = point.positionMeters.getY();
//       double tTarget = point.targetHolonomicRotation.getRadians();

//       SmartDashboard.putNumber("Y Target Pos", yTarget);



//       currentPoint++;
//     }


//   }

//   // Called once the command ends or is interrupted.
//   @Override
//   public void end(boolean interrupted) {}

//   // Returns true when the command should end.
//   @Override
//   public boolean isFinished() {
//     return (currentPoint - 1) >= states.size();
//   }
// }
