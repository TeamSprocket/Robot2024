// package frc.robot.commands.superstructure;

// import java.util.function.Supplier;

// import edu.wpi.first.wpilibj.Timer;
// import edu.wpi.first.wpilibj.XboxController;
// import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
// import edu.wpi.first.wpilibj2.command.Command;
// import frc.robot.Constants;
// import frc.robot.subsystems.Superstructure;
// import frc.robot.subsystems.Superstructure.SSStates;

// public class Handoff extends Command{
//     private final Superstructure superstructure;

//     public Handoff(Superstructure superstructure) {
//         this.superstructure = superstructure;

//         addRequirements(superstructure);
//     }

//     public void initialize() {
//         superstructure.setState(SSStates.WAIT_HANDOFF);
//     }

//     @Override
//     public void execute() {}

//     public void end() {
//         superstructure.setState(SSStates.STOWED);
//     }

//     public boolean isFinished() {
//         return superstructure.getState() == SSStates.STOWED;
//     }
// }