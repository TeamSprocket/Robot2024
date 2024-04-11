// // Copyright (c) FIRST and other WPILib contributors.
// // Open Source Software; you can modify and/or share it under the terms of
// // the WPILib BSD license file in the root directory of this project.

// package frc.robot.commands.persistent;

// import java.util.function.Supplier;

// import edu.wpi.first.wpilibj2.command.Command;
// import frc.robot.subsystems.Elevator;

// public class ElevatorManual extends Command {
//   /** Creates a new ElevatorManual. */
//   Elevator elevator;
//   Supplier<Double> motorOutputSupplier;

//   public ElevatorManual(Elevator elevator, Supplier<Double> motorOutputSupplier) {
//     this.elevator = elevator;
//     this.motorOutputSupplier = motorOutputSupplier;

//     addRequirements(elevator);
//     // Use addRequirements() here to declare subsystem dependencies.
//   }

//   // Called when the command is initially scheduled.
//   @Override
//   public void initialize() {}

//   // Called every time the scheduler runs while the command is scheduled.
//   @Override
//   public void execute() {
//     elevator.manual(motorOutputSupplier.get() * 0.2);
//     System.out.println(motorOutputSupplier.get() * 0.2);
//   }

//   // Called once the command ends or is interrupted.
//   @Override
//   public void end(boolean interrupted) {}

//   // Returns true when the command should end.
//   @Override
//   public boolean isFinished() {
//     return false;
//   }
// }
