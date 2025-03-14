package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.ConstantsMechanisms.ElbowConstants;
import frc.robot.ConstantsMechanisms.ElevatorConstants;
import frc.robot.ConstantsMechanisms.WristConstants;
import frc.robot.subsystems.ElbowSubsystem;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.WristSubsystem;

public class ArmHomeWithAlgae extends SequentialCommandGroup {
  // First start hold wrist still and move elbow back to not have algae get dislodged,
  // then lower elevator
  public ArmHomeWithAlgae(
      ElevatorSubsystem elevator, ElbowSubsystem elbow, WristSubsystem wrist, double tolerance) {
    addCommands(
        Commands.parallel(
            new MM_WristToPosition(wrist, WristConstants.kHomeWithAlgaeAngle, tolerance),
            Commands.sequence(
                //                new WaitCommand(0.25),
                new MM_ElbowToPosition(elbow, ElbowConstants.kHomeAngle, tolerance)),
            Commands.sequence(
                new WaitCommand(0.4),
                // Force elevator command to finish so that we can reset 0 position
                new MM_ElevatorToPosition(elevator, ElevatorConstants.kHomePos, .5),
                new InstantCommand(() -> elevator.setElevatorPosition(0.0), elevator))));
  }
}
