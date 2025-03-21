package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.ConstantsMechanisms.ElbowConstants;
import frc.robot.ConstantsMechanisms.ElevatorConstants;
import frc.robot.ConstantsMechanisms.WristConstants;
import frc.robot.subsystems.ElbowSubsystem;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.WristSubsystem;

public class ArmHomeFromAlgaeLow extends SequentialCommandGroup {
  // First start hold wrist still and move elbow back to not have algae get dislodged,
  // then lower elevator
  public ArmHomeFromAlgaeLow(
      ElevatorSubsystem elevator, ElbowSubsystem elbow, WristSubsystem wrist, double tolerance) {
    addCommands(
        Commands.parallel(
            new MM_WristToPosition(wrist, WristConstants.kHomeWithAlgaeAngle, tolerance),
            Commands.sequence(
                //                new WaitCommand(0.25),
                new MM_ElbowToPosition(elbow, ElbowConstants.kHomeWithAlgae, tolerance)),
            Commands.sequence(
                new WaitCommand(0.15), // Elev just has to go up a few inches
                new MM_ElevatorToPosition(
                    elevator, ElevatorConstants.kHomeFromAlgaeHighPos, tolerance))));
  }
}
