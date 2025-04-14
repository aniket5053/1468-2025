package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.ConstantsMechanisms.ElbowConstants;
import frc.robot.ConstantsMechanisms.ElevatorConstants;
import frc.robot.ConstantsMechanisms.WristConstants;
import frc.robot.subsystems.ElbowSubsystem;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.WristSubsystem;

public class ArmHomeFromAlgaeLow extends SequentialCommandGroup {
  // hold wrist and elevator still, move elbow back to not have algae get dislodged,

  public ArmHomeFromAlgaeLow(
      ElevatorSubsystem elevator, ElbowSubsystem elbow, WristSubsystem wrist, double tolerance) {
    addCommands(
        Commands.race(
            new MM_WristToPosition(wrist, WristConstants.kHomeWithAlgaeAngle, 0.0),
            // Commands.sequence(
            //                new WaitCommand(0.25),
            new MM_ElbowToPosition(elbow, ElbowConstants.kHomeWithAlgae, tolerance),
            // Commands.sequence(
            //                new WaitCommand(0.5), // Elev just has to go up a few inches - was
            // .25, too fast
            new MM_ElevatorToPosition( // just hold elev still now - save time
                elevator, ElevatorConstants.kHomeFromAlgaeLowPos, 0.0)));
  }
}
