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

public class ArmHomeAfterAlgaeShotAuto extends SequentialCommandGroup {
  // this command is only executed after shooting Algae Out in Autonomous,
  // The next Algae is High (so is the 3rd) so get arm ready to do a high algae
  public ArmHomeAfterAlgaeShotAuto(
      ElevatorSubsystem elevator, ElbowSubsystem elbow, WristSubsystem wrist, double tolerance) {
    addCommands(
        Commands.parallel(
            new MM_WristToPosition(wrist, WristConstants.kAlgaeHighAngle, tolerance),
            new MM_ElevatorToPosition(elevator, ElevatorConstants.kAlgaeHighPos, .5),
            Commands.sequence(
                new WaitCommand(0.25),
                new MM_ElbowToPosition(elbow, ElbowConstants.kHomeWithAlgae, tolerance))));
  }
}
