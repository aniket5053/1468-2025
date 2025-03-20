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

public class ArmPreAlgaeBargeNet extends SequentialCommandGroup {
  // This command is only activated after ArmHomeWithAlgae,
  // so the arm is fairly vertical, elevator slightly up, and wrist at 0 prior to call.
  // so no wait delays needed - keeping small wrist delay for safety (lab button pushing)

  public ArmPreAlgaeBargeNet(
      ElevatorSubsystem elevator, ElbowSubsystem elbow, WristSubsystem wrist, double tolerance) {
    addCommands(
        Commands.parallel(
            new MM_ElbowToPosition(elbow, ElbowConstants.kBargeNetAngle, tolerance),
            Commands.sequence(
                //                new WaitCommand(0.05),
                new MM_ElevatorToPosition(elevator, ElevatorConstants.kPreBargeNetPos, tolerance)),
            Commands.sequence(
                new WaitCommand(0.15),
                new MM_WristToPosition(wrist, WristConstants.kBargeNetAngle, tolerance))));
  }
}
