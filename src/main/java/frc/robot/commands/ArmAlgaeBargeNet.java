package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.ConstantsMechanisms.ElbowConstants;
import frc.robot.ConstantsMechanisms.ElevatorConstants;
import frc.robot.ConstantsMechanisms.WristConstants;
import frc.robot.subsystems.ElbowSubsystem;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.HandlerSubsystem;
import frc.robot.subsystems.WristSubsystem;

public class ArmAlgaeBargeNet extends SequentialCommandGroup {
  // Since Barge/Net is very high, start elbow to get arm close to vertical,
  // and then extend elevator, wrist last to insure it's free to move
  public ArmAlgaeBargeNet(
      ElevatorSubsystem elevator,
      ElbowSubsystem elbow,
      WristSubsystem wrist,
      HandlerSubsystem s_Handler) {
    addCommands(
        Commands.parallel(
            new HandlerHarvestAlgae(s_Handler),
            new MM_ElbowToPosition(
                elbow, ElbowConstants.kBargeNetAngle, ElbowConstants.kToleranceDegrees),
            Commands.sequence(
                new WaitCommand(0.05),
                new MM_ElevatorToPosition(
                    elevator, ElevatorConstants.kBargeNetPos, ElevatorConstants.kToleranceInches)),
            Commands.sequence(
                new WaitCommand(0.40),
                new MM_WristToPosition(
                    wrist, WristConstants.kBargeNetAngle, WristConstants.kToleranceDegrees))));
  }
}
