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

public class ArmUnClimb extends SequentialCommandGroup {
  // First start wrist to insure it wont get wedged into elevator,
  // then lower elevator, so it's not sticking out too far, lastly, lower elbow angle
  public ArmUnClimb(ElevatorSubsystem elevator, ElbowSubsystem elbow, WristSubsystem wrist) {
    addCommands(
        Commands.parallel(
            new MM_ElevatorToPosition(
                elevator, ElevatorConstants.kUnClimbPos, ElevatorConstants.kToleranceInches),
            Commands.sequence(
                new WaitCommand(0.2),
                new MM_WristToPosition(
                    wrist, WristConstants.kUnClimbAngle, WristConstants.kToleranceDegrees),
                Commands.sequence(
                    new WaitCommand(0.45),
                    new MM_ElbowToPosition(
                        elbow, ElbowConstants.kUnClimbAngle, ElbowConstants.kToleranceDegrees)))));
  }
}
