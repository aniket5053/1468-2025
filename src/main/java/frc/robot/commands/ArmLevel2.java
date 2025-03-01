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

public class ArmLevel2 extends SequentialCommandGroup {
  // Since Level 2 is low, start elbow and elevator movement, wrist last to insure it's free to move
  public ArmLevel2(ElevatorSubsystem elevator, ElbowSubsystem elbow, WristSubsystem wrist) {
    addCommands(
        Commands.parallel(
            new MM_ElbowToPosition(
                elbow, ElbowConstants.kLevel2Angle, ElbowConstants.kToleranceDegrees),
            Commands.sequence(
                new WaitCommand(0.05),
                new MM_ElevatorToPosition(
                    elevator, ElevatorConstants.kLevel2Pos, ElevatorConstants.kToleranceInches)),
            Commands.sequence(
                new WaitCommand(0.15),
                new MM_WristToPosition(
                    wrist, WristConstants.kLevel2Angle, WristConstants.kToleranceDegrees))));
  }
}
