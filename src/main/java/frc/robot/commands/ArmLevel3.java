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

public class ArmLevel3 extends SequentialCommandGroup {
  // Since Level 3 is not low, start elbow to get arm close to vertical,
  // and then extend elevator, wrist last to insure it's free to move
  public ArmLevel3(ElevatorSubsystem elevator, ElbowSubsystem elbow, WristSubsystem wrist) {
    addCommands(
        Commands.parallel(
            new MM_ElbowToPosition(
                elbow, ElbowConstants.kLevel3Angle, ElbowConstants.kToleranceDegrees),
            Commands.sequence(
                new WaitCommand(0.2),
                new MM_ElevatorToPosition(
                    elevator, ElevatorConstants.kLevel3Pos, ElevatorConstants.kToleranceInches)),
            Commands.sequence(
                new WaitCommand(0.3),
                new MM_WristToPosition(
                    wrist, WristConstants.kLevel3Angle, WristConstants.kToleranceDegrees))));
  }
}
