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

public class ArmPreLevel4 extends SequentialCommandGroup {
  // Since Level 4 is high, start elbow to get arm close to vertical,
  // and then extend elevator, wrist last to insure it's free to move
  public ArmPreLevel4(ElevatorSubsystem elevator, ElbowSubsystem elbow, WristSubsystem wrist) {
    addCommands(
        Commands.parallel(
            new MM_ElbowToPosition(
                elbow, ElbowConstants.kPreLevel4Angle, ElbowConstants.kToleranceDegrees),
            Commands.sequence(
                new WaitCommand(0.20),
                new MM_ElevatorToPosition(
                    elevator, ElevatorConstants.kPreLevel4Pos, ElevatorConstants.kToleranceInches)),
            Commands.sequence(
                new WaitCommand(0.30),
                new MM_WristToPosition(
                    wrist, WristConstants.kLevel4Angle, WristConstants.kToleranceDegrees))));
  }
}
