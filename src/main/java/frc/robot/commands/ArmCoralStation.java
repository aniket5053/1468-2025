package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.ConstantsMechanisms.ElbowConstants;
import frc.robot.ConstantsMechanisms.ElevatorConstants;
import frc.robot.ConstantsMechanisms.WristConstants;
import frc.robot.subsystems.ElbowSubsystem;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.WristSubsystem;

public class ArmCoralStation extends SequentialCommandGroup {
  // Since Level 1 is low, start elbow and elevator movement, wrist last to insure it's free to move
  public ArmCoralStation(ElevatorSubsystem elevator, ElbowSubsystem elbow, WristSubsystem wrist) {
    addCommands(
        Commands.parallel(
            new MM_WristToPosition(
                wrist, WristConstants.kCoralStationAngle, WristConstants.kToleranceDegrees),
            new MM_ElevatorToPosition(
                elevator, ElevatorConstants.kCoralStationPos, ElevatorConstants.kToleranceInches)),
        new MM_ElbowToPosition(
            elbow, ElbowConstants.kCoralStationAngle, ElbowConstants.kToleranceDegrees));
  }
}
