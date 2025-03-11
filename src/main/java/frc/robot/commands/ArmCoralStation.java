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
  // Since Level 1 elevator is at 0 and wrist movement is close to -90, start everything together
  public ArmCoralStation(
      ElevatorSubsystem elevator, ElbowSubsystem elbow, WristSubsystem wrist, double tolerance) {
    addCommands(
        Commands.parallel(
            new MM_WristToPosition(wrist, WristConstants.kCoralStationAngle, tolerance),
            new MM_ElevatorToPosition(elevator, ElevatorConstants.kCoralStationPos, tolerance),
            new MM_ElbowToPosition(elbow, ElbowConstants.kCoralStationAngle, tolerance)));
  }
}
