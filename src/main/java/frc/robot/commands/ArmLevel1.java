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

public class ArmLevel1 extends SequentialCommandGroup {
  // Since Level 1 just keeps elev at 0, move wrist delay elbow to avoid hitting reef.
  public ArmLevel1(
      ElevatorSubsystem elevator, ElbowSubsystem elbow, WristSubsystem wrist, double tolerance) {
    addCommands(
        Commands.parallel(
            new MM_ElevatorToPosition(elevator, ElevatorConstants.kLevel1Pos, tolerance),
            new MM_WristToPosition(wrist, WristConstants.kLevel1Angle, tolerance),
            Commands.sequence(
                new WaitCommand(0.25), // TA TODO: Optimize delay
                new MM_ElbowToPosition(elbow, ElbowConstants.kLevel1Angle, tolerance))));
  }
}
