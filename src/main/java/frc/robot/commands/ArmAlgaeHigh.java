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

public class ArmAlgaeHigh extends SequentialCommandGroup {
  // First extend elevator to free wrist movement, elbow last to move handler into algae
  public ArmAlgaeHigh(
      ElevatorSubsystem elevator, ElbowSubsystem elbow, WristSubsystem wrist, double tolerance) {
    addCommands(
        Commands.parallel(
            new MM_ElevatorToPosition(elevator, ElevatorConstants.kAlgaeHighPos, tolerance),
            Commands.sequence(
                new WaitCommand(0.1),
                new MM_WristToPosition(wrist, WristConstants.kAlgaeHighAngle, tolerance)),
            Commands.sequence(
                new WaitCommand(0.35),
                new MM_ElbowToPosition(elbow, ElbowConstants.kAlgaeHighAngle, tolerance))));
  }
}
