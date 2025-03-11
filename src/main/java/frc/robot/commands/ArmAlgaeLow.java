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

public class ArmAlgaeLow extends SequentialCommandGroup {
  // Since elevator is at 0, move wrist to position and then elbow into algae
  public ArmAlgaeLow(
      ElevatorSubsystem elevator, ElbowSubsystem elbow, WristSubsystem wrist, double tolerance) {
    addCommands(
        Commands.parallel(
            new MM_ElevatorToPosition(elevator, ElevatorConstants.kAlgaeLowPos, tolerance),
            Commands.sequence(
                //           new WaitCommand(0.2),
                new MM_WristToPosition(wrist, WristConstants.kAlgaeLowAngle, tolerance)),
            Commands.sequence(
                new WaitCommand(0.25),
                new MM_ElbowToPosition(elbow, ElbowConstants.kAlgaeLowAngle, tolerance))));
  }
}
