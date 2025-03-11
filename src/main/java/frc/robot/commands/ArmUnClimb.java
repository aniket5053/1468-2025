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
  // First raise elevator and start elbow. Delay wrist to insure it doesnt jam
  public ArmUnClimb(
      ElevatorSubsystem elevator, ElbowSubsystem elbow, WristSubsystem wrist, double tolerance) {
    addCommands(
        Commands.parallel(
            new MM_ElevatorToPosition(elevator, ElevatorConstants.kUnClimbPos, tolerance),
            Commands.sequence(
                //                  new WaitCommand(0.45),
                new MM_ElbowToPosition(elbow, ElbowConstants.kUnClimbAngle, tolerance)),
            Commands.sequence(
                new WaitCommand(0.25), // TA TODO: Optimize delay of wrist to not jam
                new MM_WristToPosition(wrist, WristConstants.kUnClimbAngle, tolerance))));
  }
}
