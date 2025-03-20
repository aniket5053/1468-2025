package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.ConstantsMechanisms.ElbowConstants;
import frc.robot.ConstantsMechanisms.ElevatorConstants;
import frc.robot.ConstantsMechanisms.WristConstants;
import frc.robot.subsystems.ElbowSubsystem;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.WristSubsystem;

public class ArmLevel4Auto extends SequentialCommandGroup {
  // in Autonomous Only, we have a PreLevel4 position that holds the elevator in a middle height and
  // the elbow closer to verticle
  // start all, elbow is slowest, so hold elevator and wrist, end command when elbow is done.
  public ArmLevel4Auto(
      ElevatorSubsystem elevator, ElbowSubsystem elbow, WristSubsystem wrist, double tolerance) {
    addCommands(
        Commands.race(
            new MM_ElbowToPosition(elbow, ElbowConstants.kLevel4AngleAuto, tolerance),
            new MM_ElevatorToPosition(elevator, ElevatorConstants.kLevel4Pos, 0.0),
            new MM_WristToPosition(wrist, WristConstants.kLevel4Angle, 0.0)));
  }
}
