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

public class ArmLevel4 extends SequentialCommandGroup {
  // start elevator to get above reef, then move elbow and wrist
  public ArmLevel4(
      ElevatorSubsystem elevator, ElbowSubsystem elbow, WristSubsystem wrist, double tolerance) {
    addCommands(
        Commands.parallel(
            new MM_ElevatorToPosition(elevator, ElevatorConstants.kLevel4Pos, tolerance),
            Commands.sequence(
                new WaitCommand(0.10), // TA TODO: Optimize delay - make sure not to hit reef
                new MM_ElbowToPosition(elbow, ElbowConstants.kLevel4Angle, tolerance)),
            Commands.sequence(
                new WaitCommand(0.75), // TA TODO: Optimize delay - make sure not to hit reef
                new MM_WristToPosition(wrist, WristConstants.kLevel4Angle, tolerance))));
  }
}
