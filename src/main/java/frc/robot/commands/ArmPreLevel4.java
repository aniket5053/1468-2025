package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.ConstantsMechanisms.ElbowConstants;
import frc.robot.ConstantsMechanisms.ElevatorConstants;
import frc.robot.ConstantsMechanisms.WristConstants;
import frc.robot.subsystems.ElbowSubsystem;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.WristSubsystem;

public class ArmPreLevel4 extends SequentialCommandGroup {
  // Since Level 4 is high, move elbow to get arm closer to vertical and arm up above L3,
  // but still back so some driving can still occur, leave wrist down to not hit anything
  public ArmPreLevel4(
      ElevatorSubsystem elevator, ElbowSubsystem elbow, WristSubsystem wrist, double tolerance) {
    addCommands(
        Commands.parallel(
            new MM_ElbowToPosition(elbow, ElbowConstants.kPreLevel4Angle, tolerance),
            Commands.sequence(
                //                new WaitCommand(0.20),
                new MM_ElevatorToPosition(elevator, ElevatorConstants.kPreLevel4Pos, tolerance)),
            Commands.sequence(
                //                new WaitCommand(0.30),
                new MM_WristToPosition(wrist, WristConstants.kHomeAngle, tolerance))));
  }
}
