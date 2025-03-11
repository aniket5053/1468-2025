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

public class ArmPreClimb1 extends SequentialCommandGroup {
  // "UNclimb" command must be selected first, which will have elevator and wrist in correct
  // position, but for safety reasons delay wrist
  // so now just move elbow
  public ArmPreClimb1(
      ElevatorSubsystem elevator, ElbowSubsystem elbow, WristSubsystem wrist, double tolerance) {
    addCommands(
        Commands.parallel(
            new MM_ElevatorToPosition(elevator, ElevatorConstants.kClimbPos, tolerance),
            Commands.sequence(
                new WaitCommand(0.25),
                new MM_WristToPosition(wrist, WristConstants.kClimbAngle, tolerance)),
            Commands.sequence(
                //                new WaitCommand(0.45),
                new MM_ElbowToPosition(elbow, ElbowConstants.kPreClimbAngle1, tolerance))));
  }
}
