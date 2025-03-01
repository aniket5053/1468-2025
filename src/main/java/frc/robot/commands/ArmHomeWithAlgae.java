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

public class ArmHomeWithAlgae extends SequentialCommandGroup {
  // First start wrist to insure it wont get wedged into elevator,
  // then lower elevator, so it's not sticking out too far, lastly, lower elbow angle
  public ArmHomeWithAlgae(ElevatorSubsystem elevator, ElbowSubsystem elbow, WristSubsystem wrist) {
    addCommands(
        Commands.parallel(
            new MM_WristToPosition(
                wrist, WristConstants.kProcessorAngle, WristConstants.kToleranceDegrees),
            Commands.sequence(
                new WaitCommand(0.1),
                new MM_ElevatorToPosition(
                    elevator, ElevatorConstants.kHomePos, ElevatorConstants.kToleranceInches)),
            Commands.sequence(
                new WaitCommand(0.25),
                new MM_ElbowToPosition(
                    elbow, ElbowConstants.kHomeAngle, ElbowConstants.kToleranceDegrees))));
  }
}
