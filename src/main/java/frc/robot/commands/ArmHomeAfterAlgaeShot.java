package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.ConstantsMechanisms.ElbowConstants;
import frc.robot.ConstantsMechanisms.ElevatorConstants;
import frc.robot.ConstantsMechanisms.WristConstants;
import frc.robot.subsystems.ElbowSubsystem;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.WristSubsystem;

public class ArmHomeAfterAlgaeShot extends SequentialCommandGroup {
  // this command is only executed after shooting Algae Out, but is optimized for a Barge/Net shot
  // since the elevator is so high
  // note that for a processor shot, the elevator is up a little to allow for wrist motion.
  public ArmHomeAfterAlgaeShot(
      ElevatorSubsystem elevator, ElbowSubsystem elbow, WristSubsystem wrist, double tolerance) {
    addCommands(
        Commands.parallel(
            new MM_WristToPosition(wrist, WristConstants.kHomeAngle, tolerance),
            // Force elevator command to finish so that we can reset 0 position
            Commands.sequence(
                new MM_ElevatorToPosition(elevator, ElevatorConstants.kHomePos, .5),
                new InstantCommand(() -> elevator.setElevatorPosition(0.0), elevator)),
            Commands.sequence(
                new WaitCommand(0.25),
                new MM_ElbowToPosition(elbow, ElbowConstants.kHomeAngle, tolerance))));
  }
}
