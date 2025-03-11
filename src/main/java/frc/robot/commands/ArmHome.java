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

public class ArmHome extends SequentialCommandGroup {
  // First start wrist and elevator to insure it wont get wedged into reef,
  // then lower elevator, so it's not sticking out too far, and so not to get the wrist jammed
  public ArmHome(
      ElevatorSubsystem elevator, ElbowSubsystem elbow, WristSubsystem wrist, double tolerance) {

    addCommands(
        Commands.parallel(
            new MM_WristToPosition(wrist, WristConstants.kHomeAngle, tolerance),
            Commands.sequence(
                //              new WaitCommand(0.25),
                new MM_ElbowToPosition(elbow, ElbowConstants.kHomeAngle, tolerance)),
            Commands.sequence(
                new WaitCommand(
                    0.25), // TA TODO: Optimize delay - make sure not to hit reef - esp L4 and L3
                // Force elevator command to finish so that we can reset 0 position
                new MM_ElevatorToPosition(elevator, ElevatorConstants.kHomePos, .5),
                new InstantCommand(() -> elevator.setElevatorPosition(0.0), elevator))));
  }
}
