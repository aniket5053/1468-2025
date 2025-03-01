package frc.robot.commands;

import static frc.robot.ConstantsMechanisms.ElevatorConstants.*;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ElevatorSubsystem;

public class MM_ElevatorToPosition extends Command {
  private final ElevatorSubsystem ElevatorSubsystem;
  private final double targetPositionInches;
  private final double toleranceInInches;
  private int counter = 0;

  public MM_ElevatorToPosition(
      ElevatorSubsystem ElevatorSubsystem, double targetPositionInches, double toleranceInInches) {
    this.ElevatorSubsystem = ElevatorSubsystem;
    this.targetPositionInches = targetPositionInches;
    this.toleranceInInches = toleranceInInches;
    addRequirements(ElevatorSubsystem);
  }

  @Override
  public void initialize() {
    // Reset encoder position if needed, or ensure it's properly initialized elsewhere
    // ElevatorSubsystem.setElevatorPosition(0);
    // Set the target position for Motion Magic
    ElevatorSubsystem.setElevatorMagicMoPos(targetPositionInches / kEncoderRotation2Inches);
    counter = 0;
  }

  @Override
  public void execute() {
    // Set the target position for Motion Magic
    // ElevatorSubsystem.setElevatorMagicMoPos(targetPositionRotations);
  }

  @Override
  public void end(boolean interrupted) {
    // Stop the motor when the command ends
    ElevatorSubsystem.stop();
  }

  @Override
  public boolean isFinished() {
    // Check if the motor is close enough to the target position
    // note if tolerance = 0.0 the command will never finish
    // return Math.abs(ElevatorSubsystem.getElevatorPosition() - targetPositionRotations) <
    // tolerance;
    if (Math.abs(
            ElevatorSubsystem.getElevatorPosition()
                - targetPositionInches / kEncoderRotation2Inches)
        < toleranceInInches / kEncoderRotation2Inches) {
      counter++;
    }
    return ((counter >= 5));
  }
}
