package frc.robot.commands;

import static frc.robot.ConstantsMechanisms.ElbowConstants.*;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ElbowSubsystem;

public class MM_ElbowToPosition extends Command {
  private final ElbowSubsystem elbowSubsystem;
  private final double targetPositionDegrees;
  private final double toleranceDegrees;
  private int counter = 0; // delay finishing a few counts

  public MM_ElbowToPosition(
      ElbowSubsystem elbowSubsystem, double targetPositionDegrees, double toleranceDegrees) {
    this.elbowSubsystem = elbowSubsystem;
    this.targetPositionDegrees = targetPositionDegrees;
    this.toleranceDegrees = toleranceDegrees;
    addRequirements(elbowSubsystem);
  }

  @Override
  public void initialize() {
    // Reset encoder position if needed, or ensure it's properly initialized elsewhere
    // elbowSubsystem.setElbowPosition(0);
    // Set the target position for Motion Magic
    elbowSubsystem.setElbowMagicMoPos(targetPositionDegrees / kEncoderRotation2Degrees);
    counter = 0;
  }

  @Override
  public void execute() {
    // Set the target position for Motion Magic
    // elbowSubsystem.setElbowMagicMoPos(targetPositionRotations);
  }

  @Override
  public void end(boolean interrupted) {
    // Stop the motor when the command ends
    elbowSubsystem.stop();
  }

  @Override
  public boolean isFinished() {
    // Check if the motor is close enough to the target position
    // note if tolerance = 0.0 the command will never finish
    // return Math.abs(elbowSubsystem.getLtElbowPosition() - targetPositionRotations) < tolerance;
    if (Math.abs(
            elbowSubsystem.getLtElbowPosition() - targetPositionDegrees / kEncoderRotation2Degrees)
        < toleranceDegrees / kEncoderRotation2Degrees) {
      counter++;
    }
    return ((counter >= 5));
  }
}
