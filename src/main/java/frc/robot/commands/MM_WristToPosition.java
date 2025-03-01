package frc.robot.commands;

import static frc.robot.ConstantsMechanisms.WristConstants.*;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.WristSubsystem;

public class MM_WristToPosition extends Command {
  private final WristSubsystem WristSubsystem;
  private final double targetPositionDegrees;
  private final double toleranceDegrees;
  private int counter = 0;

  public MM_WristToPosition(
      WristSubsystem WristSubsystem, double targetPositionDegrees, double toleranceDegrees) {
    this.WristSubsystem = WristSubsystem;
    this.targetPositionDegrees = targetPositionDegrees;
    this.toleranceDegrees = toleranceDegrees;
    addRequirements(WristSubsystem);
  }

  @Override
  public void initialize() {
    // Reset encoder position if needed, or ensure it's properly initialized elsewhere
    // WristSubsystem.setWristPosition(0);
    // Set the target position for Motion Magic
    WristSubsystem.setWristMagicMoPos(targetPositionDegrees / kEncoderRotation2Degrees);
    counter = 0;
  }

  @Override
  public void execute() {
    // Set the target position for Motion Magic
    // WristSubsystem.setWristMagicMoPos(targetPositionRotations);
  }

  @Override
  public void end(boolean interrupted) {
    // Stop the motor when the command ends
    WristSubsystem.stop();
  }

  @Override
  public boolean isFinished() {
    // Check if the motor is close enough to the target position
    // note if tolerance = 0.0 the command will never finish
    // return Math.abs(WristSubsystem.getWristPosition() - targetPositionRotations) < tolerance;
    if (Math.abs(
            WristSubsystem.getWristPosition() - targetPositionDegrees / kEncoderRotation2Degrees)
        < toleranceDegrees / kEncoderRotation2Degrees) {
      counter++;
    }
    return ((counter >= 5));
  }
}
