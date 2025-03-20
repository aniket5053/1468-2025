package frc.robot.commands;

import static frc.robot.ConstantsMechanisms.ElbowConstants.*;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ElbowSubsystem;

public class MM_ElbowToPosition extends Command {
  private final ElbowSubsystem m_elbowSubsystem;
  private double m_targetPositionDegrees;
  private final double m_toleranceDegrees;
  private int counter = 0; // delay finishing a few counts
  private double currentPosition;

  public MM_ElbowToPosition(
      ElbowSubsystem elbowSubsystem, double targetPositionDegrees, double toleranceDegrees) {
    m_elbowSubsystem = elbowSubsystem;
    m_targetPositionDegrees = targetPositionDegrees;
    m_toleranceDegrees = toleranceDegrees;
    addRequirements(m_elbowSubsystem);
  }

  @Override
  public void initialize() {
    // Reset encoder position if needed, or ensure it's properly initialized elsewhere
    // elbowSubsystem.setElbowPosition(0);

    // note that the left side is negative angles, right side positive
    currentPosition = -m_elbowSubsystem.getLtElbowPosition() * kEncoderRotation2Degrees;
    //   SmartDashboard.putNumber("Current LFT Elbow Pos in INIT", currentPosition);

    if (m_targetPositionDegrees == kUpSmallDegrees)
      m_targetPositionDegrees = currentPosition + kSmallMoveDegrees;
    if (m_targetPositionDegrees == kDownSmallDegrees)
      m_targetPositionDegrees = currentPosition - kSmallMoveDegrees;
    if (m_targetPositionDegrees < kLowerSoftLimit) m_targetPositionDegrees = kLowerSoftLimit;
    if (m_targetPositionDegrees > kUpperSoftLimit) m_targetPositionDegrees = kUpperSoftLimit;

    // Set the target position for Motion Magic
    m_elbowSubsystem.setElbowMagicMoPos(m_targetPositionDegrees / kEncoderRotation2Degrees);
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
    m_elbowSubsystem.stop();
  }

  @Override
  public boolean isFinished() {
    // Check if the motor is close enough to the target position
    // note if tolerance = 0.0 the command will never finish
    // return Math.abs(elbowSubsystem.getLtElbowPosition() - targetPositionRotations) < tolerance;
    // SmartDashboard.putNumber(
    //     "Current LFT Elbow Pos in FIN", -m_elbowSubsystem.getLtElbowPosition());

    if (Math.abs(
            m_elbowSubsystem.getLtElbowPosition()
                + m_targetPositionDegrees / kEncoderRotation2Degrees)
        < m_toleranceDegrees / kEncoderRotation2Degrees) {
      counter++;
    }
    return ((counter >= 5));
  }
}
