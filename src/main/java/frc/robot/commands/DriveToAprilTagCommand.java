package frc.robot.commands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.VisionSubsystem;
import frc.robot.subsystems.drive.Drive;

public class DriveToAprilTagCommand extends Command {
  private final double targetYaw = -5.5; // Desired yaw value to align to
  private final double yawTolerance = 5.0; // degrees
  private final double strafeTolerance = 2.0;
  private final double xDriveSpeed = 0.25; // percentage
  private final double yDriveSpeed = 0.20;
  private final double rotationSpeed = 0.25; // percentage
  private boolean isAligned = false; // Tracks whether alignment is achieved

  private final Drive m_drive;
  private final VisionSubsystem m_vision;

  public DriveToAprilTagCommand(Drive drive, VisionSubsystem vision) {
    m_drive = drive;
    addRequirements(m_drive);
    m_vision = vision;
    addRequirements(m_vision);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  @Override
  public void execute() {
    if (m_vision.specificAprilTagDetected(7)) {

      double Tx = m_vision.getTargetX();
      double Ty = m_vision.getTargetY();
      double Tyaw = m_drive.getRotation().getDegrees();
      SmartDashboard.putNumber("DriveToTag Tx", Tx);
      SmartDashboard.putNumber("DriveToTag Ty", Ty);
      SmartDashboard.putNumber("DriveToTag Tyaw", Tyaw);

      Tx = (Tx - 2) * 2.0;
      if (Tx > 0.5) Tx = 0.5;
      if (Tx < -0.5) Tx = -0.5;
      Ty = (Ty - .00) * 2.0;
      if (Ty > 0.5) Ty = 0.5;
      if (Ty < -0.5) Ty = -0.5;
      Tyaw = (Tyaw - 4.5) * (-.05);
      if (Tyaw > 0.25) Tyaw = 0.25;
      if (Tyaw < -0.25) Tyaw = -0.25;

      m_drive.driveWithSpeeds(Tx, Ty, Tyaw, false);
    } else {
      SmartDashboard.putString("DriveToTag Status", "NOTE NOT SEEN");
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    SmartDashboard.putString("DriveToTag Status", "NOT ACTIVE");
    isAligned = false;
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
