package frc.robot.commands.old;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.VisionSubsystem;
import frc.robot.subsystems.drive.Drive;

public class DriveToNoteCommand extends Command {
  private final double targetYaw = -5.5; // Desired yaw value to align to
  private final double yawTolerance = 5.0; // degrees
  private final double strafeTolerance = 2.0;
  private final double xDriveSpeed = 0.25; // percentage
  private final double yDriveSpeed = 0.20;
  private final double rotationSpeed = 0.25; // percentage
  private boolean isAligned = false; // Tracks whether alignment is achieved

  private final Drive m_drive;
  private final VisionSubsystem m_vision;

  public DriveToNoteCommand(Drive drive, VisionSubsystem vision) {
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
    if (m_vision.leftFrtCamTgtDectected()) {
      // Phase 1: Rotate until aligned
      if (!isAligned) {
        if ((Math.abs(m_vision.getleftFrtCamBestTgtYaw()) - Math.abs(targetYaw)) > yawTolerance) {
          if (m_vision.getleftFrtCamBestTgtYaw() < targetYaw) {
            // Rotate CCW
            SmartDashboard.putString("DriveToNote Status", "CCW");
            m_drive.driveWithSpeeds(0, 0, rotationSpeed, false);
          } else {
            // Rotate CW
            SmartDashboard.putString("DriveToNote Status", "CW");
            m_drive.driveWithSpeeds(0, 0, -rotationSpeed, false);
          }
        } else {
          isAligned = true; // Alignment achieved
          m_drive.stop();
        }
      } else {
        // Phase 2: Drive forward and strafe if necessary
        if (m_vision.getleftFrtCamBestTgtYaw() < targetYaw
            && (targetYaw - m_vision.getleftFrtCamBestTgtYaw()) > strafeTolerance) {
          // Slide left
          SmartDashboard.putString("DriveToNote Status", "Slide left");
          m_drive.driveWithSpeeds(xDriveSpeed, yDriveSpeed, 0, false);
        } else if (m_vision.getleftFrtCamBestTgtYaw() > targetYaw
            && (targetYaw + m_vision.getleftFrtCamBestTgtYaw()) > strafeTolerance) {
          // Slide right
          SmartDashboard.putString("DriveToNote Status", "Slide right");
          m_drive.driveWithSpeeds(xDriveSpeed, -yDriveSpeed, 0, false);
        } else {
          // Just forward
          SmartDashboard.putString("DriveToNote Status", "Just forward");
          m_drive.driveWithSpeeds(xDriveSpeed, 0, 0, false);
        }
      }
    } else {
      SmartDashboard.putString("DriveToNote Status", "NOTE NOT SEEN");
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    SmartDashboard.putString("DriveToNote Status", "NOT ACTIVE");
    isAligned = false;
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
