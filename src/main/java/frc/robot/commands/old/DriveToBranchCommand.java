package frc.robot.commands.old;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.VisionSubsystem;
import frc.robot.subsystems.drive.Drive;

public class DriveToBranchCommand extends Command {
  private double offset = 0.0; // Desired yaw value to align to

  private boolean isAligned = false; // Tracks whether alignment is achieved

  private final Drive m_drive;
  private final VisionSubsystem m_vision;

  public DriveToBranchCommand(Drive drive, VisionSubsystem vision, double offset) {
    m_drive = drive;
    addRequirements(m_drive);
    m_vision = vision;
    addRequirements(m_vision);
    this.offset = offset;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  @Override
  public void execute() {
    if (m_vision.frntCamTgtDectected()) {

      double Tx = m_vision.getFrntCamBestTgtX();
      double Ty = m_vision.getFrntCamBestTgtY();
      double Tyaw = m_vision.getFrntCamBestTgtYaw();
      //      double robotYaw = m_drive.getRotation().getDegrees();

      Tx = (Tx - 0.3) * 2.0; // drive to ~12 inches of april tag - X direction
      if (Tx > 0.5) Tx = 0.5;
      if (Tx < -0.5) Tx = -0.5;

      Ty = (Ty - this.offset / 39.3701) * 2.0; // drive to offset position - Y direction
      if (Ty > 0.375) Ty = 0.375; // .5 too much
      if (Ty < -0.375) Ty = -0.375;

      Tyaw =
          -(Tyaw - 4.5)
              * (+.1); // 0.05 too small  //zero out yaw - want to be parallel to april tag
      if (Tyaw > 0.375) Tyaw = 0.375; // 0.25 too small
      if (Tyaw < -0.375) Tyaw = -0.375;

      m_drive.driveWithSpeeds(Tx, Ty, Tyaw, false);

    } else {
      m_drive.driveWithSpeeds(0, 0, 0, false);
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
