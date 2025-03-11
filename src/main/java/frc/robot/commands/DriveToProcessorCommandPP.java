package frc.robot.commands;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.GoalEndState;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.path.Waypoint;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.drive.Drive;
import java.util.List;

public class DriveToProcessorCommandPP extends Command {

  private double endPtX, endPtY, endPtHoloRotation;

  private final Drive m_drive;

  public DriveToProcessorCommandPP(Drive drive) {
    m_drive = drive;
    addRequirements(m_drive);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  @Override
  public void execute() {

    // The rotation component in these poses represents the direction of travel
    Pose2d startPt = m_drive.getPose();
    Pose2d endPt;
    if (DriverStation.getAlliance().isPresent()
        && (DriverStation.getAlliance().get() == Alliance.Red)) {
      //  Real Coordinates
      //     endPtX = 11.561;
      //     endPtY = 7.485;
      //     endPtHoloRotation = 90.0;
      endPtX = 15.0;
      endPtY = 6;
      endPtHoloRotation = -90.0;
      endPt = new Pose2d(endPtX, endPtY, Rotation2d.fromDegrees(endPtHoloRotation));
    } else if (DriverStation.getAlliance().isPresent()
        && (DriverStation.getAlliance().get() == Alliance.Blue)) {
      endPtX = 5.988;
      endPtY = 0.572;
      endPtHoloRotation = -90.0;
      endPt = new Pose2d(endPtX, endPtY, Rotation2d.fromDegrees(endPtHoloRotation));
    } else {
      endPt = startPt;
    }

    List<Waypoint> wayPoints = PathPlannerPath.waypointsFromPoses(startPt, endPt);
    PathPlannerPath path =
        new PathPlannerPath(
            wayPoints,
            new PathConstraints(
                4.0,
                4.0,
                // 3.0, 3.0,      4.0s seem to work better
                Units.degreesToRadians(540),
                Units.degreesToRadians(720)),
            null,
            new GoalEndState(0.0, Rotation2d.fromDegrees(endPtHoloRotation)));

    // Prevent this path from being flipped on the red alliance, since the given
    // positions are already correct
    path.preventFlipping = true;

    AutoBuilder.followPath(path).schedule();
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    SmartDashboard.putString("DriveToProcessor status", "NOT ACTIVE");
    m_drive.stop();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
