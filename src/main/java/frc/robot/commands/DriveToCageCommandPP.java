package frc.robot.commands;

import static frc.robot.ConstantsMechanisms.DriveConstants.*;

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
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.ConstantsMechanisms.DriveConstants;
import frc.robot.subsystems.drive.Drive;
import java.util.List;

public class DriveToCageCommandPP extends Command {

  private double endPtX, endPtY, endPtHoloRotation;
  private double offset;

  private final Drive m_drive;

  public DriveToCageCommandPP(Drive drive, double offset) {
    m_drive = drive;
    addRequirements(m_drive);
    this.offset = offset;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  @Override
  public void execute() {
    //    String selectedCage = cageChooser.getSelected(); // Get the selected cage

    // The rotation component in these poses represents the direction of travel
    Pose2d startPt = m_drive.getPose();
    double startX = m_drive.getPose().getX();
    double startY = m_drive.getPose().getY();
    double currentRotation = m_drive.getPose().getRotation().getDegrees();
    if (Math.abs(currentRotation) > 90.0) endPtHoloRotation = 180.0;
    else endPtHoloRotation = 0.0;
    Pose2d endPt;

    // For safety - Only allow button push if close to Barge in both x and y
    if (DriverStation.getAlliance().isPresent()
        && (DriverStation.getAlliance().get() == Alliance.Red)
        && ((startX < 13.0) && (startY < 4.5))) {
      //  Real Coordinates
      endPtX = 9.9;

      if (endPtHoloRotation == 180.0) // going to climb
      {
        endPtX = 9.35;

        if (m_drive.getCageLocation() == DriveConstants.kLeftSide) {
          endPtY = 0.815;
        } else if (m_drive.getCageLocation() == DriveConstants.kRightSide) {
          endPtY = 3.015;
        } else {
          endPtY = 1.915;
        }
      } else {
        endPtX = 9.9;
        if (startY > 3.4) endPtY = 3.4;
        else endPtY = startY;
      }

      endPt = new Pose2d(endPtX, endPtY, Rotation2d.fromDegrees(endPtHoloRotation));

    } else if (DriverStation.getAlliance().isPresent()
        && (DriverStation.getAlliance().get() == Alliance.Blue)
        && ((startX > 4.5) && (startY > 4.0))) {
      endPtX = 7.65;

      if (endPtHoloRotation == 0.0) // going to climb
      {
        endPtX = 8.2;

        if (m_drive.getCageLocation() == DriveConstants.kLeftSide) {
          endPtY = 7.238;
        } else if (m_drive.getCageLocation() == DriveConstants.kRightSide) {
          endPtY = 5.038;
        } else {
          endPtY = 6.138;
        }

      } else { // going to put algae on the barge - go straight forward, unless too far right, then
        // shift over
        endPtX = 7.65;
        if (startY < 4.6) endPtY = 4.6;
        else endPtY = startY;
      }

      endPt = new Pose2d(endPtX, endPtY, Rotation2d.fromDegrees(endPtHoloRotation));

      // Only get here if a no Alliance retrieved, or too far from barge
    } else {
      endPt = startPt;
    }

    // create intermediate point for path that is 10 inches from the end point
    double interPtX = (endPtX - (0.254 * Math.cos(endPtHoloRotation * (Math.PI / 180))));
    double interPtY = (endPtY - (0.254 * Math.sin(endPtHoloRotation * (Math.PI / 180))));
    Pose2d interPt = new Pose2d(interPtX, interPtY, Rotation2d.fromDegrees(endPtHoloRotation));

    List<Waypoint> wayPoints = PathPlannerPath.waypointsFromPoses(startPt, interPt, endPt);

    //    List<Waypoint> wayPoints = PathPlannerPath.waypointsFromPoses(startPt, endPt);
    PathPlannerPath path =
        new PathPlannerPath(
            wayPoints,
            new PathConstraints(
                3.0,
                3.0,
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
    //   SmartDashboard.putString("DriveToTag Cage", "NOT ACTIVE");
    m_drive.stop();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
