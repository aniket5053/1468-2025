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
import frc.robot.subsystems.VisionSubsystem;
import frc.robot.subsystems.drive.Drive;
import java.util.List;

public class DriveToCoralStCommandPP extends Command {
  private double offset; // Desired yaw value to align to
  // Note that these 3 vars are determined in the "DetermineEndPoint" routine
  private double endPtX, endPtY, endPtHoloRotation;

  private final Drive m_drive;
  private final VisionSubsystem m_vision;

  public DriveToCoralStCommandPP(Drive drive, VisionSubsystem vision, double offset) {
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

    // The rotation component in these poses represents the direction of travel
    Pose2d startPt = m_drive.getPose();
    double startX = m_drive.getPose().getX();
    double startY = m_drive.getPose().getY();
    double TgtID = 0.0;

    if (DriverStation.getAlliance().isPresent()
        && (DriverStation.getAlliance().get() == Alliance.Blue)
        && (startX < 6.0)) {

      if (startY < 4.0) {
        TgtID = 12.0;
      } else {
        TgtID = 13.0;
      }

      // For safety - Only allow button push if close to Barge in both x and y
    } else if (DriverStation.getAlliance().isPresent()
        && (DriverStation.getAlliance().get() == Alliance.Red)
        && (startX > 13.0)) {

      if (startY < 4.0) {
        TgtID = 1.0;
      } else {
        TgtID = 2.0;
      }
    } else TgtID = 99; // dont auto drive, bad startPt (99 will cause 0 length path)

    DetermineEndPoint(TgtID, offset);
    // since driving backwards, the direction of travel is 180 + rotation
    Pose2d endPt =
        new Pose2d(endPtX, endPtY, Rotation2d.fromDegrees(180.0 + endPtHoloRotation)); // x,-y

    List<Waypoint> wayPoints = PathPlannerPath.waypointsFromPoses(startPt, endPt);
    PathPlannerPath path =
        new PathPlannerPath(
            wayPoints,
            new PathConstraints(
                3.0,
                3.0, // was 4
                // 3.0, 3.0,
                Units.degreesToRadians(540),
                Units.degreesToRadians(720)), // was 540
            null,
            //  new
            // IdealStartingState(m_drive.getFFCharacterizationVelocity()*TunerConstants.kDriveGearRatio, m_drive.getRotation()),
            new GoalEndState(0.0, Rotation2d.fromDegrees(endPtHoloRotation)));

    // Prevent this path from being flipped on the red alliance, since the given
    // positions are already correct
    path.preventFlipping = true;

    AutoBuilder.followPath(path).schedule();

    //      Rear Camera Code - removed, now have 2 front cameras
    //    } else {
    //      m_drive.stop();
    //      SmartDashboard.putString("DriveToTag Status", "NOTE NOT SEEN");
    //    }
    //      Rear Camera Code - removed, now have 2 front cameras
  }
  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    //  SmartDashboard.putString("DriveToTag Status", "NOT ACTIVE");
    m_drive.stop();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }

  // Note that these 3 vars are determined in the "DetermineEndPoint" routine
  // private double endPtX, endPtY, endPtHoloRotation;
  public void DetermineEndPoint(double Id, double offset) {
    switch ((int) Id) {
        // red Coral Stations
      case 1:
        if (offset == kRightSide) {
          endPtX = 16.75;
          endPtY = 1.33;
          endPtHoloRotation = 126.0;
        } else if (offset == kLeftSide) {
          endPtX = 15.95;
          endPtY = 0.76;
          endPtHoloRotation = 126.0;
        } else { // not used
          endPtX = 16.413;
          endPtY = 1.039;
          endPtHoloRotation = 126.0;
        }
        break;

      case 2:
        if (offset == kRightSide) {
          endPtX = 15.99;
          endPtY = 7.29;
          endPtHoloRotation = -126.0;
        } else if (offset == kLeftSide) {
          endPtX = 16.8;
          endPtY = 6.7;
          endPtHoloRotation = -126.0;
        } else { // not used
          endPtX = 16.380;
          endPtY = 7.010;
          endPtHoloRotation = -126.0;
        }
        break;
        // blue Coral Station
      case 12:
        if (offset == kRightSide) {
          endPtX = 1.560;
          endPtY = 0.760;
          endPtHoloRotation = 54.0;
        } else if (offset == kLeftSide) {
          endPtX = 0.75;
          endPtY = 1.350;
          endPtHoloRotation = 54.0;
        } else { // not used
          endPtX = 1.150;
          endPtY = 1.030;
          endPtHoloRotation = 54.0;
        }
        break;

      case 13:
        if (offset == kRightSide) {
          endPtX = 0.800;
          endPtY = 6.720;
          endPtHoloRotation = -54.0;
        } else if (offset == kLeftSide) {
          endPtX = 1.600;
          endPtY = 7.290;
          endPtHoloRotation = -54.0;
        } else { // not used
          endPtX = 1.183;
          endPtY = 7.020;
          endPtHoloRotation = -54.0;
        }
        break;

      default:
        endPtX = m_drive.getPose().getX();
        endPtY = m_drive.getPose().getY();
        endPtHoloRotation = m_drive.getPose().getRotation().getDegrees();
        break;
    }
  }
}
