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
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
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
    //      Rear Camera Code - removed, now have 2 front cameras
    // Activate a path only if its a valid Reef April Tag ID detected
    //    if (m_vision.rearCamTgtDectected()
    //        && ( // red reef april tags are numbers 6 - 11, blue are 17 - 22
    //        (DriverStation.getAlliance().isPresent()
    //                && (DriverStation.getAlliance().get() == Alliance.Red)
    //                && (m_vision.getRearCamBestTgtId() >= 1)
    //                && (m_vision.getRearCamBestTgtId() <= 2))
    //            || (DriverStation.getAlliance().isPresent()
    //                && (DriverStation.getAlliance().get() == Alliance.Blue)
    //                && (m_vision.getRearCamBestTgtId() >= 12)
    //                && (m_vision.getRearCamBestTgtId() <= 13)))) {
    //      Rear Camera Code - removed, now have 2 front cameras

    // The rotation component in these poses represents the direction of travel
    double TempX = m_drive.getPose().getX();
    double TempY = m_drive.getPose().getY();
    Rotation2d TempAngle = Rotation2d.fromDegrees(180.0 + endPtHoloRotation);
    //      Pose2d startPt = m_drive.getPose();
    Pose2d startPt = new Pose2d(TempX, TempY, TempAngle);
    double TgtID = 0.0;
    if (TempX < 8.0) { // Blue Side
      if (TempY < 4.0) {
        TgtID = 12.0;
      } else {
        TgtID = 13.0;
      }
    } else { // Red Side
      if (TempY < 4.0) {
        TgtID = 1.0;
      } else {
        TgtID = 2.0;
      }
    }

    //      double TgtID = m_vision.getRearCamBestTgtId();

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
                Units.degreesToRadians(360),
                Units.degreesToRadians(540)), // was 540
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
    SmartDashboard.putString("DriveToTag Status", "NOT ACTIVE");
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
        if (offset == kLeftSide) {
          endPtX = 16.852;
          endPtY = 1.360;
          endPtHoloRotation = 126.0;
        } else if (offset == kRightSide) {
          endPtX = 15.941;
          endPtY = 0.700;
          endPtHoloRotation = 126.0;
        } else {
          endPtX = 16.413;
          endPtY = 1.039;
          endPtHoloRotation = 126.0;
        }
        break;

      case 2:
        if (offset == kLeftSide) {
          endPtX = 15.941;
          endPtY = 7.330;
          endPtHoloRotation = -126.0;
        } else if (offset == kRightSide) {
          endPtX = 16.852;
          endPtY = 6.666;
          endPtHoloRotation = -126.0;
        } else {
          endPtX = 16.380;
          endPtY = 7.010;
          endPtHoloRotation = -126.0;
        }
        break;
        // blue Coral Station
      case 12:
        if (offset == kLeftSide) {
          endPtX = 1.590;
          endPtY = 0.710;
          endPtHoloRotation = 54.0;
        } else if (offset == kRightSide) {
          endPtX = 0.700;
          endPtY = 1.358;
          endPtHoloRotation = 54.0;
        } else {
          endPtX = 1.150;
          endPtY = 1.030;
          endPtHoloRotation = 54.0;
        }
        break;

      case 13:
        if (offset == kLeftSide) {
          endPtX = 0.690;
          endPtY = 6.660;
          endPtHoloRotation = -54.0;
        } else if (offset == kRightSide) {
          endPtX = 1.645;
          endPtY = 7.356;
          endPtHoloRotation = -54.0;
        } else {
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
