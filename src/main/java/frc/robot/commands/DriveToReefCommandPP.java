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

public class DriveToReefCommandPP extends Command {
  private double offset; // Desired yaw value to align to
  //  private double robotX, robotY;
  //  private double robotYaw, robotYawDegs, turnRads;
  //  private boolean isAligned = false; // Tracks whether alignment is achieved
  // Note that these 3 vars are determined in the "DetermineEndPoint" routine
  private double endPtX, endPtY, endPtHoloRotation;

  private final Drive m_drive;
  private final VisionSubsystem m_vision;

  public DriveToReefCommandPP(Drive drive, VisionSubsystem vision, double offset) {
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

    // Activate a path only if its a valid Reef April Tag ID detected
    double TgtID = m_vision.getTrgtIdToDriveTo();
    if (TgtID != 99.0) {

      // The rotation component in these poses represents the direction of travel
      double TempX = m_drive.getPose().getX();
      double TempY = m_drive.getPose().getY();
      Rotation2d TempAngle = Rotation2d.fromDegrees(endPtHoloRotation);
      //      Pose2d startPt = m_drive.getPose();
      Pose2d startPt = new Pose2d(TempX, TempY, TempAngle);

      DetermineEndPoint(TgtID, offset);
      Pose2d endPt = new Pose2d(endPtX, endPtY, Rotation2d.fromDegrees(endPtHoloRotation)); // x,-y

      List<Waypoint> wayPoints = PathPlannerPath.waypointsFromPoses(startPt, endPt);
      PathPlannerPath path =
          new PathPlannerPath(
              wayPoints,
              new PathConstraints(
                  3.0,
                  3.0, // was 4
                  // 3.0, 3.0,
                  Units.degreesToRadians(360),
                  Units.degreesToRadians(720)), // was 540
              null,
              new GoalEndState(0.0, Rotation2d.fromDegrees(endPtHoloRotation)));

      // Prevent this path from being flipped on the red alliance, since the given
      // positions are already correct
      path.preventFlipping = true;

      AutoBuilder.followPath(path).schedule();
      SmartDashboard.putString("DriveToReefCmd", "Driving to AprilTag!");
      //////// Have to get initial yaw value from robot and calculate a delta yaw offf
      //////// the apriltag yaw

    } else {
      m_drive.stop();
      SmartDashboard.putString("DriveToReefCmd", "No April Tag - Stop!");
    }
  }
  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    SmartDashboard.putString("DriveToReefCmd", "NOT ACTIVE");
    //    isAligned = false;
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

        ////////////////////////////// red reef /////////////////////////////////////////
      case 6:
        if (offset == kLeftSide) {
          endPtX = 13.576;
          endPtY = 2.805;
          endPtHoloRotation = 120.0;
        } else if (offset == kRightSide) {
          endPtX = 13.864;
          endPtY = 2.971;
          endPtHoloRotation = 120.0;
        } else {
          endPtX = 13.742;
          endPtY = 2.865;
          endPtHoloRotation = 120.0;
        }
        break;

      case 7:
        if (offset == kLeftSide) {
          endPtX = 14.380;
          endPtY = 3.862;
          endPtHoloRotation = 180.0;
        } else if (offset == kRightSide) {
          endPtX = 14.380;
          endPtY = 4.191;
          endPtHoloRotation = 180.0;
        } else {
          endPtX = 14.400;
          endPtY = 4.040;
          endPtHoloRotation = 180.0;
        }
        break;

      case 8:
        if (offset == kLeftSide) {
          endPtX = 13.860;
          endPtY = 5.080;
          endPtHoloRotation = -120.0;
        } else if (offset == kRightSide) {
          endPtX = 13.577;
          endPtY = 5.245;
          endPtHoloRotation = -120.0;
        } else {
          endPtX = 13.746;
          endPtY = 5.191;
          endPtHoloRotation = -120.0;
        }
        break;

      case 9:
        if (offset == kLeftSide) {
          endPtX = 12.548;
          endPtY = 5.245;
          endPtHoloRotation = -60.0;
        } else if (offset == kRightSide) {
          endPtX = 12.263;
          endPtY = 5.080;
          endPtHoloRotation = -60.0;
        } else {
          endPtX = 12.379;
          endPtY = 5.198;
          endPtHoloRotation = -60.0;
        }
        break;

      case 10:
        if (offset == kLeftSide) {
          endPtX = 11.748;
          endPtY = 4.191;
          endPtHoloRotation = 0.0;
        } else if (offset == kRightSide) {
          endPtX = 11.748;
          endPtY = 3.860;
          endPtHoloRotation = 0.0;
        } else {
          endPtX = 11.700;
          endPtY = 4.040;
          endPtHoloRotation = 0.0;
        }
        break;

      case 11:
        if (offset == kLeftSide) {
          endPtX = 12.264;
          endPtY = 2.971;
          endPtHoloRotation = 60.0;
        } else if (offset == kRightSide) {
          endPtX = 12.550;
          endPtY = 2.805;
          endPtHoloRotation = 60.0;
        } else {
          endPtX = 12.386;
          endPtY = 2.856;
          endPtHoloRotation = 60.0;
        }
        break;

        ////////////////////////////// blue reef /////////////////////////////////////////
      case 17:
        if (offset == kLeftSide) {
          endPtX = 3.688;
          endPtY = 2.970;
          endPtHoloRotation = 60.0;
        } else if (offset == kRightSide) {
          endPtX = 3.974;
          endPtY = 2.806;
          endPtHoloRotation = 60.0;
        } else {
          endPtX = 3.810;
          endPtY = 2.855;
          endPtHoloRotation = 60.0;
        }
        break;

      case 18:
        if (offset == kLeftSide) {
          endPtX = 3.172;
          endPtY = 4.191;
          endPtHoloRotation = 0.0;
        } else if (offset == kRightSide) {
          endPtX = 3.172;
          endPtY = 3.860;
          endPtHoloRotation = 0.0;
        } else {
          endPtX = 3.150;
          endPtY = 4.040;
          endPtHoloRotation = 0.0;
        }
        break;

      case 19:
        if (offset == kLeftSide) {
          endPtX = 3.975;
          endPtY = 5.244;
          endPtHoloRotation = -60.0;
        } else if (offset == kRightSide) {
          endPtX = 3.687;
          endPtY = 5.082;
          endPtHoloRotation = -60.0;
        } else {
          endPtX = 3.827;
          endPtY = 5.178;
          endPtHoloRotation = -60.0;
        }
        break;

      case 20:
        if (offset == kLeftSide) {
          endPtX = 5.287;
          endPtY = 5.078;
          endPtHoloRotation = -120.0;
        } else if (offset == kRightSide) {
          endPtX = 5.002;
          endPtY = 5.244;
          endPtHoloRotation = -120.0;
        } else {
          endPtX = 5.162;
          endPtY = 5.194;
          endPtHoloRotation = -120.0;
        }
        break;

      case 21:
        if (offset == kLeftSide) {
          endPtX = 5.800;
          endPtY = 3.860;
          endPtHoloRotation = 180.0;
        } else if (offset == kRightSide) {
          endPtX = 5.800;
          endPtY = 4.190;
          endPtHoloRotation = 180.0;
        } else {
          endPtX = 5.825;
          endPtY = 4.040;
          endPtHoloRotation = 180.0;
        }
        break;

      case 22:
        if (offset == kLeftSide) {
          endPtX = 5.000;
          endPtY = 2.807;
          endPtHoloRotation = 120.0;
        } else if (offset == kRightSide) {
          endPtX = 5.286;
          endPtY = 2.972;
          endPtHoloRotation = 120.0;
        } else {
          endPtX = 5.165;
          endPtY = 2.880;
          endPtHoloRotation = 120.0;
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
