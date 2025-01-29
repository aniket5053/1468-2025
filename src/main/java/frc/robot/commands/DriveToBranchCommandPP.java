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
import frc.robot.ConstantsForHHS_Code;
import frc.robot.subsystems.VisionSubsystem;
import frc.robot.subsystems.drive.Drive;
import java.util.List;

public class DriveToBranchCommandPP extends Command {
  private double offset; // Desired yaw value to align to
  private double robotX, robotY;
  private double robotYaw, robotYawDegs, turnRads;
  private boolean isAligned = false; // Tracks whether alignment is achieved
  // Note that these 3 vars are determined in the "DetermineEndPoint" routine
  private double endPtX, endPtY, endPtHoloRotation;

  private final Drive m_drive;
  private final VisionSubsystem m_vision;

  public DriveToBranchCommandPP(Drive drive, VisionSubsystem vision, double offset) {
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
    if (m_vision.frntCamTgtDectected()
        && ( // red reef april tags are numbers 6 - 11, blue are 17 - 22
        (DriverStation.getAlliance().isPresent()
                && (DriverStation.getAlliance().get() == Alliance.Red)
                && (m_vision.getFrntCamBestTgtId() >= 6)
                && (m_vision.getFrntCamBestTgtId() <= 11))
            || (DriverStation.getAlliance().isPresent()
                && (DriverStation.getAlliance().get() == Alliance.Blue)
                && (m_vision.getFrntCamBestTgtId() >= 17)
                && (m_vision.getFrntCamBestTgtId() <= 22)))) {

      double TgtID = m_vision.getFrntCamBestTgtId();
      double Tx = m_vision.getFrntCamBestTgtX();
      double Ty = m_vision.getFrntCamBestTgtY();
      double Tyaw = m_vision.getFrntCamBestTgtYaw();

      robotX = m_drive.getPose().getX();
      robotY = m_drive.getPose().getY();
      robotYaw = m_drive.getRotation().getRadians();
      robotYawDegs = m_drive.getRotation().getDegrees();

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
                  //              4.0, 4.0,
                  3.0, 3.0, Units.degreesToRadians(360), Units.degreesToRadians(540)),
              null,
              new GoalEndState(0.0, Rotation2d.fromDegrees(endPtHoloRotation)));

      // Prevent this path from being flipped on the red alliance, since the given
      // positions are already correct
      path.preventFlipping = true;

      AutoBuilder.followPath(path).schedule();

      //////// Have to get initial yaw value from robot and calculate a delta yaw offf
      //////// the apriltag yaw

    } else {
      m_drive.stop();
      SmartDashboard.putString("DriveToTag Status", "NOTE NOT SEEN");
    }
  }
  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    SmartDashboard.putString("DriveToTag Status", "NOT ACTIVE");
    isAligned = false;
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
        // red reef
      case 6:
        if (offset == ConstantsForHHS_Code.LeftSide) {
          endPtX = 13.585;
          endPtY = 2.790;
          endPtHoloRotation = 120.0;
        } else if (offset == ConstantsForHHS_Code.RightSide) {
          endPtX = 13.874;
          endPtY = 2.952;
          endPtHoloRotation = 120.0;
        } else {
          endPtX = 13.750;
          endPtY = 2.856;
          endPtHoloRotation = 120.0;
        }
        break;

      case 7:
        if (offset == ConstantsForHHS_Code.LeftSide) {
          endPtX = 14.385;
          endPtY = 3.862;
          endPtHoloRotation = 180.0;
        } else if (offset == ConstantsForHHS_Code.RightSide) {
          endPtX = 14.385;
          endPtY = 4.191;
          endPtHoloRotation = 180.0;
        } else {
          endPtX = 14.400;
          endPtY = 4.048;
          endPtHoloRotation = 180.0;
        }
        break;

      case 8:
        if (offset == ConstantsForHHS_Code.LeftSide) {
          endPtX = 13.866;
          endPtY = 5.086;
          endPtHoloRotation = -120.0;
        } else if (offset == ConstantsForHHS_Code.RightSide) {
          endPtX = 13.578;
          endPtY = 5.250;
          endPtHoloRotation = -120.0;
        } else {
          endPtX = 13.746;
          endPtY = 5.191;
          endPtHoloRotation = -120.0;
        }
        break;

      case 9:
        if (offset == ConstantsForHHS_Code.LeftSide) {
          endPtX = 12.544;
          endPtY = 5.252;
          endPtHoloRotation = -60.0;
        } else if (offset == ConstantsForHHS_Code.RightSide) {
          endPtX = 12.263;
          endPtY = 5.086;
          endPtHoloRotation = -60.0;
        } else {
          endPtX = 12.379;
          endPtY = 5.198;
          endPtHoloRotation = -60.0;
        }
        break;

      case 10:
        if (offset == ConstantsForHHS_Code.LeftSide) {
          endPtX = 11.740;
          endPtY = 4.191;
          endPtHoloRotation = 0.0;
        } else if (offset == ConstantsForHHS_Code.RightSide) {
          endPtX = 11.740;
          endPtY = 3.862;
          endPtHoloRotation = 0.0;
        } else {
          endPtX = 11.700;
          endPtY = 4.048;
          endPtHoloRotation = 0.0;
        }
        break;

      case 11:
        if (offset == ConstantsForHHS_Code.LeftSide) {
          endPtX = 12.263;
          endPtY = 2.964;
          endPtHoloRotation = 60.0;
        } else if (offset == ConstantsForHHS_Code.RightSide) {
          endPtX = 12.544;
          endPtY = 2.798;
          endPtHoloRotation = 60.0;
        } else {
          endPtX = 12.386;
          endPtY = 2.856;
          endPtHoloRotation = 60.0;
        }
        break;
        // blue reef
      case 17:
        if (offset == ConstantsForHHS_Code.LeftSide) {
          endPtX = 3.684;
          endPtY = 2.962;
          endPtHoloRotation = 60.0;
        } else if (offset == ConstantsForHHS_Code.RightSide) {
          endPtX = 3.971;
          endPtY = 2.800;
          endPtHoloRotation = 60.0;
        } else {
          endPtX = 3.804;
          endPtY = 2.863;
          endPtHoloRotation = 60.0;
        }
        break;

      case 18:
        if (offset == ConstantsForHHS_Code.LeftSide) {
          endPtX = 3.170;
          endPtY = 4.189;
          endPtHoloRotation = 0.0;
        } else if (offset == ConstantsForHHS_Code.RightSide) {
          endPtX = 3.170;
          endPtY = 3.860;
          endPtHoloRotation = 0.0;
        } else {
          endPtX = 3.150;
          endPtY = 4.040;
          endPtHoloRotation = 0.0;
        }
        break;

      case 19:
        if (offset == ConstantsForHHS_Code.LeftSide) {
          endPtX = 3.972;
          endPtY = 5.249;
          endPtHoloRotation = -60.0;
        } else if (offset == ConstantsForHHS_Code.RightSide) {
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
        if (offset == ConstantsForHHS_Code.LeftSide) {
          endPtX = 5.291;
          endPtY = 5.084;
          endPtHoloRotation = -120.0;
        } else if (offset == ConstantsForHHS_Code.RightSide) {
          endPtX = 5.004;
          endPtY = 5.248;
          endPtHoloRotation = -120.0;
        } else {
          endPtX = 5.165;
          endPtY = 5.198;
          endPtHoloRotation = -120.0;
        }
        break;

      case 21:
        if (offset == ConstantsForHHS_Code.LeftSide) {
          endPtX = 5.805;
          endPtY = 3.860;
          endPtHoloRotation = 180.0;
        } else if (offset == ConstantsForHHS_Code.RightSide) {
          endPtX = 5.805;
          endPtY = 4.190;
          endPtHoloRotation = 180.0;
        } else {
          endPtX = 5.815;
          endPtY = 4.040;
          endPtHoloRotation = 180.0;
        }
        break;

      case 22:
        if (offset == ConstantsForHHS_Code.LeftSide) {
          endPtX = 5.005;
          endPtY = 2.800;
          endPtHoloRotation = 120.0;
        } else if (offset == ConstantsForHHS_Code.RightSide) {
          endPtX = 5.288;
          endPtY = 2.965;
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
