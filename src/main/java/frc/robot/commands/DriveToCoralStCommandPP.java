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
    // Activate a path only if its a valid Reef April Tag ID detected
    if (m_vision.rearCamTgtDectected()
        && ( // red reef april tags are numbers 6 - 11, blue are 17 - 22
        (DriverStation.getAlliance().isPresent()
                && (DriverStation.getAlliance().get() == Alliance.Red)
                && (m_vision.getRearCamBestTgtId() >= 1)
                && (m_vision.getRearCamBestTgtId() <= 2))
            || (DriverStation.getAlliance().isPresent()
                && (DriverStation.getAlliance().get() == Alliance.Blue)
                && (m_vision.getRearCamBestTgtId() >= 12)
                && (m_vision.getRearCamBestTgtId() <= 13)))) {

      // The rotation component in these poses represents the direction of travel
      double TempX = m_drive.getPose().getX();
      double TempY = m_drive.getPose().getY();
      Rotation2d TempAngle = Rotation2d.fromDegrees(180.0 + endPtHoloRotation);
      //      Pose2d startPt = m_drive.getPose();
      Pose2d startPt = new Pose2d(TempX, TempY, TempAngle);

      double TgtID = m_vision.getRearCamBestTgtId();
      DetermineEndPoint(TgtID, offset);
      // since driving backwards, the direction of travel is 180 + rotation
      Pose2d endPt =
          new Pose2d(endPtX, endPtY, Rotation2d.fromDegrees(180.0 + endPtHoloRotation)); // x,-y

      List<Waypoint> wayPoints = PathPlannerPath.waypointsFromPoses(startPt, endPt);
      PathPlannerPath path =
          new PathPlannerPath(
              wayPoints,
              new PathConstraints(
                  //              4.0, 4.0,
                  3.0, 3.0, Units.degreesToRadians(360), Units.degreesToRadians(540)),
              null,
              //  new
              // IdealStartingState(m_drive.getFFCharacterizationVelocity()*TunerConstants.kDriveGearRatio, m_drive.getRotation()),
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
        if (offset == ConstantsForHHS_Code.LeftSide) {
          endPtX = 16.852;
          endPtY = 1.364;
          endPtHoloRotation = 126.0;
        } else if (offset == ConstantsForHHS_Code.RightSide) {
          endPtX = 15.941;
          endPtY = 0.702;
          endPtHoloRotation = 126.0;
        } else {
          endPtX = 16.373;
          endPtY = 1.108;
          endPtHoloRotation = 126.0;
        }
        break;

      case 2:
        if (offset == ConstantsForHHS_Code.LeftSide) {
          endPtX = 15.941;
          endPtY = 7.325;
          endPtHoloRotation = -126.0;
        } else if (offset == ConstantsForHHS_Code.RightSide) {
          endPtX = 16.852;
          endPtY = 6.647;
          endPtHoloRotation = -126.0;
        } else {
          endPtX = 16.373;
          endPtY = 7.010;
          endPtHoloRotation = -126.0;
        }
        break;
        // blue Coral Station
      case 12:
        if (offset == ConstantsForHHS_Code.LeftSide) {
          endPtX = 1.594;
          endPtY = 0.712;
          endPtHoloRotation = 54.0;
        } else if (offset == ConstantsForHHS_Code.RightSide) {
          endPtX = 0.713;
          endPtY = 1.358;
          endPtHoloRotation = 54.0;
        } else {
          endPtX = 1.229;
          endPtY = 0.983;
          endPtHoloRotation = 54.0;
        }
        break;

      case 13:
        if (offset == ConstantsForHHS_Code.LeftSide) {
          endPtX = 0.703;
          endPtY = 6.660;
          endPtHoloRotation = -54.0;
        } else if (offset == ConstantsForHHS_Code.RightSide) {
          endPtX = 1.661;
          endPtY = 7.356;
          endPtHoloRotation = -54.0;
        } else {
          endPtX = 1.183;
          endPtY = 7.011;
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
