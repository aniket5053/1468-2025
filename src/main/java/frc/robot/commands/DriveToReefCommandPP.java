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
    double TgtID_SZ = m_drive.getTrgtIdToDriveTo_SZ();
    // double TgtID = m_vision.getTrgtIdToDriveTo();
    if (TgtID_SZ != 99.0) {

      // The rotation component in these poses represents the direction of travel
      double TempX = m_drive.getPose().getX();
      double TempY = m_drive.getPose().getY();
      Rotation2d TempAngle = Rotation2d.fromDegrees(endPtHoloRotation);
      //      Pose2d startPt = m_drive.getPose();
      Pose2d startPt = new Pose2d(TempX, TempY, TempAngle);

      DetermineEndPoint(TgtID_SZ, offset);

      // creates intermediate point for on-the-fly path that has the bumper 14 inches from the end
      // point
      double interPtX = (endPtX - (0.3556 * Math.cos(endPtHoloRotation * (Math.PI / 180))));
      double interPtY = (endPtY - (0.3556 * Math.sin(endPtHoloRotation * (Math.PI / 180))));
      Pose2d interPt = new Pose2d(interPtX, interPtY, Rotation2d.fromDegrees(endPtHoloRotation));

      Pose2d endPt = new Pose2d(endPtX, endPtY, Rotation2d.fromDegrees(endPtHoloRotation)); // x,-y

      List<Waypoint> wayPoints = PathPlannerPath.waypointsFromPoses(startPt, interPt, endPt);
      PathPlannerPath path =
          new PathPlannerPath(
              wayPoints,
              new PathConstraints(
                  4.0,
                  4.0, // was 3,3
                  // 3.0, 3.0,
                  Units.degreesToRadians(540), // was 360
                  Units.degreesToRadians(720)), // was 540
              null,
              new GoalEndState(
                  0.0,
                  Rotation2d.fromDegrees(
                      endPtHoloRotation))); // allows the robot to glide into the reef, 0.1 seems to
      // not do much

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

        // new coordinates

        ////////////////////////////// red reef /////////////////////////////////////////
        /*    s/s coefficients
          case 6:
            if (offset == kLeftSide) {
              endPtX = 13.55429891;
              endPtY = 2.83923003;
              endPtHoloRotation = 120.0;
            } else if (offset == kRightSide) {
              endPtX = 13.83894067;
              endPtY = 3.00356803;
              endPtHoloRotation = 120.0;
            } else {
              endPtX = 13.69661979;
              endPtY = 2.92139903;
              endPtHoloRotation = 120.0;
            }
            break;

          case 7:
            if (offset == kLeftSide) {
              endPtX = 14.33428679;
              endPtY = 3.86159132;
              endPtHoloRotation = 180.0;
            } else if (offset == kRightSide) {
              endPtX = 14.33428679;
              endPtY = 4.19026732;
              endPtHoloRotation = 180.0;
            } else {
              endPtX = 14.33428679;
              endPtY = 4.02592932;
              endPtHoloRotation = 180.0;
            }
            break;

          case 8:
            if (offset == kLeftSide) {
              endPtX = 13.83888988;
              endPtY = 5.04826129;
              endPtHoloRotation = -120.0;
            } else if (offset == kRightSide) {
              endPtX = 13.55424812;
              endPtY = 5.21259929;
              endPtHoloRotation = -120.0;
            } else {
              endPtX = 13.696569;
              endPtY = 5.13043029;
              endPtHoloRotation = -120.0;
            }
            break;

          case 9:
            if (offset == kLeftSide) {
              endPtX = 12.56350509;
              endPtY = 5.21256997;
              endPtHoloRotation = -60.0;
            } else if (offset == kRightSide) {
              endPtX = 12.27886333;
              endPtY = 5.04823197;
              endPtHoloRotation = -60.0;
            } else {
              endPtX = 12.42118421;
              endPtY = 5.13040097;
              endPtHoloRotation = -60.0;
            }
            break;

          case 10:
            if (offset == kLeftSide) {
              endPtX = 11.78351721;
              endPtY = 4.19020868;
              endPtHoloRotation = 0.0;
            } else if (offset == kRightSide) {
              endPtX = 11.78351721;
              endPtY = 3.86153268;
              endPtHoloRotation = 0.0;
            } else {
              endPtX = 11.78351721;
              endPtY = 4.02587068;
              endPtHoloRotation = 0.0;
            }
            break;

          case 11:
            if (offset == kLeftSide) {
              endPtX = 12.27891412;
              endPtY = 3.00353871;
              endPtHoloRotation = 60.0;
            } else if (offset == kRightSide) {
              endPtX = 12.56355588;
              endPtY = 2.83920071;
              endPtHoloRotation = 60.0;
            } else {
              endPtX = 12.421235;
              endPtY = 2.92136971;
              endPtHoloRotation = 60.0;
            }
            break;

            ////////////////////////////// blue reef -shift/////////////////////////////////////////
          case 22:
            if (offset == kLeftSide) {
              endPtX = 13.55429891 - 8.569579;
              endPtY = 2.83923003;
              endPtHoloRotation = 120.0;
            } else if (offset == kRightSide) {
              endPtX = 13.83894067 - 8.569579;
              endPtY = 3.00356803;
              endPtHoloRotation = 120.0;
            } else {
              endPtX = 13.69661979 - 8.569579;
              endPtY = 2.92139903;
              endPtHoloRotation = 120.0;
            }
            break;

          case 21:
            if (offset == kLeftSide) {
              endPtX = 14.33428679 - 8.569579;
              endPtY = 3.86159132;
              endPtHoloRotation = 180.0;
            } else if (offset == kRightSide) {
              endPtX = 14.33428679 - 8.569579;
              endPtY = 4.19026732;
              endPtHoloRotation = 180.0;
            } else {
              endPtX = 14.33428679 - 8.569579;
              endPtY = 4.02592932;
              endPtHoloRotation = 180.0;
            }
            break;

          case 20:
            if (offset == kLeftSide) {
              endPtX = 13.83888988 - 8.569579;
              endPtY = 5.04826129;
              endPtHoloRotation = -120.0;
            } else if (offset == kRightSide) {
              endPtX = 13.55424812 - 8.569579;
              endPtY = 5.21259929;
              endPtHoloRotation = -120.0;
            } else {
              endPtX = 13.696569 - 8.569579;
              endPtY = 5.13043029;
              endPtHoloRotation = -120.0;
            }
            break;

          case 19:
            if (offset == kLeftSide) {
              endPtX = 12.56350509 - 8.569579;
              endPtY = 5.21256997;
              endPtHoloRotation = -60.0;
            } else if (offset == kRightSide) {
              endPtX = 12.27886333 - 8.569579;
              endPtY = 5.04823197;
              endPtHoloRotation = -60.0;
            } else {
              endPtX = 12.42118421 - 8.569579;
              endPtY = 5.13040097;
              endPtHoloRotation = -60.0;
            }
            break;

          case 18:
            if (offset == kLeftSide) {
              endPtX = 11.78351721 - 8.569579;
              endPtY = 4.19020868;
              endPtHoloRotation = 0.0;
            } else if (offset == kRightSide) {
              endPtX = 11.78351721 - 8.569579;
              endPtY = 3.86153268;
              endPtHoloRotation = 0.0;
            } else {
              endPtX = 11.78351721 - 8.569579;
              endPtY = 4.02587068;
              endPtHoloRotation = 0.0;
            }
            break;

          case 17:
            if (offset == kLeftSide) {
              endPtX = 12.27891412 - 8.569579;
              endPtY = 3.00353871;
              endPtHoloRotation = 60.0;
            } else if (offset == kRightSide) {
              endPtX = 12.56355588 - 8.569579;
              endPtY = 2.83920071;
              endPtHoloRotation = 60.0;
            } else {
              endPtX = 12.421235 - 8.569579;
              endPtY = 2.92136971;
              endPtHoloRotation = 60.0;
            }
            break;
        end of S/S coeffients */
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
          endPtX = 4.982; // was 5.000;
          endPtY = 2.800; // was 2.807;
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
