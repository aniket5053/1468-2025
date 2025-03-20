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

      Pose2d endPt = new Pose2d(endPtX, endPtY, Rotation2d.fromDegrees(endPtHoloRotation));

      List<Waypoint> wayPoints = PathPlannerPath.waypointsFromPoses(startPt, interPt, endPt);
      PathPlannerPath path =
          new PathPlannerPath(
              wayPoints,
              new PathConstraints(
                  3.0,
                  3.0, // was 3,3
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

        /////////////////////////////////////////////////////////////////////////////////
        //    Updated CAD coefficients - At reef                                       //
        ////////////////////////////// red reef /////////////////////////////////////////
        // case 6:
        //   if (offset == kLeftSide) {
        //     endPtX = 13.554;
        //     endPtY = 2.839;
        //     endPtHoloRotation = 120.0;
        //   } else if (offset == kRightSide) {
        //     endPtX = 13.839;
        //     endPtY = 3.004;
        //     endPtHoloRotation = 120.0;
        //   } else {
        //     endPtX = 13.697;
        //     endPtY = 2.921;
        //     endPtHoloRotation = 120.0;
        //   }
        //   break;

        // case 7:
        //   if (offset == kLeftSide) {
        //     endPtX = 14.334;
        //     endPtY = 3.862;
        //     endPtHoloRotation = 180.0;
        //   } else if (offset == kRightSide) {
        //     endPtX = 14.334;
        //     endPtY = 4.190;
        //     endPtHoloRotation = 180.0;
        //   } else {
        //     endPtX = 14.334;
        //     endPtY = 4.026;
        //     endPtHoloRotation = 180.0;
        //   }
        //   break;

        // case 8:
        //   if (offset == kLeftSide) {
        //     endPtX = 13.839;
        //     endPtY = 5.048;
        //     endPtHoloRotation = -120.0;
        //   } else if (offset == kRightSide) {
        //     endPtX = 13.554;
        //     endPtY = 5.213;
        //     endPtHoloRotation = -120.0;
        //   } else {
        //     endPtX = 13.697;
        //     endPtY = 5.130;
        //     endPtHoloRotation = -120.0;
        //   }
        //   break;

        // case 9:
        //   if (offset == kLeftSide) {
        //     endPtX = 12.564;
        //     endPtY = 5.213;
        //     endPtHoloRotation = -60.0;
        //   } else if (offset == kRightSide) {
        //     endPtX = 12.279;
        //     endPtY = 5.048;
        //     endPtHoloRotation = -60.0;
        //   } else {
        //     endPtX = 12.421;
        //     endPtY = 5.130;
        //     endPtHoloRotation = -60.0;
        //   }
        //   break;

        // case 10:
        //   if (offset == kLeftSide) {
        //     endPtX = 11.784;
        //     endPtY = 4.190;
        //     endPtHoloRotation = 0.0;
        //   } else if (offset == kRightSide) {
        //     endPtX = 11.784;
        //     endPtY = 3.862;
        //     endPtHoloRotation = 0.0;
        //   } else {
        //     endPtX = 11.784;
        //     endPtY = 4.026;
        //     endPtHoloRotation = 0.0;
        //   }
        //   break;

        // case 11:
        //   if (offset == kLeftSide) {
        //     endPtX = 12.279;
        //     endPtY = 3.004;
        //     endPtHoloRotation = 60.0;
        //   } else if (offset == kRightSide) {
        //     endPtX = 12.564;
        //     endPtY = 2.839;
        //     endPtHoloRotation = 60.0;
        //   } else {
        //     endPtX = 12.421;
        //     endPtY = 2.921;
        //     endPtHoloRotation = 60.0;
        //   }
        //   break;

        //   ////////////////////////////// blue reef
        // -shift/////////////////////////////////////////
        // case 22:
        //   if (offset == kLeftSide) {
        //     endPtX = 4.985;
        //     endPtY = 2.839;
        //     endPtHoloRotation = 120.0;
        //   } else if (offset == kRightSide) {
        //     endPtX = 5.269;
        //     endPtY = 3.004;
        //     endPtHoloRotation = 120.0;
        //   } else {
        //     endPtX = 5.127;
        //     endPtY = 2.921;
        //     endPtHoloRotation = 120.0;
        //   }
        //   break;

        // case 21:
        //   if (offset == kLeftSide) {
        //     endPtX = 5.765;
        //     endPtY = 3.862;
        //     endPtHoloRotation = 180.0;
        //   } else if (offset == kRightSide) {
        //     endPtX = 5.765;
        //     endPtY = 4.190;
        //     endPtHoloRotation = 180.0;
        //   } else {
        //     endPtX = 5.765;
        //     endPtY = 4.026;
        //     endPtHoloRotation = 180.0;
        //   }
        //   break;

        // case 20:
        //   if (offset == kLeftSide) {
        //     endPtX = 5.269;
        //     endPtY = 5.048;
        //     endPtHoloRotation = -120.0;
        //   } else if (offset == kRightSide) {
        //     endPtX = 4.985;
        //     endPtY = 5.213;
        //     endPtHoloRotation = -120.0;
        //   } else {
        //     endPtX = 5.127;
        //     endPtY = 5.130;
        //     endPtHoloRotation = -120.0;
        //   }
        //   break;

        // case 19:
        //   if (offset == kLeftSide) {
        //     endPtX = 3.994;
        //     endPtY = 5.213;
        //     endPtHoloRotation = -60.0;
        //   } else if (offset == kRightSide) {
        //     endPtX = 3.709;
        //     endPtY = 5.048;
        //     endPtHoloRotation = -60.0;
        //   } else {
        //     endPtX = 3.852;
        //     endPtY = 5.130;
        //     endPtHoloRotation = -60.0;
        //   }
        //   break;

        // case 18:
        //   if (offset == kLeftSide) {
        //     endPtX = 3.214;
        //     endPtY = 4.190;
        //     endPtHoloRotation = 0.0;
        //   } else if (offset == kRightSide) {
        //     endPtX = 3.214;
        //     endPtY = 3.862;
        //     endPtHoloRotation = 0.0;
        //   } else {
        //     endPtX = 3.214;
        //     endPtY = 4.026;
        //     endPtHoloRotation = 0.0;
        //   }
        //   break;

        // case 17:
        //   if (offset == kLeftSide) {
        //     endPtX = 3.709;
        //     endPtY = 3.004;
        //     endPtHoloRotation = 60.0;
        //   } else if (offset == kRightSide) {
        //     endPtX = 3.994;
        //     endPtY = 2.839;
        //     endPtHoloRotation = 60.0;
        //   } else {
        //     endPtX = 3.852;
        //     endPtY = 2.921;
        //     endPtHoloRotation = 60.0;
        //   }
        //   break;
        //     end of Updated CAD Coefficients - At Reef */

        /////////////////////////////////////////////////////////////////////////////////
        //    Updated CAD coefficients - 1/2 inch into reef                              //
        ////////////////////////////// red reef /////////////////////////////////////////
        // case 6:
        //   if (offset == kLeftSide) {
        //     endPtX = 13.548;
        //     endPtY = 2.850;
        //     endPtHoloRotation = 120.0;
        //   } else if (offset == kRightSide) {
        //     endPtX = 13.833;
        //     endPtY = 3.015;
        //     endPtHoloRotation = 120.0;
        //   } else {
        //     endPtX = 13.709;
        //     endPtY = 2.899;
        //     endPtHoloRotation = 120.0;
        //   }
        //   break;

        // case 7:
        //   if (offset == kLeftSide) {
        //     endPtX = 14.322;
        //     endPtY = 3.862;
        //     endPtHoloRotation = 180.0;
        //   } else if (offset == kRightSide) {
        //     endPtX = 14.322;
        //     endPtY = 4.190;
        //     endPtHoloRotation = 180.0;
        //   } else {
        //     endPtX = 14.360;
        //     endPtY = 4.026;
        //     endPtHoloRotation = 180.0;
        //   }
        //   break;

        // case 8:
        //   if (offset == kLeftSide) {
        //     endPtX = 13.833;
        //     endPtY = 5.037;
        //     endPtHoloRotation = -120.0;
        //   } else if (offset == kRightSide) {
        //     endPtX = 13.548;
        //     endPtY = 5.202;
        //     endPtHoloRotation = -120.0;
        //   } else {
        //     endPtX = 13.709;
        //     endPtY = 5.152;
        //     endPtHoloRotation = -120.0;
        //   }
        //   break;

        // case 9:
        //   if (offset == kLeftSide) {
        //     endPtX = 12.570;
        //     endPtY = 5.202;
        //     endPtHoloRotation = -60.0;
        //   } else if (offset == kRightSide) {
        //     endPtX = 12.285;
        //     endPtY = 5.037;
        //     endPtHoloRotation = -60.0;
        //   } else {
        //     endPtX = 12.408;
        //     endPtY = 5.152;
        //     endPtHoloRotation = -60.0;
        //   }
        //   break;

        // case 10:
        //   if (offset == kLeftSide) {
        //     endPtX = 11.796;
        //     endPtY = 4.190;
        //     endPtHoloRotation = 0.0;
        //   } else if (offset == kRightSide) {
        //     endPtX = 11.796;
        //     endPtY = 3.862;
        //     endPtHoloRotation = 0.0;
        //   } else {
        //     endPtX = 11.758;
        //     endPtY = 4.026;
        //     endPtHoloRotation = 0.0;
        //   }
        //   break;

        // case 11:
        //   if (offset == kLeftSide) {
        //     endPtX = 12.285;
        //     endPtY = 3.015;
        //     endPtHoloRotation = 60.0;
        //   } else if (offset == kRightSide) {
        //     endPtX = 12.570;
        //     endPtY = 2.850;
        //     endPtHoloRotation = 60.0;
        //   } else {
        //     endPtX = 12.409;
        //     endPtY = 2.899;
        //     endPtHoloRotation = 60.0;
        //   }
        //   break;

        //   ////////////////////////////// blue reef
        // -shift/////////////////////////////////////////
        // case 22:
        //   if (offset == kLeftSide) {
        //     endPtX = 4.978;
        //     endPtY = 2.850;
        //     endPtHoloRotation = 120.0;
        //   } else if (offset == kRightSide) {
        //     endPtX = 5.263;
        //     endPtY = 3.015;
        //     endPtHoloRotation = 120.0;
        //   } else {
        //     endPtX = 5.140;
        //     endPtY = 2.899;
        //     endPtHoloRotation = 120.0;
        //   }
        //   break;

        // case 21:
        //   if (offset == kLeftSide) {
        //     endPtX = 5.752;
        //     endPtY = 3.862;
        //     endPtHoloRotation = 180.0;
        //   } else if (offset == kRightSide) {
        //     endPtX = 5.752;
        //     endPtY = 4.190;
        //     endPtHoloRotation = 180.0;
        //   } else {
        //     endPtX = 5.790;
        //     endPtY = 4.026;
        //     endPtHoloRotation = 180.0;
        //   }
        //   break;

        // case 20:
        //   if (offset == kLeftSide) {
        //     endPtX = 5.263;
        //     endPtY = 5.037;
        //     endPtHoloRotation = -120.0;
        //   } else if (offset == kRightSide) {
        //     endPtX = 4.978;
        //     endPtY = 5.202;
        //     endPtHoloRotation = -120.0;
        //   } else {
        //     endPtX = 5.140;
        //     endPtY = 5.152;
        //     endPtHoloRotation = -120.0;
        //   }
        //   break;

        // case 19:
        //   if (offset == kLeftSide) {
        //     endPtX = 4.000;
        //     endPtY = 5.202;
        //     endPtHoloRotation = -60.0;
        //   } else if (offset == kRightSide) {
        //     endPtX = 3.716;
        //     endPtY = 5.037;
        //     endPtHoloRotation = -60.0;
        //   } else {
        //     endPtX = 3.839;
        //     endPtY = 5.152;
        //     endPtHoloRotation = -60.0;
        //   }
        //   break;

        // case 18:
        //   if (offset == kLeftSide) {
        //     endPtX = 3.227;
        //     endPtY = 4.190;
        //     endPtHoloRotation = 0.0;
        //   } else if (offset == kRightSide) {
        //     endPtX = 3.227;
        //     endPtY = 3.862;
        //     endPtHoloRotation = 0.0;
        //   } else {
        //     endPtX = 3.189;
        //     endPtY = 4.026;
        //     endPtHoloRotation = 0.0;
        //   }
        //   break;

        // case 17:
        //   if (offset == kLeftSide) {
        //     endPtX = 3.716;
        //     endPtY = 3.015;
        //     endPtHoloRotation = 60.0;
        //   } else if (offset == kRightSide) {
        //     endPtX = 4.000;
        //     endPtY = 2.850;
        //     endPtHoloRotation = 60.0;
        //   } else {
        //     endPtX = 3.839;
        //     endPtY = 2.899;
        //     endPtHoloRotation = 60.0;
        //   }
        //   break;
        //   //     end of Updated CAD Coefficients - 1/2 inch into reef */

        /////////////////////////////////////////////////////////////////////////////////
        //    Updated CAD coefficients - 1/2 in from reef                              //
        ////////////////////////////// red reef /////////////////////////////////////////
      case 6:
        if (offset == kLeftSide) {
          endPtX = 13.561;
          endPtY = 2.828;
          endPtHoloRotation = 120.0;
        } else if (offset == kRightSide) {
          endPtX = 13.845;
          endPtY = 2.993;
          endPtHoloRotation = 120.0;
        } else {
          endPtX = 13.709;
          endPtY = 2.899;
          endPtHoloRotation = 120.0;
        }
        break;

      case 7:
        if (offset == kLeftSide) {
          endPtX = 14.347;
          endPtY = 3.862;
          endPtHoloRotation = 180.0;
        } else if (offset == kRightSide) {
          endPtX = 14.347;
          endPtY = 4.190;
          endPtHoloRotation = 180.0;
        } else {
          endPtX = 14.360;
          endPtY = 4.026;
          endPtHoloRotation = 180.0;
        }
        break;

      case 8:
        if (offset == kLeftSide) {
          endPtX = 13.845;
          endPtY = 5.059;
          endPtHoloRotation = -120.0;
        } else if (offset == kRightSide) {
          endPtX = 13.561;
          endPtY = 5.224;
          endPtHoloRotation = -120.0;
        } else {
          endPtX = 13.709;
          endPtY = 5.152;
          endPtHoloRotation = -120.0;
        }
        break;

      case 9:
        if (offset == kLeftSide) {
          endPtX = 12.557;
          endPtY = 5.224;
          endPtHoloRotation = -60.0;
        } else if (offset == kRightSide) {
          endPtX = 12.273;
          endPtY = 5.059;
          endPtHoloRotation = -60.0;
        } else {
          endPtX = 12.408;
          endPtY = 5.152;
          endPtHoloRotation = -60.0;
        }
        break;

      case 10:
        if (offset == kLeftSide) {
          endPtX = 11.771;
          endPtY = 4.190;
          endPtHoloRotation = 0.0;
        } else if (offset == kRightSide) {
          endPtX = 11.771;
          endPtY = 3.862;
          endPtHoloRotation = 0.0;
        } else {
          endPtX = 11.758;
          endPtY = 4.026;
          endPtHoloRotation = 0.0;
        }
        break;

      case 11:
        if (offset == kLeftSide) {
          endPtX = 12.273;
          endPtY = 2.993;
          endPtHoloRotation = 60.0;
        } else if (offset == kRightSide) {
          endPtX = 12.557;
          endPtY = 2.828;
          endPtHoloRotation = 60.0;
        } else {
          endPtX = 12.409;
          endPtY = 2.899;
          endPtHoloRotation = 60.0;
        }
        break;

        ////////////////////////////// blue reef
        /////////////////////////////////////////
      case 22:
        if (offset == kLeftSide) {
          endPtX = 4.991;
          endPtY = 2.828;
          endPtHoloRotation = 120.0;
        } else if (offset == kRightSide) {
          endPtX = 5.276;
          endPtY = 2.993;
          endPtHoloRotation = 120.0;
        } else {
          endPtX = 5.140;
          endPtY = 2.899;
          endPtHoloRotation = 120.0;
        }
        break;

      case 21:
        if (offset == kLeftSide) {
          endPtX = 5.777;
          endPtY = 3.862;
          endPtHoloRotation = 180.0;
        } else if (offset == kRightSide) {
          endPtX = 5.777;
          endPtY = 4.190;
          endPtHoloRotation = 180.0;
        } else {
          endPtX = 5.790;
          endPtY = 4.026;
          endPtHoloRotation = 180.0;
        }
        break;

      case 20:
        if (offset == kLeftSide) {
          endPtX = 5.276;
          endPtY = 5.059;
          endPtHoloRotation = -120.0;
        } else if (offset == kRightSide) {
          endPtX = 4.991;
          endPtY = 5.224;
          endPtHoloRotation = -120.0;
        } else {
          endPtX = 5.140;
          endPtY = 5.152;
          endPtHoloRotation = -120.0;
        }
        break;

      case 19:
        if (offset == kLeftSide) {
          endPtX = 3.988;
          endPtY = 5.224;
          endPtHoloRotation = -60.0;
        } else if (offset == kRightSide) {
          endPtX = 3.703;
          endPtY = 5.059;
          endPtHoloRotation = -60.0;
        } else {
          endPtX = 3.839;
          endPtY = 5.152;
          endPtHoloRotation = -60.0;
        }
        break;

      case 18:
        if (offset == kLeftSide) {
          endPtX = 3.201;
          endPtY = 4.190;
          endPtHoloRotation = 0.0;
        } else if (offset == kRightSide) {
          endPtX = 3.201;
          endPtY = 3.862;
          endPtHoloRotation = 0.0;
        } else {
          endPtX = 3.189;
          endPtY = 4.026;
          endPtHoloRotation = 0.0;
        }
        break;

      case 17:
        if (offset == kLeftSide) {
          endPtX = 3.703;
          endPtY = 2.993;
          endPtHoloRotation = 60.0;
        } else if (offset == kRightSide) {
          endPtX = 3.988;
          endPtY = 2.828;
          endPtHoloRotation = 60.0;
        } else {
          endPtX = 3.839;
          endPtY = 2.899;
          endPtHoloRotation = 60.0;
        }
        break;
        //   //     end of Updated CAD Coefficients - 1/2 inch from reef */

        /////////////////////////////////////////////////////////////////////////////////
        //    Updated CAD coefficients - 1 in from reef                              //
        ////////////////////////////// red reef /////////////////////////////////////////
        // case 6:
        //   if (offset == kLeftSide) {
        //     endPtX = 13.567;
        //     endPtY = 2.817;
        //     endPtHoloRotation = 120.0;
        //   } else if (offset == kRightSide) {
        //     endPtX = 13.852;
        //     endPtY = 2.982;
        //     endPtHoloRotation = 120.0;
        //   } else {
        //     endPtX = 13.709;
        //     endPtY = 2.899;
        //     endPtHoloRotation = 120.0;
        //   }
        //   break;

        // case 7:
        //   if (offset == kLeftSide) {
        //     endPtX = 14.360;
        //     endPtY = 3.862;
        //     endPtHoloRotation = 180.0;
        //   } else if (offset == kRightSide) {
        //     endPtX = 14.360;
        //     endPtY = 4.190;
        //     endPtHoloRotation = 180.0;
        //   } else {
        //     endPtX = 14.360;
        //     endPtY = 4.026;
        //     endPtHoloRotation = 180.0;
        //   }
        //   break;

        // case 8:
        //   if (offset == kLeftSide) {
        //     endPtX = 13.852;
        //     endPtY = 5.070;
        //     endPtHoloRotation = -120.0;
        //   } else if (offset == kRightSide) {
        //     endPtX = 13.567;
        //     endPtY = 5.235;
        //     endPtHoloRotation = -120.0;
        //   } else {
        //     endPtX = 13.709;
        //     endPtY = 5.152;
        //     endPtHoloRotation = -120.0;
        //   }
        //   break;

        // case 9:
        //   if (offset == kLeftSide) {
        //     endPtX = 12.551;
        //     endPtY = 5.235;
        //     endPtHoloRotation = -60.0;
        //   } else if (offset == kRightSide) {
        //     endPtX = 12.266;
        //     endPtY = 5.070;
        //     endPtHoloRotation = -60.0;
        //   } else {
        //     endPtX = 12.408;
        //     endPtY = 5.152;
        //     endPtHoloRotation = -60.0;
        //   }
        //   break;

        // case 10:
        //   if (offset == kLeftSide) {
        //     endPtX = 11.758;
        //     endPtY = 4.190;
        //     endPtHoloRotation = 0.0;
        //   } else if (offset == kRightSide) {
        //     endPtX = 11.758;
        //     endPtY = 3.862;
        //     endPtHoloRotation = 0.0;
        //   } else {
        //     endPtX = 11.758;
        //     endPtY = 4.026;
        //     endPtHoloRotation = 0.0;
        //   }
        //   break;

        // case 11:
        //   if (offset == kLeftSide) {
        //     endPtX = 12.266;
        //     endPtY = 2.982;
        //     endPtHoloRotation = 60.0;
        //   } else if (offset == kRightSide) {
        //     endPtX = 12.551;
        //     endPtY = 2.817;
        //     endPtHoloRotation = 60.0;
        //   } else {
        //     endPtX = 12.409;
        //     endPtY = 2.899;
        //     endPtHoloRotation = 60.0;
        //   }
        //   break;

        //   ////////////////////////////// blue reef
        // -shift/////////////////////////////////////////
        // case 22:
        //   if (offset == kLeftSide) {
        //     endPtX = 4.997;
        //     endPtY = 2.817;
        //     endPtHoloRotation = 120.0;
        //   } else if (offset == kRightSide) {
        //     endPtX = 5.282;
        //     endPtY = 2.982;
        //     endPtHoloRotation = 120.0;
        //   } else {
        //     endPtX = 5.140;
        //     endPtY = 2.899;
        //     endPtHoloRotation = 120.0;
        //   }
        //   break;

        // case 21:
        //   if (offset == kLeftSide) {
        //     endPtX = 5.790;
        //     endPtY = 3.862;
        //     endPtHoloRotation = 180.0;
        //   } else if (offset == kRightSide) {
        //     endPtX = 5.790;
        //     endPtY = 4.190;
        //     endPtHoloRotation = 180.0;
        //   } else {
        //     endPtX = 5.790;
        //     endPtY = 4.026;
        //     endPtHoloRotation = 180.0;
        //   }
        //   break;

        // case 20:
        //   if (offset == kLeftSide) {
        //     endPtX = 5.282;
        //     endPtY = 5.070;
        //     endPtHoloRotation = -120.0;
        //   } else if (offset == kRightSide) {
        //     endPtX = 4.997;
        //     endPtY = 5.235;
        //     endPtHoloRotation = -120.0;
        //   } else {
        //     endPtX = 5.140;
        //     endPtY = 5.152;
        //     endPtHoloRotation = -120.0;
        //   }
        //   break;

        // case 19:
        //   if (offset == kLeftSide) {
        //     endPtX = 3.981;
        //     endPtY = 5.235;
        //     endPtHoloRotation = -60.0;
        //   } else if (offset == kRightSide) {
        //     endPtX = 3.697;
        //     endPtY = 5.070;
        //     endPtHoloRotation = -60.0;
        //   } else {
        //     endPtX = 3.839;
        //     endPtY = 5.152;
        //     endPtHoloRotation = -60.0;
        //   }
        //   break;

        // case 18:
        //   if (offset == kLeftSide) {
        //     endPtX = 3.189;
        //     endPtY = 4.190;
        //     endPtHoloRotation = 0.0;
        //   } else if (offset == kRightSide) {
        //     endPtX = 3.189;
        //     endPtY = 3.862;
        //     endPtHoloRotation = 0.0;
        //   } else {
        //     endPtX = 3.189;
        //     endPtY = 4.026;
        //     endPtHoloRotation = 0.0;
        //   }
        //   break;

        // case 17:
        //   if (offset == kLeftSide) {
        //     endPtX = 3.697;
        //     endPtY = 2.982;
        //     endPtHoloRotation = 60.0;
        //   } else if (offset == kRightSide) {
        //     endPtX = 3.981;
        //     endPtY = 2.817;
        //     endPtHoloRotation = 60.0;
        //   } else {
        //     endPtX = 3.839;
        //     endPtY = 2.899;
        //     endPtHoloRotation = 60.0;
        //   }
        //   break;
        //   //     end of Updated CAD Coefficients - 1 inch from reef */

        /* //PathPlanner Coefficients
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
        //End of PathPlanner Coefficients */

      default:
        endPtX = m_drive.getPose().getX();
        endPtY = m_drive.getPose().getY();
        endPtHoloRotation = m_drive.getPose().getRotation().getDegrees();
        break;
    }
  }
}
