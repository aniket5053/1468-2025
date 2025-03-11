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
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
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

    // The rotation component in these poses represents the direction of travel
    Pose2d startPt = m_drive.getPose();
    double currentRotation = m_drive.getPose().getRotation().getDegrees();
    if (Math.abs(currentRotation) > 90.0) endPtHoloRotation = 180.0;
    else endPtHoloRotation = 0.0;
    Pose2d endPt;
    if (DriverStation.getAlliance().isPresent()
        && (DriverStation.getAlliance().get() == Alliance.Red)) {
      //  Real Coordinates
      endPtX = 9.8;
      if (offset == kCenter) endPtY = 1.915;
      if (offset == kLeftSide) endPtY = 0.815;
      if (offset == kRightSide) endPtY = 3.015;
      // endPtHoloRotation = 180.0;
      endPt = new Pose2d(endPtX, endPtY, Rotation2d.fromDegrees(endPtHoloRotation));
    } else if (DriverStation.getAlliance().isPresent()
        && (DriverStation.getAlliance().get() == Alliance.Blue)) {
      endPtX = 7.75;
      if (offset == kCenter) endPtY = 6.138;
      if (offset == kLeftSide) endPtY = 7.238;
      if (offset == kRightSide) endPtY = 5.038;
      // endPtHoloRotation = 0.0;
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
    SmartDashboard.putString("DriveToTag Cage", "NOT ACTIVE");
    m_drive.stop();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
