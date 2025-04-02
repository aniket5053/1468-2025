// Copyright 2021-2024 FRC 6328
// http://github.com/Mechanical-Advantage
//
// This program is free software; you can redistribute it and/or
// modify it under the terms of the GNU General Public License
// version 3 as published by the Free Software Foundation or
// available in the root directory of this project.
//
// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
// GNU General Public License for more details.

package frc.robot;

import static frc.robot.ConstantsMechanisms.DriveConstants.kCenter;
import static frc.robot.ConstantsMechanisms.DriveConstants.kLeftSide;
import static frc.robot.ConstantsMechanisms.DriveConstants.kRightSide;

import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.net.PortForwarder;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Threads;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.commands.AllMotorsBrake;
import frc.robot.commands.AllMotorsCoast;
import java.util.List;
import java.util.Optional;
import org.littletonrobotics.junction.LogFileUtil;
import org.littletonrobotics.junction.LoggedRobot;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.NT4Publisher;
import org.littletonrobotics.junction.wpilog.WPILOGReader;
import org.littletonrobotics.junction.wpilog.WPILOGWriter;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonUtils;
import org.photonvision.targeting.PhotonTrackedTarget;

/**
 * The VM is configured to automatically run this class, and to call the functions corresponding to
 * each mode, as described in the TimedRobot documentation. If you change the name of this class or
 * the package after creating this project, you must also update the build.gradle file in the
 * project.
 */
public class Robot extends LoggedRobot {
  private Command autonomousCommand;
  private RobotContainer robotContainer;

  private double rtCamX, rtCamY, rtCamYaw;
  private double ltCamX, ltCamY, ltCamYaw;
  private double robotX, robotY, robotYaw;

  // limit switches (The handler limit switch is defined in the Handler Subsystem)
  private DigitalInput ShootLimitSwitch = new DigitalInput(3);
  private DigitalInput CageLimitSwitch = new DigitalInput(5);

  private final SendableChooser<String> cageChooser = new SendableChooser<>();
  private String selectedOption;

  public Robot() {
    // Record metadata
    // Logger.recordMetadata("ProjectName", BuildConstants.MAVEN_NAME);
    // Logger.recordMetadata("BuildDate", BuildConstants.BUILD_DATE);
    // Logger.recordMetadata("GitSHA", BuildConstants.GIT_SHA);
    // Logger.recordMetadata("GitDate", BuildConstants.GIT_DATE);
    // Logger.recordMetadata("GitBranch", BuildConstants.GIT_BRANCH);
    // switch (BuildConstants.DIRTY) {
    //   case 0:
    //     Logger.recordMetadata("GitDirty", "All changes committed");
    //     break;
    //   case 1:
    //     Logger.recordMetadata("GitDirty", "Uncomitted changes");
    //     break;
    //   default:
    //     Logger.recordMetadata("GitDirty", "Unknown");
    //     break;
    // }

    // Set up data receivers & replay source
    switch (Constants.currentMode) {
      case REAL:
        // Running on a real robot, log to a USB stick ("/U/logs")
        Logger.addDataReceiver(new WPILOGWriter());
        Logger.addDataReceiver(new NT4Publisher());
        break;

      case SIM:
        // Running a physics simulator, log to NT
        Logger.addDataReceiver(new NT4Publisher());
        break;

      case REPLAY:
        // Replaying a log, set up replay source
        setUseTiming(false); // Run as fast as possible
        String logPath = LogFileUtil.findReplayLog();
        Logger.setReplaySource(new WPILOGReader(logPath));
        Logger.addDataReceiver(new WPILOGWriter(LogFileUtil.addPathSuffix(logPath, "_sim")));
        break;
    }

    // Start AdvantageKit logger
    Logger.start();

    // Instantiate our RobotContainer. This will perform all our button bindings,
    // and put our autonomous chooser on the dashboard.
    robotContainer = new RobotContainer();
  }

  @Override
  public void robotInit() {
    PortForwarder.add(5800, "photonvision.local", 5800);

    // Add options to the chooser
    cageChooser.setDefaultOption("Center Cage", "CenterCage");
    cageChooser.addOption("Left Cage", "LeftCage");
    cageChooser.addOption("Right Cage", "RightCage");
    // Add the chooser to the SmartDashboard
    SmartDashboard.putData("Cage Selector", cageChooser);
  }

  /** This function is called periodically during all modes. */
  @Override
  public void robotPeriodic() {
    // Switch thread to high priority to improve loop timing
    Threads.setCurrentThreadPriority(true, 99);

    // Runs the Scheduler. This is responsible for polling buttons, adding
    // newly-scheduled commands, running already-scheduled commands, removing
    // finished or interrupted commands, and running subsystem periodic() methods.
    // This must be called from the robot's periodic block in order for anything in
    // the Command-based framework to work.
    CommandScheduler.getInstance().run();

    // Return to normal thread priority
    Threads.setCurrentThreadPriority(false, 10);

    //   if (robotContainer.s_Vision.getLeftFrtCamNumOfTgts()
    //       < robotContainer.s_Vision.getRightFrtCamNumOfTgts()) {

    // Adds time-stamped gyro readings
    double timestamp = Timer.getFPGATimestamp();
    robotContainer.s_Vision.rightFrtCamPoseEstimator.addHeadingData(
        timestamp, robotContainer.drive.rawGyroRotation);
    robotContainer.s_Vision.leftFrtCamPoseEstimator.addHeadingData(
        timestamp, robotContainer.drive.rawGyroRotation);

    // Correct pose estimate with vision measurements
    var rightFrtCamPoseEst = robotContainer.s_Vision.getEstimatedGlobalPoseUsingrightFrtCamTgts();
    rightFrtCamPoseEst.ifPresent(
        estRt -> {
          // Change our trust in the measurement based on the tags we can see
          var estStdDevs = robotContainer.s_Vision.getEstimationrightFrtCamStdDevs();

          robotContainer.drive.addVisionMeasurement(
              estRt.estimatedPose.toPose2d(), estRt.timestampSeconds, estStdDevs);

          rtCamX = estRt.estimatedPose.toPose2d().getX();
          rtCamY = estRt.estimatedPose.toPose2d().getY();
          rtCamYaw = estRt.estimatedPose.toPose2d().getRotation().getDegrees();

          // outputs camera to apriltag data for right camera
          List<PhotonTrackedTarget> rtTargetsUsed = estRt.targetsUsed;
          String rtTargetsUsedString = String.format("%s", rtTargetsUsed);
          SmartDashboard.putString("Right Targets Used for Pose (SZ)", rtTargetsUsedString);

          PhotonPoseEstimator.PoseStrategy rtCameraPoseStrategy = estRt.strategy;
          String rtCamPoseStratString = String.format("%s", rtCameraPoseStrategy);
          SmartDashboard.putString("Right Cam. Pose Strategy (SZ)", rtCamPoseStratString);

          // outputs 3D Pose of robot from right camera, does not actually update pose, only used
          // for
          // robot offsets
          Transform3d rtCamToTarget = rtTargetsUsed.get(0).bestCameraToTarget;
          Optional<Pose3d> rtPose3dFromTag =
              AprilTagFields.k2025ReefscapeWelded
                  .loadAprilTagLayoutField()
                  .getTagPose(rtTargetsUsed.get(0).getFiducialId());
          Pose3d rtCamPose3d =
              PhotonUtils.estimateFieldToRobotAprilTag(
                  rtCamToTarget, rtPose3dFromTag.get(), robotContainer.s_Vision.rightcamtorobot);
          SmartDashboard.putNumber("Right Cam. Robot 3D Pose X (SZ)", rtCamPose3d.getX());
          SmartDashboard.putNumber("Right Cam. Robot 3D Pose Y (SZ)", rtCamPose3d.getY());
          SmartDashboard.putNumber("Right Cam. Robot 3D Pose Z (SZ)", rtCamPose3d.getZ());
          SmartDashboard.putNumber(
              "Right Cam. Robot 3D Pose Roll (SZ)",
              rtCamPose3d.getRotation().getX() * (180 / Math.PI));
          SmartDashboard.putNumber(
              "Right Cam. Robot 3D Pose Pitch (SZ)",
              rtCamPose3d.getRotation().getY() * (180 / Math.PI));
          SmartDashboard.putNumber(
              "Right Cam. Robot 3D Pose Yaw (SZ)",
              rtCamPose3d.getRotation().getZ() * (180 / Math.PI));
        });
    //   } else {
    var leftFrtCamPoseEst = robotContainer.s_Vision.getEstimatedGlobalPoseUsingleftFrtCamTgts();
    leftFrtCamPoseEst.ifPresent(
        estLt -> {
          // Change our trust in the measurement based on the tags we can see
          var estStdDevs = robotContainer.s_Vision.getEstimationleftFrtCamStdDevs();

          robotContainer.drive.addVisionMeasurement(
              estLt.estimatedPose.toPose2d(), estLt.timestampSeconds, estStdDevs);

          ltCamX = estLt.estimatedPose.toPose2d().getX();
          ltCamY = estLt.estimatedPose.toPose2d().getY();
          ltCamYaw = estLt.estimatedPose.toPose2d().getRotation().getDegrees();

          // outputs camera to apriltag data for left camera
          List<PhotonTrackedTarget> ltTargetsUsed = estLt.targetsUsed;
          String ltTargetsUsedString = String.format("%s", ltTargetsUsed);
          SmartDashboard.putString("Left Targets Used for Pose (SZ)", ltTargetsUsedString);

          PhotonPoseEstimator.PoseStrategy ltCameraPoseStrategy = estLt.strategy;
          String ltCamPoseStratString = String.format("%s", ltCameraPoseStrategy);
          SmartDashboard.putString("Left Cam. Pose Strategy (SZ)", ltCamPoseStratString);

          // outputs 3D Pose of robot from left camera, does not actually update pose, only used for
          // robot offsets
          Transform3d ltCamToTarget = ltTargetsUsed.get(0).bestCameraToTarget;
          Optional<Pose3d> pose3dFromTag =
              AprilTagFields.k2025ReefscapeWelded
                  .loadAprilTagLayoutField()
                  .getTagPose(ltTargetsUsed.get(0).getFiducialId());
          Pose3d ltCamPose3d =
              PhotonUtils.estimateFieldToRobotAprilTag(
                  ltCamToTarget, pose3dFromTag.get(), robotContainer.s_Vision.leftcamtotobot);
          SmartDashboard.putNumber("Left Cam. Robot 3D Pose X (SZ)", ltCamPose3d.getX());
          SmartDashboard.putNumber("Left Cam. Robot 3D Pose Y (SZ)", ltCamPose3d.getY());
          SmartDashboard.putNumber("Left Cam. Robot 3D Pose Z (SZ)", ltCamPose3d.getZ());
          SmartDashboard.putNumber(
              "Left Cam. Robot 3D Pose Roll (SZ)",
              ltCamPose3d.getRotation().getX() * (180 / Math.PI));
          SmartDashboard.putNumber(
              "Left Cam. Robot 3D Pose Pitch (SZ)",
              ltCamPose3d.getRotation().getY() * (180 / Math.PI));
          SmartDashboard.putNumber(
              "Left Cam. Robot 3D Pose Yaw (SZ)",
              ltCamPose3d.getRotation().getZ() * (180 / Math.PI));
        });
    //   }

    SmartDashboard.putNumber("LeftCamTgts", robotContainer.s_Vision.getLeftFrtCamNumOfTgts());
    SmartDashboard.putNumber("RightCamTgts", robotContainer.s_Vision.getRightFrtCamNumOfTgts());

    robotX = robotContainer.drive.getPose().getX();
    robotY = robotContainer.drive.getPose().getY();
    robotYaw = robotContainer.drive.getPose().getRotation().getDegrees();

    SmartDashboard.putNumber("Robot X", robotX);
    SmartDashboard.putNumber("Robot Y", robotY);
    SmartDashboard.putNumber("Robot Yaw", robotYaw);

    SmartDashboard.putNumber("LeftCam X", ltCamX);
    SmartDashboard.putNumber("LeftCam Y", ltCamY);
    SmartDashboard.putNumber("LeftCam Yaw", ltCamYaw);

    SmartDashboard.putNumber("RightCam X", rtCamX);
    SmartDashboard.putNumber("RightCam Y", rtCamY);
    SmartDashboard.putNumber("RightCam Yaw", rtCamYaw);

    // change Yaw from +/-180 to 0 - 360
    if ((ltCamYaw < 0.0) && (rtCamYaw > 0.0)) ltCamYaw = 360.0 + ltCamYaw;
    if ((rtCamYaw < 0.0) && (ltCamYaw > 0.0)) rtCamYaw = 360.0 + rtCamYaw;

    double aveYaw = (ltCamYaw + rtCamYaw) / 2;
    if ((robotYaw < 0.0) && (aveYaw > 0.0)) robotYaw = 360.0 + robotYaw;
    if ((aveYaw < 0.0) && (robotYaw > 0.0)) aveYaw = 360.0 + aveYaw;

    double deltaLtRtX = ltCamX - rtCamX;
    double deltaLtRtY = ltCamY - rtCamY;
    double deltaLtRtYaw = ltCamYaw - rtCamYaw;

    double deltaLtRobotX = ltCamX - robotX;
    double deltaLtRobotY = ltCamY - robotY;

    double deltaRtRobotX = rtCamX - robotX;
    double deltaRtRobotY = rtCamY - robotY;

    double deltaRobotX = robotX - (ltCamX + rtCamX) / 2;
    double deltaRobotY = robotY - (ltCamY + rtCamY) / 2;
    double deltaRobotYaw = robotYaw - aveYaw;
    SmartDashboard.putNumber("LRdX", deltaLtRtX);
    SmartDashboard.putNumber("LRdY", deltaLtRtY);
    SmartDashboard.putNumber("LRdYaw", deltaLtRtYaw);
    SmartDashboard.putNumber("BOTdX", deltaRobotX);
    SmartDashboard.putNumber("BOTdY", deltaRobotY);
    SmartDashboard.putNumber("BOTdYaw", deltaRobotYaw);

    double LtRtDist = Math.sqrt(deltaLtRtX * deltaLtRtX + deltaLtRtY * deltaLtRtY);
    double LtRobotDist = Math.sqrt(deltaLtRobotX * deltaLtRobotX + deltaLtRobotY * deltaLtRobotY);
    double RtRobotDist = Math.sqrt(deltaRtRobotX * deltaRtRobotX + deltaRtRobotY * deltaRtRobotY);

    boolean LtRtClose, LtRobotClose, RtRobotClose;
    if (LtRtDist < 0.02) LtRtClose = true;
    else LtRtClose = false;
    if (LtRobotDist < 0.02) LtRobotClose = true;
    else LtRobotClose = false;
    if (RtRobotDist < 0.02) RtRobotClose = true;
    else RtRobotClose = false;
    SmartDashboard.putBoolean("LtRtClose", LtRtClose);
    SmartDashboard.putBoolean("LtRobotClose", LtRobotClose);
    SmartDashboard.putBoolean("RtRobotClose", RtRobotClose);

    SmartDashboard.putNumber("Mod0 in Rotations", robotContainer.drive.getModuleAngle(0) / 360.0);
    SmartDashboard.putNumber("Mod1 in Rotations", robotContainer.drive.getModuleAngle(1) / 360.0);
    SmartDashboard.putNumber("Mod2 in Rotations", robotContainer.drive.getModuleAngle(2) / 360.0);
    SmartDashboard.putNumber("Mod3 in Rotations", robotContainer.drive.getModuleAngle(3) / 360.0);

    boolean isAligned = ShootLimitSwitch.get();
    // if (!isAligned) robotContainer.s_LED.setWhiteBlinking();
    SmartDashboard.putBoolean("Aligned", isAligned); // TA TODO: Need to get a working Align LS

    boolean cageCaptured = CageLimitSwitch.get();
    //   if (!cageCaptured) robotContainer.s_LED.setWhiteBlinking();
    SmartDashboard.putBoolean("CageCaptured", cageCaptured);
  }

  /** This function is called once when the robot is disabled. */
  @Override
  public void disabledInit() {
    // Run the scheduler in disabled mode
    CommandScheduler.getInstance().run();

    robotContainer.s_LED.setOrangePattern();
  }

  /** This function is called periodically when disabled. */
  @Override
  public void disabledPeriodic() {

    if (robotContainer.driverRightJoystick.getRawButton(10))
      new AllMotorsBrake(
              robotContainer.s_Elbow,
              robotContainer.s_Elevator,
              robotContainer.s_Wrist,
              robotContainer.drive)
          .schedule();
    if (robotContainer.driverRightJoystick.getRawButton(11))
      new AllMotorsCoast(
              robotContainer.s_Elbow,
              robotContainer.s_Elevator,
              robotContainer.s_Wrist,
              robotContainer.drive)
          .schedule();

    // Run the scheduler in disabled mode
    CommandScheduler.getInstance().run();
  }

  /** This autonomous runs the autonomous command selected by your {@link RobotContainer} class. */
  @Override
  public void autonomousInit() {
    autonomousCommand = robotContainer.getAutonomousCommand();

    // schedule the autonomous command (example)
    if (autonomousCommand != null) {
      autonomousCommand.schedule();
    }
  }

  /** This function is called periodically during autonomous. */
  @Override
  public void autonomousPeriodic() {}

  /** This function is called once when teleop is enabled. */
  @Override
  public void teleopInit() {
    // This makes sure that the autonomous stops running when
    // teleop starts running. If you want the autonomous to
    // continue until interrupted by another command, remove
    // this line or comment it out.
    if (autonomousCommand != null) {
      autonomousCommand.cancel();
    }
  }

  /** This function is called periodically during operator control. */
  @Override
  public void teleopPeriodic() {

    selectedOption = cageChooser.getSelected();
    switch (selectedOption) {
      case "CenterCage":
        robotContainer.drive.setCageLocation(kCenter);
        break;
      case "LeftCage":
        robotContainer.drive.setCageLocation(kLeftSide);
        break;
      case "RightCage":
        robotContainer.drive.setCageLocation(kRightSide);
        break;
      default:
        robotContainer.drive.setCageLocation(kCenter);
        break;
    }

    SmartDashboard.putNumber("CageLocation", robotContainer.drive.getCageLocation());
  }

  /** This function is called once when test mode is enabled. */
  @Override
  public void testInit() {
    // Cancels all running commands at the start of test mode.
    CommandScheduler.getInstance().cancelAll();
  }

  /** This function is called periodically during test mode. */
  @Override
  public void testPeriodic() {}

  /** This function is called once when the robot is first started up. */
  @Override
  public void simulationInit() {}

  /** This function is called periodically whilst in simulation. */
  @Override
  public void simulationPeriodic() {}
}
