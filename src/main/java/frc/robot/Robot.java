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

import edu.wpi.first.net.PortForwarder;
import edu.wpi.first.wpilibj.Threads;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.commands.AllMotorsBrake;
import frc.robot.commands.AllMotorsCoast;
import org.littletonrobotics.junction.LogFileUtil;
import org.littletonrobotics.junction.LoggedRobot;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.NT4Publisher;
import org.littletonrobotics.junction.wpilog.WPILOGReader;
import org.littletonrobotics.junction.wpilog.WPILOGWriter;

/**
 * The VM is configured to automatically run this class, and to call the functions corresponding to
 * each mode, as described in the TimedRobot documentation. If you change the name of this class or
 * the package after creating this project, you must also update the build.gradle file in the
 * project.
 */
public class Robot extends LoggedRobot {
  private Command autonomousCommand;
  private RobotContainer robotContainer;

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

    // Correct pose estimate with vision measurements
    var rightFrtCamPoseEst = robotContainer.s_Vision.getEstimatedGlobalPoseUsingrightFrtCamTgts();
    rightFrtCamPoseEst.ifPresent(
        estRt -> {
          // Change our trust in the measurement based on the tags we can see
          var estStdDevs = robotContainer.s_Vision.getEstimationrightFrtCamStdDevs();

          robotContainer.drive.addVisionMeasurement(
              estRt.estimatedPose.toPose2d(), estRt.timestampSeconds, estStdDevs);

          SmartDashboard.putNumber("RightCam X", estRt.estimatedPose.toPose2d().getX());
          SmartDashboard.putNumber("RightCam Y", estRt.estimatedPose.toPose2d().getY());
          SmartDashboard.putNumber(
              "RightCam Yaw", estRt.estimatedPose.toPose2d().getRotation().getDegrees());
        });
    //   } else {
    var leftFrtCamPoseEst = robotContainer.s_Vision.getEstimatedGlobalPoseUsingleftFrtCamTgts();
    leftFrtCamPoseEst.ifPresent(
        estLt -> {
          // Change our trust in the measurement based on the tags we can see
          var estStdDevs = robotContainer.s_Vision.getEstimationleftFrtCamStdDevs();

          robotContainer.drive.addVisionMeasurement(
              estLt.estimatedPose.toPose2d(), estLt.timestampSeconds, estStdDevs);

          SmartDashboard.putNumber("LeftCam X", estLt.estimatedPose.toPose2d().getX());
          SmartDashboard.putNumber("LeftCam Y", estLt.estimatedPose.toPose2d().getY());
          SmartDashboard.putNumber(
              "LeftCam Yaw", estLt.estimatedPose.toPose2d().getRotation().getDegrees());
        });
    //   }

    SmartDashboard.putNumber("LeftCamTgts", robotContainer.s_Vision.getLeftFrtCamNumOfTgts());
    SmartDashboard.putNumber("RightCamTgts", robotContainer.s_Vision.getRightFrtCamNumOfTgts());

    SmartDashboard.putNumber("Robot X", robotContainer.drive.getPose().getX());
    SmartDashboard.putNumber("Robot Y", robotContainer.drive.getPose().getY());
    SmartDashboard.putNumber("Robot Yaw", robotContainer.drive.getRotation().getDegrees());

    SmartDashboard.putNumber("Mod0 in Rotations", robotContainer.drive.getModuleAngle(0) / 360.0);
    SmartDashboard.putNumber("Mod1 in Rotations", robotContainer.drive.getModuleAngle(1) / 360.0);
    SmartDashboard.putNumber("Mod2 in Rotations", robotContainer.drive.getModuleAngle(2) / 360.0);
    SmartDashboard.putNumber("Mod3 in Rotations", robotContainer.drive.getModuleAngle(3) / 360.0);
  }

  /** This function is called once when the robot is disabled. */
  @Override
  public void disabledInit() {
    // Run the scheduler in disabled mode
    CommandScheduler.getInstance().run();
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
  public void teleopPeriodic() {}

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
