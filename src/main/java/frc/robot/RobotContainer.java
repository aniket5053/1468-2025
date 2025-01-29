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

import com.pathplanner.lib.auto.AutoBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.POVButton;
import frc.robot.commands.CANdleConfigCommands;
// import frc.robot.ConstantsForHHS_Code.*;
import frc.robot.commands.DriveCommands;
import frc.robot.commands.DriveToBranchCommandPP;
import frc.robot.commands.DriveToCoralStCommandPP;
import frc.robot.commands.DriveToProcessorCommandPP;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.CANdleSubsystem;
import frc.robot.subsystems.VisionSubsystem;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.drive.GyroIO;
import frc.robot.subsystems.drive.GyroIOPigeon2;
import frc.robot.subsystems.drive.ModuleIO;
import frc.robot.subsystems.drive.ModuleIOSim;
import frc.robot.subsystems.drive.ModuleIOTalonFX;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {

  // Joysticks used
  final Joystick driverLeftJoystick = new Joystick(0);
  final Joystick driverRightJoystick = new Joystick(1);

  final Joystick operatorJoystick = new Joystick(3);
  final Joystick testOprJoystick = new Joystick(2); // this was for testing purposes only - Tom 2024

  // Subsystems
  final Drive drive;
  final VisionSubsystem s_Vision = new VisionSubsystem();
  final CANdleSubsystem m_candleSubsystem = new CANdleSubsystem(operatorJoystick);

  // Dashboard inputs
  //  private final LoggedDashboardChooser<Command> autoChooser;
  private final SendableChooser<Command> autoChooser;

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    switch (Constants.currentMode) {
      case REAL:
        // Real robot, instantiate hardware IO implementations
        drive =
            new Drive(
                new GyroIOPigeon2(),
                new ModuleIOTalonFX(TunerConstants.FrontLeft),
                new ModuleIOTalonFX(TunerConstants.FrontRight),
                new ModuleIOTalonFX(TunerConstants.BackLeft),
                new ModuleIOTalonFX(TunerConstants.BackRight));
        break;

      case SIM:
        // Sim robot, instantiate physics sim IO implementations
        drive =
            new Drive(
                new GyroIO() {},
                new ModuleIOSim(TunerConstants.FrontLeft),
                new ModuleIOSim(TunerConstants.FrontRight),
                new ModuleIOSim(TunerConstants.BackLeft),
                new ModuleIOSim(TunerConstants.BackRight));
        break;

      default:
        // Replayed robot, disable IO implementations
        drive =
            new Drive(
                new GyroIO() {},
                new ModuleIO() {},
                new ModuleIO() {},
                new ModuleIO() {},
                new ModuleIO() {});
        break;
    }

    // Set up auto routines
    //   autoChooser = new LoggedDashboardChooser<>("Auto Choices", AutoBuilder.buildAutoChooser());

    // Set up SysId routines
    //   autoChooser.addOption(
    //       "Drive Wheel Radius Characterization",
    // DriveCommands.wheelRadiusCharacterization(drive));
    //   autoChooser.addOption(
    //       "Drive Simple FF Characterization", DriveCommands.feedforwardCharacterization(drive));
    //   autoChooser.addOption(
    //       "Drive SysId (Quasistatic Forward)",
    //       drive.sysIdQuasistatic(SysIdRoutine.Direction.kForward));
    //   autoChooser.addOption(
    //       "Drive SysId (Quasistatic Reverse)",
    //       drive.sysIdQuasistatic(SysIdRoutine.Direction.kReverse));
    //   autoChooser.addOption(
    //       "Drive SysId (Dynamic Forward)", drive.sysIdDynamic(SysIdRoutine.Direction.kForward));
    //   autoChooser.addOption(
    //       "Drive SysId (Dynamic Reverse)", drive.sysIdDynamic(SysIdRoutine.Direction.kReverse));

    // Build an auto chooser. This will use Commands.none() as the default option.
    autoChooser = AutoBuilder.buildAutoChooser();

    // Another option that allows you to specify the default auto by its name
    // autoChooser = AutoBuilder.buildAutoChooser("My Default Auto");

    SmartDashboard.putData("Auto Chooser", autoChooser);

    // Configure the button bindings
    configureButtonBindings();
  }

  /**
   * Use this method to define your button->command mappings. Buttons can be created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a {@link
   * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {
    // Driver Buttons
    //   final JoystickButton driveToNote = new JoystickButton(driverLeftJoystick, 1);

    //    final JoystickButton aimAtAprilTag = new JoystickButton(driverRightJoystick, 1);
    final JoystickButton drivePPProcessorAprilTag = new JoystickButton(driverLeftJoystick, 2);
    final JoystickButton drivePPLeftReefAprilTag = new JoystickButton(driverLeftJoystick, 3);
    final JoystickButton drivePPRightReefAprilTag = new JoystickButton(driverLeftJoystick, 4);
    final JoystickButton drivePPLeftCoralStAprilTag = new JoystickButton(driverLeftJoystick, 5);
    final JoystickButton drivePPRightCoralStAprilTag = new JoystickButton(driverLeftJoystick, 6);

    final POVButton lockTo000 = new POVButton(driverLeftJoystick, 0);
    final POVButton lockTo060 = new POVButton(driverLeftJoystick, 45);
    final POVButton lockTo090 = new POVButton(driverLeftJoystick, 90);
    final POVButton lockTo120 = new POVButton(driverLeftJoystick, 135);
    final POVButton lockTo180 = new POVButton(driverLeftJoystick, 180);
    final POVButton lockToM120 = new POVButton(driverLeftJoystick, 225);
    final POVButton lockToM090 = new POVButton(driverLeftJoystick, 270);
    final POVButton lockToM060 = new POVButton(driverLeftJoystick, 315);

    // old way
    //  final JoystickButton driveLeftReefAprilTag = new JoystickButton(driverLeftJoystick, 3);
    //  final JoystickButton driveRightReefAprilTag = new JoystickButton(driverLeftJoystick, 4);
    //  final JoystickButton driveLeftCoralAprilTag = new JoystickButton(driverLeftJoystick, 5);
    //  final JoystickButton driveRightCoralAprilTag = new JoystickButton(driverLeftJoystick, 6);
    final JoystickButton resetGyro = new JoystickButton(driverRightJoystick, 7);
    final JoystickButton xPattern = new JoystickButton(driverRightJoystick, 8);
    final JoystickButton stopPath = new JoystickButton(driverRightJoystick, 9);

    // Default command, normal field-relative drive
    drive.setDefaultCommand(
        DriveCommands.joystickDrive(
            drive,
            () -> -driverLeftJoystick.getY(),
            () -> -driverLeftJoystick.getX(),
            () -> -driverRightJoystick.getX()));

    // Reset gyro to 0°
    resetGyro.onTrue(
        Commands.runOnce(
                () -> drive.setPose(new Pose2d(drive.getPose().getTranslation(), new Rotation2d())),
                drive)
            .ignoringDisable(true));

    // Switch to X pattern
    xPattern.onTrue(Commands.runOnce(drive::stopWithX, drive));
    // Stop! - This will stop a "on the fly" pathplanner path
    stopPath.onTrue(Commands.runOnce(drive::stop, drive));

    // Lock to 0°
    lockTo000.whileTrue(
        DriveCommands.joystickDriveAtAngle(
            drive,
            () -> -driverLeftJoystick.getY(),
            () -> -driverLeftJoystick.getX(),
            () -> Rotation2d.fromDegrees(0.0)));

    // Lock to 60°
    lockTo060.whileTrue(
        DriveCommands.joystickDriveAtAngle(
            drive,
            () -> -driverLeftJoystick.getY(),
            () -> -driverLeftJoystick.getX(),
            () -> Rotation2d.fromDegrees(58.0)));

    // Lock to 90°
    lockTo090.whileTrue(
        DriveCommands.joystickDriveAtAngle(
            drive,
            () -> -driverLeftJoystick.getY(),
            () -> -driverLeftJoystick.getX(),
            () -> Rotation2d.fromDegrees(90.0)));

    // Lock to 120°
    lockTo120.whileTrue(
        DriveCommands.joystickDriveAtAngle(
            drive,
            () -> -driverLeftJoystick.getY(),
            () -> -driverLeftJoystick.getX(),
            () -> Rotation2d.fromDegrees(122.0)));

    // Lock to 180°
    lockTo180.whileTrue(
        DriveCommands.joystickDriveAtAngle(
            drive,
            () -> -driverLeftJoystick.getY(),
            () -> -driverLeftJoystick.getX(),
            () -> Rotation2d.fromDegrees(180.0)));

    // Lock to -120°
    lockToM120.whileTrue(
        DriveCommands.joystickDriveAtAngle(
            drive,
            () -> -driverLeftJoystick.getY(),
            () -> -driverLeftJoystick.getX(),
            () -> Rotation2d.fromDegrees(-122.0)));

    // Lock to -90°
    lockToM090.whileTrue(
        DriveCommands.joystickDriveAtAngle(
            drive,
            () -> -driverLeftJoystick.getY(),
            () -> -driverLeftJoystick.getX(),
            () -> Rotation2d.fromDegrees(-90.0)));

    // Lock to -60°
    lockToM060.whileTrue(
        DriveCommands.joystickDriveAtAngle(
            drive,
            () -> -driverLeftJoystick.getY(),
            () -> -driverLeftJoystick.getX(),
            () -> Rotation2d.fromDegrees(-58.0)));

    drivePPProcessorAprilTag.debounce(0.10).onTrue(new DriveToProcessorCommandPP(drive));
    drivePPLeftReefAprilTag
        .debounce(0.10)
        .onTrue(new DriveToBranchCommandPP(drive, s_Vision, ConstantsForHHS_Code.LeftSide));
    drivePPRightReefAprilTag
        .debounce(0.10)
        .onTrue(new DriveToBranchCommandPP(drive, s_Vision, ConstantsForHHS_Code.RightSide));

    drivePPLeftCoralStAprilTag
        .debounce(0.10)
        .onTrue(new DriveToCoralStCommandPP(drive, s_Vision, ConstantsForHHS_Code.LeftSide));
    drivePPRightCoralStAprilTag
        .debounce(0.10)
        .onTrue(new DriveToCoralStCommandPP(drive, s_Vision, ConstantsForHHS_Code.RightSide));
    // old way
    //    driveLeftReefAprilTag.whileTrue(new DriveToBranchCommand(drive, s_Vision, -6.5));
    //    driveRightReefAprilTag.whileTrue(new DriveToBranchCommand(drive, s_Vision, +6.5));
    // driveLeftCoralAprilTag.whileTrue(new DriveToCoralCommand(drive, s_Vision, -6.5));
    // driveRightCoralAprilTag.whileTrue(new DriveToCoralCommand(drive, s_Vision, +6.5));

    // Drive to note example
    //   driveToNote.whileTrue(new DriveToAprilTagCommand(drive, s_Vision));
    // Lock to AprilTag example: ID 7 (blue speaker center)
    // aimAtAprilTag.whileTrue(new AimAtAprilTagCommand(drive, s_Vision, 7));

    ////////////////////////////////  CANdle test commands  //////////////////////////////

    new JoystickButton(operatorJoystick, 3)
        .onTrue(new RunCommand(m_candleSubsystem::setColors, m_candleSubsystem));
    new JoystickButton(operatorJoystick, 4)
        .onTrue(new RunCommand(m_candleSubsystem::incrementAnimation, m_candleSubsystem));
    new JoystickButton(operatorJoystick, 5)
        .onTrue(new RunCommand(m_candleSubsystem::decrementAnimation, m_candleSubsystem));

    new POVButton(operatorJoystick, Constants.MaxBrightnessAngle)
        .onTrue(new CANdleConfigCommands.ConfigBrightness(m_candleSubsystem, 1.0));
    new POVButton(operatorJoystick, Constants.MidBrightnessAngle)
        .onTrue(new CANdleConfigCommands.ConfigBrightness(m_candleSubsystem, 0.3));
    new POVButton(operatorJoystick, Constants.ZeroBrightnessAngle)
        .onTrue(new CANdleConfigCommands.ConfigBrightness(m_candleSubsystem, 0));
    new POVButton(operatorJoystick, Constants.ChangeDirectionAngle)
        .onTrue(new RunCommand(() -> m_candleSubsystem.toggleAnimDirection(), m_candleSubsystem));

    new JoystickButton(operatorJoystick, 9)
        .onTrue(new RunCommand(() -> m_candleSubsystem.clearAllAnims(), m_candleSubsystem));
    new JoystickButton(operatorJoystick, 10)
        .onTrue(new RunCommand(() -> m_candleSubsystem.toggle5VOverride(), m_candleSubsystem));
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // return autoChooser.get();
    return autoChooser.getSelected();
  }
}
