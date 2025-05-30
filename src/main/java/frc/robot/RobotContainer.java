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

import static frc.robot.ConstantsMechanisms.*;
import static frc.robot.ConstantsMechanisms.DriveConstants.*;
import static frc.robot.ConstantsMechanisms.HandlerConstants.kHoldForever;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.POVButton;
import frc.robot.ConstantsMechanisms.ElbowConstants;
import frc.robot.ConstantsMechanisms.ElevatorConstants;
import frc.robot.ConstantsMechanisms.HandlerConstants;
import frc.robot.ConstantsMechanisms.WristConstants;
import frc.robot.commands.*;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.*;
import frc.robot.subsystems.drive.*;

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
  // Operator Left (2) and Right(3)
  final Joystick testOprJoystick = new Joystick(2); // this was for testing purposes only - Tom 2024
  final Joystick operatorJoystick = new Joystick(3);

  // Subsystems
  final Drive drive;
  final ElbowSubsystem s_Elbow = new ElbowSubsystem();
  final ElevatorSubsystem s_Elevator = new ElevatorSubsystem();
  final WristSubsystem s_Wrist = new WristSubsystem();
  final HandlerSubsystem s_Handler = new HandlerSubsystem();
  //   final ClimberSubsystem s_Climber = new ClimberSubsystem();
  final ClimberSubsystem s_Climber;
  final VisionSubsystem s_Vision = new VisionSubsystem();
  final LEDSubsystem s_LED = new LEDSubsystem();

  // Dashboard inputs
  //  private final LoggedDashboardChooser<Command> autoChooser;
  private final SendableChooser<Command> autoChooser;

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {

    s_Climber = new ClimberSubsystem();

    // Register named commands for PathPlanner Auto Routines
    // The drive commands are in the PathPlanner AutoRoutines only
    // Add a few LED Commands for the driving

    // The following commands are for Left and Right Autos which try to score Coral
    // also for the Center Auto scoring of preloaded coral
    NamedCommands.registerCommand( // Only use in DEADLINE with a drive cmd as the DEADLINE
        "HoldHomePos", // - poor name now since it goes to Pre4 -
        // start arm home, once drivng forward get arm up
        new ArmHome(s_Elevator, s_Elbow, s_Wrist, kToleranceHold)
            .withTimeout(.1)
            .andThen(new ArmPreLevel4Auto(s_Elevator, s_Elbow, s_Wrist, kToleranceHold)));

    NamedCommands.registerCommand(
        "ArmToL4", // Auto Tolerance so command ends when arm at correct position
        new ArmLevel4Auto(s_Elevator, s_Elbow, s_Wrist, kToleranceAuto));

    NamedCommands.registerCommand(
        "Shoot",
        Commands.race( // Hold arm still until shot complete, then move arm down and end command
                // timeout shoot in case it gets stuck
                new HandlerShootCoralOutAuto(s_Handler).withTimeout(1.0),
                new ArmLevel4(s_Elevator, s_Elbow, s_Wrist, kToleranceHold))
            // Auto Tolerance so command ends when arm at correct position
            // .andThen(new ArmHome(s_Elevator, s_Elbow, s_Wrist, kToleranceAuto)));

            .andThen(
                new ArmPreLevel4Auto(s_Elevator, s_Elbow, s_Wrist, kToleranceAuto)
                    .alongWith(new HandlerShootCoralOut(s_Handler))));

    NamedCommands.registerCommand(
        "HarvestCoral", // Only use as the DEADLINE with a drive Cmd (this cmd ends when coral
        // harvested)
        Commands.race( // Hold arm still until Coral harvested, then end command
            new HandlerHarvestCoral(s_Handler),
            new ArmCoralStation(s_Elevator, s_Elbow, s_Wrist, kToleranceHold)));
    //            .andThen(new ArmHome(s_Elevator, s_Elbow, s_Wrist, kToleranceAuto)));

    // The following commands are for the Center Auto which trys to get and score algae
    // (after scoring intial preloaded coral)
    NamedCommands.registerCommand(
        "HoldAlgaeHomePos", // Only use in race with a drive cmd - THIS COMMAND DOESNT FINISH
        Commands.race(
            new ArmHomeAfterAlgaeShotAuto(s_Elevator, s_Elbow, s_Wrist, kToleranceHold),
            (new HandlerHarvestAlgae(s_Handler, kHoldForever))));

    NamedCommands.registerCommand(
        "GetAlgaeHigh",
        Commands.race( // Hold arm still until Algae harvested, then move arm home
                new ArmAlgaeHigh(s_Elevator, s_Elbow, s_Wrist, kToleranceHold),
                new HandlerHarvestAlgae(s_Handler, !kHoldForever))
            .andThen(
                Commands.race( // Hold Algae while Moving Home
                    new HandlerHarvestAlgae(s_Handler, kHoldForever),
                    new ArmHomeFromAlgaeHigh(s_Elevator, s_Elbow, s_Wrist, kToleranceAuto * 4.0))));

    NamedCommands.registerCommand(
        "GetAlgaeLow",
        Commands.race( // Hold arm still until Algae harvested, then move arm home
                new ArmAlgaeLow(s_Elevator, s_Elbow, s_Wrist, kToleranceHold),
                new HandlerHarvestAlgae(s_Handler, !kHoldForever))
            .andThen(
                Commands.race( // Hold Algae while Moving Home
                    new HandlerHarvestAlgae(s_Handler, kHoldForever),
                    new ArmHomeFromAlgaeLow(s_Elevator, s_Elbow, s_Wrist, kToleranceAuto * 4.0))));

    NamedCommands.registerCommand(
        "HoldAlgaePreBargePos", // Only use in race with a drive cmd - THIS COMMAND NEVER FINISHES
        Commands.race(
            new ArmPreAlgaeBargeNet(s_Elevator, s_Elbow, s_Wrist, kToleranceHold),
            (new HandlerHarvestAlgae(s_Handler, kHoldForever))));

    NamedCommands.registerCommand(
        "ShootAlgae",
        Commands.race( // Move arm up to barge holding algae, then shoot, then home and end command
                new ArmAlgaeBargeNet(s_Elevator, s_Elbow, s_Wrist, kToleranceAuto),
                new HandlerHarvestAlgae(s_Handler, kHoldForever))
            .andThen(
                Commands.race(
                    new HandlerShootAlgaeOut(s_Handler),
                    new ArmAlgaeBargeNet(s_Elevator, s_Elbow, s_Wrist, kToleranceHold)))
            .andThen(new ArmHomeAfterAlgaeShotAuto(s_Elevator, s_Elbow, s_Wrist, kToleranceAuto)));

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

    // Show what command your subsystem is running on the SmartDashboard - doesnt work
    // SmartDashboard.putData(drive);
    // SmartDashboard.putData(s_Elbow);
    // SmartDashboard.putData(s_Elevator);
    // SmartDashboard.putData(s_Wrist);
    // SmartDashboard.putData(s_Handler);
    // SmartDashboard.putData(s_Vision);
    // SmartDashboard.putData(s_LED);

    // Put Cage Selection binary buttons on SmartDashboard
    SmartDashboard.putBoolean("LeftCage", false);
    SmartDashboard.putBoolean("CenterCage", false);
    SmartDashboard.putBoolean("RightCage", false);

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

    // Default command, normal field-relative drive
    drive.setDefaultCommand(
        DriveCommands.joystickDrive(
            drive,
            () -> -driverLeftJoystick.getY(),
            () -> -driverLeftJoystick.getX(),
            () -> -driverRightJoystick.getX()));

    // Driver Buttons - Left Joystick  ---------------------------------------------
    // Driver Buttons - Left Joystick  ---------------------------------------------

    //    new JoystickButton(driverLeftJoystick, 1)
    new JoystickButton(driverLeftJoystick, 3)
        .debounce(0.10)
        .onTrue(
            new DriveToCoralStCommandPP(drive, s_Vision, kLeftSide)
                .alongWith(s_LED.setDriveCmdStarted())
                .andThen(s_LED.setDriveCmdFinished()));

    // Coral Station Right Side
    new JoystickButton(driverLeftJoystick, 4)
        //    new JoystickButton(driverRightJoystick, 1)
        .debounce(0.10)
        .onTrue(
            new DriveToCoralStCommandPP(drive, s_Vision, kRightSide)
                .alongWith(s_LED.setDriveCmdStarted())
                .andThen(s_LED.setDriveCmdFinished()));

    // Configuration Buttons - comment out when done testing
    // new JoystickButton(driverLeftJoystick, 11)
    //     .debounce(0.10)
    //     .onTrue(DriveCommands.feedforwardCharacterization(drive));
    // new JoystickButton(driverLeftJoystick, 12)
    //     .debounce(0.10)
    //     .onTrue(DriveCommands.wheelRadiusCharacterization(drive));
    // Configuration Buttons - comment out when done testing

    new JoystickButton(driverLeftJoystick, 6)
        .debounce(0.10)
        .onTrue(
            new DriveToProcessorCommandPP(drive)
                .alongWith(s_LED.setDriveCmdStarted())
                .andThen(s_LED.setDriveCmdFinished()));

    new POVButton(driverLeftJoystick, 0)
        //   new JoystickButton(driverLeftJoystick, 3)
        .debounce(0.10)
        .onTrue(
            new DriveToCageCommandPP(drive, kCenter)
                .alongWith(s_LED.setDriveCmdStarted())
                .andThen(s_LED.setDriveCmdFinished()));

    // Driver Buttons - Right Joystick --------------------------------------------
    // Driver Buttons - Right Joystick --------------------------------------------

    // Drive with left joystick, rotate to angle facing reef
    new JoystickButton(driverRightJoystick, 2)
        .whileTrue(
            DriveCommands.joystickDriveAtAngle(
                drive,
                () -> -driverLeftJoystick.getY(),
                () -> -driverLeftJoystick.getX(),
                () -> Rotation2d.fromDegrees(drive.getDesiredRotation())));

    // Center Lt, Rt Drive to Reef Commands
    //    new JoystickButton(driverRightJoystick, 3)
    new JoystickButton(driverRightJoystick, 1)
        .debounce(0.10)
        .onTrue(
            new DriveToReefCommandPP(drive, s_Vision, kCenter)
                .alongWith(s_LED.setDriveCmdStarted())
                .andThen(s_LED.setDriveCmdFinished()));
    new JoystickButton(driverRightJoystick, 4)
        .debounce(0.10)
        .onTrue(
            new DriveToReefCommandPP(drive, s_Vision, kLeftSide)
                .alongWith(s_LED.setDriveCmdStarted())
                .andThen(s_LED.setDriveCmdFinished()));
    new JoystickButton(driverRightJoystick, 5)
        .debounce(0.10)
        .onTrue(
            new DriveToReefCommandPP(drive, s_Vision, kRightSide)
                .alongWith(s_LED.setDriveCmdStarted())
                .andThen(s_LED.setDriveCmdFinished()));

    // Switch to X pattern
    new JoystickButton(driverLeftJoystick, 1)
        //    new JoystickButton(driverRightJoystick, 6)
        .debounce(0.10)
        .onTrue(Commands.runOnce(drive::stopWithX, drive));

    // Reset gyro to 0°
    new JoystickButton(driverRightJoystick, 7)
        .onTrue(
            Commands.runOnce(
                    () ->
                        drive.setPose(
                            new Pose2d(drive.getPose().getTranslation(), new Rotation2d())),
                    drive)
                .ignoringDisable(true));

    // Stop! - This will stop a "on the fly" pathplanner path
    new JoystickButton(driverRightJoystick, 9).onTrue(Commands.runOnce(drive::stop, drive));

    new JoystickButton(driverRightJoystick, 10)
        .onTrue(new AllMotorsBrake(s_Elbow, s_Elevator, s_Wrist, drive));
    new JoystickButton(driverRightJoystick, 11)
        .onTrue(new AllMotorsCoast(s_Elbow, s_Elevator, s_Wrist, drive));

    // Operator - Left  Joystick - Test Functions---------------------------
    // Operator - Left  Joystick - Test Functions---------------------------
    // Operator - Left  Joystick - Test Functions---------------------------

    // Clear Harvestor button - eject!
    new JoystickButton(testOprJoystick, 1)
        .debounce(0.10)
        .onTrue(
            new InstantCommand(
                () ->
                    s_Handler.setHandlerVoltageVelos(
                        HandlerConstants.kHandlerCoralInSpeed,
                        HandlerConstants.kHandlerCoralInSpeed)));
    new JoystickButton(testOprJoystick, 1)
        .debounce(0.10)
        .onFalse(new InstantCommand(() -> s_Handler.stop(), s_Handler));

    // Home button
    new JoystickButton(testOprJoystick, 2)
        .debounce(0.10)
        .onTrue(new ArmHome(s_Elevator, s_Elbow, s_Wrist, kToleranceHold));

    // 90 degree elbow and wrist - 0in elevator (straight up - good for lab setup)
    new JoystickButton(testOprJoystick, 3)
        .debounce(0.10)
        .onTrue(
            new MM_ElbowToPosition(s_Elbow, ElbowConstants.kStartAngle, kToleranceHold)
                .alongWith(
                    new MM_WristToPosition(s_Wrist, WristConstants.kHomeAngle, kToleranceHold))
                .alongWith(
                    new MM_ElevatorToPosition(
                        s_Elevator, ElevatorConstants.kStartPos, kToleranceHold)));

    new JoystickButton(testOprJoystick, 4)
        .debounce(0.10)
        .onTrue(
            new InstantCommand(
                () -> s_Climber.setClimberVoltageVelos(ClimberConstants.kClimberInSpeed)));
    new JoystickButton(testOprJoystick, 4)
        .debounce(0.10)
        .onFalse(new InstantCommand(() -> s_Climber.stop()));

    // new JoystickButton(testOprJoystick, 4)
    //     .debounce(0.10)
    //     .onTrue(new MM_ElbowToPosition(s_Elbow, ElbowConstants.kProcessorAngle, kToleranceHold));

    // new JoystickButton(testOprJoystick, 5)
    //     .debounce(0.10)
    //     .onTrue(new MM_ElevatorToPosition(s_Elevator, ElevatorConstants.kStartPos,
    // kToleranceHold));
    // new JoystickButton(testOprJoystick, 6)
    //     .debounce(0.10)
    //     .onTrue(
    //         new MM_ElevatorToPosition(s_Elevator, ElevatorConstants.kLevel4Pos, kToleranceHold));

    // new JoystickButton(testOprJoystick, 8)
    //     .debounce(0.10)
    //     .onTrue(new MM_WristToPosition(s_Wrist, WristConstants.kStartAngle, kToleranceHold));
    // new JoystickButton(testOprJoystick, 9)
    //     .debounce(0.10)
    //     .onTrue(new MM_WristToPosition(s_Wrist, WristConstants.kZeroOffset, kToleranceHold));

    // Test PreLevel4 button
    // new JoystickButton(testOprJoystick, 8)
    //     .debounce(0.10)
    //     .onTrue(new HandlerShootCoralOutAuto(s_Handler));
    // // .onTrue(new ArmPreLevel4Auto(s_Elevator, s_Elbow, s_Wrist, kToleranceHold))
    // .onFalse(new ArmLevel4Auto(s_Elevator, s_Elbow, s_Wrist, kToleranceHold));

    // Harvest In / eject up!
    new JoystickButton(testOprJoystick, 9)
        .debounce(0.10)
        .onTrue(
            new InstantCommand(
                () ->
                    s_Handler.setHandlerVoltageVelos(
                        HandlerConstants.kHandlerAlgaeInSpeed,
                        HandlerConstants.kHandlerAlgaeInSpeed)));
    new JoystickButton(testOprJoystick, 9)
        .debounce(0.10)
        .onFalse(new InstantCommand(() -> s_Handler.stop(), s_Handler));

    new JoystickButton(testOprJoystick, 10)
        .debounce(0.10)
        .onTrue(new ArmClimbLower(s_Elevator, s_Elbow, s_Wrist, kToleranceHold));

    new JoystickButton(testOprJoystick, 11)
        .debounce(0.10)
        .onTrue(new ArmClimbHigher(s_Elevator, s_Elbow, s_Wrist, kToleranceHold));

    // Operator - Right  Joystick - Real Functions---------------------------
    // Operator - Right  Joystick - Real Functions---------------------------
    // Operator - Right  Joystick - Real Functions---------------------------

    // Shoot Coral and then move arm to home position
    new JoystickButton(operatorJoystick, 1)
        .debounce(0.10)
        .onTrue(
            new HandlerShootCoralOut(s_Handler)
                .andThen(new ArmHome(s_Elevator, s_Elbow, s_Wrist, kToleranceHold)));

    // move arm to coral station position, and harvest coral. Once harvested move arm to Home
    // position - signal coral harvested for 1 sec (using LED shoot Cmd)
    new JoystickButton(operatorJoystick, 2)
        .debounce(0.10)
        .onTrue(
            Commands.race(
                    new ArmCoralStation(s_Elevator, s_Elbow, s_Wrist, kToleranceHold),
                    new HandlerHarvestCoral(s_Handler))
                .alongWith(s_LED.setOperatorCmdStarted())
                .andThen(s_LED.setOperatorShootCmd())
                .andThen(new ArmHome(s_Elevator, s_Elbow, s_Wrist, kToleranceHold)));

    // Reef Level 3,4,2,and 1 positions (L4 has a 2 step move)
    new JoystickButton(operatorJoystick, 3)
        .debounce(0.10)
        .onTrue(new ArmLevel3(s_Elevator, s_Elbow, s_Wrist, kToleranceHold));

    new JoystickButton(operatorJoystick, 4)
        .debounce(0.10)
        .onTrue(new ArmLevel2(s_Elevator, s_Elbow, s_Wrist, kToleranceHold));

    new JoystickButton(operatorJoystick, 5)
        //       .debounce(0.10)
        //       .onTrue(new ArmPreLevel4(s_Elevator, s_Elbow, s_Wrist, kToleranceHold));
        //   new JoystickButton(operatorJoystick, 5)
        .debounce(0.10)
        .onTrue(new ArmLevel4(s_Elevator, s_Elbow, s_Wrist, kToleranceHold));

    new JoystickButton(operatorJoystick, 6)
        .debounce(0.10)
        .onTrue(new ArmLevel1(s_Elevator, s_Elbow, s_Wrist, kToleranceHold));

    // Climb and Unclimb buttons
    new JoystickButton(operatorJoystick, 7)
        .debounce(0.10)
        .onTrue(
            new ArmUnClimb(s_Elevator, s_Elbow, s_Wrist, kToleranceHold)
                .alongWith(
                    new InstantCommand(
                        () -> s_Climber.setClimberVoltageVelos(ClimberConstants.kClimberInSpeed))));

    // new JoystickButton(operatorJoystick, 8)
    //     .debounce(0.10)
    //     .onTrue(
    //         new ArmPreClimb1(s_Elevator, s_Elbow, s_Wrist, kToleranceHold)
    //             .alongWith(new InstantCommand(() -> s_Climber.stop())));

    new JoystickButton(operatorJoystick, 8)
        .debounce(0.10)
        .onTrue(
            new ArmPreClimb2(s_Elevator, s_Elbow, s_Wrist, kToleranceHold)
                .alongWith(new InstantCommand(() -> s_Climber.stop())));

    new JoystickButton(operatorJoystick, 9)
        .debounce(0.10)
        .onTrue(
            new ArmClimb(s_Elevator, s_Elbow, s_Wrist, kToleranceHold)
                .alongWith(new InstantCommand(() -> s_Climber.stop())));

    // Arm Up to Net - holding algae
    new JoystickButton(operatorJoystick, 11)
        .debounce(0.10)
        .onTrue(
            new HandlerHarvestAlgae(s_Handler, kHoldForever)
                .alongWith(new ArmAlgaeBargeNet(s_Elevator, s_Elbow, s_Wrist, kToleranceHold)));

    new JoystickButton(operatorJoystick, 12)
        .debounce(0.10)
        .onTrue(
            new HandlerShootAlgaeOut(s_Handler)
                .andThen(new ArmHomeAfterAlgaeShot(s_Elevator, s_Elbow, s_Wrist, kToleranceHold)));

    new POVButton(operatorJoystick, 0)
        .debounce(0.10)
        .onTrue(
            Commands
                .race( // Move Arm to High Algae Position, and Hold arm still until Algae harvested,
                    // then move arm home
                    new ArmAlgaeHigh(s_Elevator, s_Elbow, s_Wrist, kToleranceHold),
                    new HandlerHarvestAlgae(s_Handler, !kHoldForever))
                .andThen(
                    Commands.parallel( // Hold Algae while Moving Home
                        new HandlerHarvestAlgae(s_Handler, kHoldForever),
                        new ArmHomeFromAlgaeHigh(s_Elevator, s_Elbow, s_Wrist, kToleranceHold))));

    new POVButton(operatorJoystick, 180)
        .debounce(0.10)
        .onTrue(
            Commands
                .race( // Move Arm to Low Algae Position, and Hold arm still until Algae harvested,
                    // then move arm home
                    new ArmAlgaeLow(s_Elevator, s_Elbow, s_Wrist, kToleranceHold),
                    new HandlerHarvestAlgae(s_Handler, !kHoldForever))
                .andThen(
                    Commands.parallel( // Hold Algae while Moving Home
                        new HandlerHarvestAlgae(s_Handler, kHoldForever),
                        new ArmHomeFromAlgaeLow(s_Elevator, s_Elbow, s_Wrist, kToleranceHold))));

    new POVButton(operatorJoystick, 90)
        .debounce(0.10)
        .onTrue( // move arm to processor position holding algae
            new ArmAlgaeProcessor(s_Elevator, s_Elbow, s_Wrist, kToleranceHold)
                .alongWith(new HandlerHarvestAlgae(s_Handler, kHoldForever)));

    //  TA TODO: This command shouldnt be needed anymore, delete if so!
    //    new POVButton(operatorJoystick, 270)
    //        .debounce(0.10)
    //        .onTrue(
    //            new HandlerHarvestAlgae(s_Handler, !kHoldForever)
    //                .andThen(new ArmHomeWithAlgae(s_Elevator, s_Elbow, s_Wrist, kToleranceHold))
    //                .alongWith(new HandlerHarvestAlgae(s_Handler, kHoldForever)));
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
