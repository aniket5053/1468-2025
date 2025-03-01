package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.*;
import frc.robot.subsystems.drive.Drive;

public class AllMotorsCoast extends Command {
  private final ElbowSubsystem elbowSubsystem;
  private final ElevatorSubsystem elevatorSubsystem;
  private final WristSubsystem wristSubsystem;
  private final Drive drive;

  public AllMotorsCoast(
      ElbowSubsystem elbowSubsystem,
      ElevatorSubsystem elevatorSubsystem,
      WristSubsystem wristSubsystem,
      Drive drive) {
    this.elbowSubsystem = elbowSubsystem;
    this.elevatorSubsystem = elevatorSubsystem;
    this.wristSubsystem = wristSubsystem;
    this.drive = drive;

    addRequirements(elbowSubsystem, elevatorSubsystem, wristSubsystem, drive);
  }

  @Override
  public void initialize() {

    elbowSubsystem.setCoastMode();
    elevatorSubsystem.setCoastMode();
    wristSubsystem.setCoastMode();
    drive.setCoast();
  }

  @Override
  public void execute() {}

  @Override
  public void end(boolean interrupted) {}
}
