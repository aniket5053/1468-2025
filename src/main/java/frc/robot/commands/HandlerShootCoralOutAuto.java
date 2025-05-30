package frc.robot.commands;

import static frc.robot.ConstantsMechanisms.HandlerConstants.*;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.HandlerSubsystem;

public class HandlerShootCoralOutAuto extends Command {
  private final HandlerSubsystem handlerSubsystem;

  public HandlerShootCoralOutAuto(HandlerSubsystem handlerSubsystem) {
    this.handlerSubsystem = handlerSubsystem;
    addRequirements(handlerSubsystem);
  }

  @Override
  public void initialize() {
    handlerSubsystem.setHandlerVoltageVelos(kHandlerCoralOutSpeedAuto, kHandlerCoralOutSpeedAuto);
  }

  @Override
  public void execute() {}

  @Override
  public void end(boolean interrupted) {
    // Stop the motor when the command ends
    handlerSubsystem.stop();
  }

  @Override
  public boolean isFinished() {
    // Stop the motor when the limit switch turns false
    return !handlerSubsystem.getLimitSwitch();
  }
}
