package frc.robot.commands;

import static frc.robot.ConstantsMechanisms.HandlerConstants.*;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.HandlerSubsystem;

public class HandlerHarvestAlgae extends Command {
  private final HandlerSubsystem handlerSubsystem;
  private int counter;

  public HandlerHarvestAlgae(HandlerSubsystem handlerSubsystem) {
    this.handlerSubsystem = handlerSubsystem;
    addRequirements(handlerSubsystem);
  }

  @Override
  public void initialize() {
    handlerSubsystem.setHandlerVoltageVelos(kHandlerAlgaeInSpeed, kHandlerAlgaeInSpeed);
  }

  @Override
  public void execute() {

    if (handlerSubsystem.getLimitSwitch()) {
      counter++;
    } else counter = 0;
  }

  @Override
  public void end(boolean interrupted) {
    // Stop the motor when the command ends
    handlerSubsystem.stop();
  }

  @Override
  public boolean isFinished() {
    // Stop the motor 50 counts after the limit switch turns true
    //    return handlerSubsystem.getLimitSwitch();
    return (counter >= 500);
  }
}
