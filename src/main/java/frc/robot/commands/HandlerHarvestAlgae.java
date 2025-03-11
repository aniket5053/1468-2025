package frc.robot.commands;

import static frc.robot.ConstantsMechanisms.HandlerConstants.*;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.HandlerSubsystem;

public class HandlerHarvestAlgae extends Command {
  private final HandlerSubsystem handlerSubsystem;
  private int counter;
  private boolean holdForever;

  public HandlerHarvestAlgae(HandlerSubsystem handlerSubsystem, boolean holdForever) {
    this.handlerSubsystem = handlerSubsystem;
    this.holdForever = holdForever;
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
    // Use holdForever in TeleOp so algae is never let go of, a shoot algae command will override
    // the hold
    // For Auto we can holdForever or Stop the motor 5 counts after the limit switch turns true
    // *** If holdForever is used in autonomous will need to "race" with another command to end
    if (holdForever) return false;
    else return (counter >= 5);
  }
}
