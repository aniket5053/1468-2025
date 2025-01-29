package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.subsystems.CANdleSubsystem;

public class CANdleConfigCommands {
  public static class ConfigBrightness extends InstantCommand {
    public ConfigBrightness(CANdleSubsystem CANdleSubsystem, double brightnessPercent) {
      super(() -> CANdleSubsystem.configBrightness(brightnessPercent), CANdleSubsystem);
    }

    @Override
    public boolean runsWhenDisabled() {
      return true;
    }
  }

  public static class ConfigLosBehavior extends InstantCommand {
    public ConfigLosBehavior(CANdleSubsystem CANdleSubsystem, boolean disableWhenLos) {
      super(() -> CANdleSubsystem.configLos(disableWhenLos), CANdleSubsystem);
    }

    @Override
    public boolean runsWhenDisabled() {
      return true;
    }
  }

  public static class ConfigStatusLedBehavior extends InstantCommand {
    public ConfigStatusLedBehavior(CANdleSubsystem CANdleSubsystem, boolean disableWhile) {
      super(() -> CANdleSubsystem.configStatusLedBehavior(disableWhile), CANdleSubsystem);
    }

    @Override
    public boolean runsWhenDisabled() {
      return true;
    }
  }
}
