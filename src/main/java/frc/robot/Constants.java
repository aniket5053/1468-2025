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

import edu.wpi.first.wpilibj.RobotBase;

/**
 * This class defines the runtime mode used by AdvantageKit. The mode is always "real" when running
 * on a roboRIO. Change the value of "simMode" to switch between "sim" (physics sim) and "replay"
 * (log replay from a file).
 */
public final class Constants {

  // CANdle constants
  /*
   * This example uses:
   * - A CANdle wired on the CAN Bus, with a 5m led strip attached for the extra animatinos.
   *
   * Controls (with Xbox controller):
   * Right Bumper: Increment animation
   * Left Bumper: Decrement animation
   * Start Button: Switch to setting the first 8 LEDs a unique combination of colors
   * POV Right: Configure maximum brightness for the CANdle
   * POV Down: Configure medium brightness for the CANdle
   * POV Left: Configure brightness to 0 for the CANdle
   * POV Up: Change the direction of Rainbow and Fire, must re-select the animation to take affect
   * A: Print the VBat voltage in Volts
   * B: Print the 5V voltage in Volts
   * X: Print the current in amps
   * Y: Print the temperature in degrees C
   */

  public static final int CANdleID = 1;
  public static final int JoystickId = 0;
  // public static final int IncrementAnimButton = XboxController.Button.kRightBumper.value;
  // public static final int DecrementAnimButton = XboxController.Button.kLeftBumper.value;
  // public static final int BlockButton = XboxController.Button.kStart.value;
  public static final int MaxBrightnessAngle = 90;
  public static final int MidBrightnessAngle = 180;
  public static final int ZeroBrightnessAngle = 270;
  public static final int ChangeDirectionAngle = 0;
  // public static final int VbatButton = XboxController.Button.kA.value;
  // public static final int V5Button = XboxController.Button.kB.value;
  // public static final int CurrentButton = XboxController.Button.kX.value;
  // public static final int TemperatureButton = XboxController.Button.kY.value;

  // AdvantageKit Constants

  public static final Mode simMode = Mode.SIM;
  public static final Mode currentMode = RobotBase.isReal() ? Mode.REAL : simMode;

  public static enum Mode {
    /** Running on a real robot. */
    REAL,

    /** Running a physics simulator. */
    SIM,

    /** Replaying from a log file. */
    REPLAY
  }
}
