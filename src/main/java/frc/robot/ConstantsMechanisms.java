package frc.robot;

import static edu.wpi.first.units.Units.*;

/** Automatically generated file containing build version information. */
public final class ConstantsMechanisms {

  public static final class DriveConstants {
    public static final double kCenter = 0;
    public static final double kLeftSide = 1;
    public static final double kRightSide = -1;
  }
  //  private ConstantsForHHS_Code() {}

  // using one tolerance criteria for elbow, elevator and wrist - tolerance is in rotations
  // tolerance is used by "isFinished" routines in commands
  // Hold Tolerance of 0.0 will force "isFinished" to always return "False" and therfore never
  // finish
  // Auto Tolerance > 0.0 will allow command to finish when current posistion is within tolerance of
  // commanded position
  public static final double kToleranceHold = 0.0;
  public static final double kToleranceAuto = 0.5; // TA TODO - Optimize this value

  //////////////////////////    ELBOW     /////////////////////////////////////////

  public static final class ElbowConstants {
    public static final int kLeftElbowMotorCanId = 21;
    public static final int kRightElbowMotorCanId = 22;

    public static final int kElbowCanCoderCanId = 5;
    public static final double kElbowCanCoderOffset = -0.00;
    public static final boolean kElbowCanCoderInverted = false;

    public static final double kUpSpeed = 0.15;
    public static final double kDownSpeed = -0.12;

    // Falcon / Talon FX uses rotations not ticks
    // numerator is conversion to degrees taking into account the gear ratio (45:1) and
    // sprocket ratio (6:1) - 270:1 Total
    // public static final double kEncoderRotation2Degrees = (360.0 / 6.0);

    // New Elbow Gear boxes are now 60:1 with 4:1 on the chain.
    // 60:1 update is in the ElbowSubsystem
    public static final double kEncoderRotation2Degrees = (360.0 / 4.0);

    //    using single tolerance now for elbow, elevator and wrist
    //    public static final double kToleranceDegrees = 0.5;

    // The elbow has an absolute encoder that will give us the offset
    public static final double kZeroOffset = 0.0; // TA TODO - Need to Set based on emperical data

    public static final double kStartAngle = 90.0; // TA TODO - Need to Set based on emperical data
    // public static final double kPreHomeAngle = 105.0;// TA TODO: try to eliminate need for this
    public static final double kHomeAngle = 120.0;
    public static final double kLevel1Angle = 100.0;
    public static final double kLevel2Angle = 100.0;
    public static final double kLevel3Angle = 97.0;
    public static final double kPreLevel4Angle = 100.0;
    public static final double kLevel4Angle = 88.5;
    public static final double kAlgaeHighAngle = 85.0; // 88 was too straight up
    public static final double kAlgaeLowAngle = 85.0; // 88 was too straight up
    public static final double kCoralStationAngle = 139.0; // 138 was too high, 140 was too low
    public static final double kBargeNetAngle = 100.0;
    public static final double kProcessorAngle = 135.0;
    public static final double kUnClimbAngle = 90.0;
    public static final double kPreClimbAngle1 = 135.0;
    public static final double kPreClimbAngle2 = 145;
    public static final double kClimbAngle = 175.0; // 170 works real well trying to optimize still

    public static final double kLowerSoftLimit = 82;
    public static final double kUpperSoftLimit = 180;

    public static final double kSmallMoveDegrees = 3;
    // If MM routine sees these values as input, it will make a small move (not got to that angle)
    public static final double kUpSmallDegrees = 1000.0;
    public static final double kDownSmallDegrees = -1000.0;
  }

  //////////////////////////    ELEVATOR     /////////////////////////////////////////

  public static final class ElevatorConstants {
    public static final int kLeftElevMotorCanId = 23;
    public static final int kRightElevMotorCanId = 24;

    public static final double kUpSpeed = 0.5; // now 25:1 raise speed
    public static final double kDownSpeed = -0.5;

    // Falcon / Taon FX Encoder native units is rotations. We convert to inches for the elevator
    // rotations * circumference of wheel (pi*d) * 2 stages
    // d= sprocket diameter ~= 1.67
    // THere is a factor of xxx that I dont understand?!?
    public static final double kEncoderRotation2Inches = (2.0 * Math.PI * 1.67);

    //    using single tolerance now for elbow, elevator and wrist
    //    public static final double kToleranceInches = 0.5;

    /******************************************************************************************* */
    /******************************************************************************************* */
    // must be zero - reset encoders here
    /******************************************************************************************* */
    // Elbow starts all the way down against the hard stop - this is the zero position
    // TA Done: 0.0 values are good, we dont "slam" the hard stop
    public static final double kZeroOffset = 0.0;

    public static final double kStartPos = 0.0;
    public static final double kHomePos = 0.0; // must be zero - reset encoders here
    public static final double kHomeWithAlgaePos = 0.0; // must be zero - reset encoders here
    /******************************************************************************************* */
    // must be zero - reset encoders here
    /******************************************************************************************* */
    /******************************************************************************************* */

    public static final double kCoralStationPos = 0.0;

    public static final double kLevel1Pos = 0.0;
    public static final double kLevel2Pos = 0.0;
    public static final double kLevel3Pos = 13.0; // higher since elbow is now 100, wass 90
    public static final double kPreLevel4Pos = 35.0; // was 22, too low
    public static final double kLevel4Pos = 40.0;
    public static final double kAlgaeLowPos = 4.0; // was 0 too low
    public static final double kAlgaeHighPos = 18.0; //
    public static final double kBargeNetPos =
        40.0; // was 44 - but algae shoots high slow lower some
    public static final double kProcessorPos = kHomeWithAlgaePos;
    public static final double kClimbPos = 6.0;
    public static final double kUnClimbPos = 6.0;

    public static double kReverseSoftLimit = 0;
    public static double kForwardSoftLimit = 48.0;

    public static double kSmallMoveInches = 1.5;
  }

  //////////////////////////    WRIST     /////////////////////////////////////////

  public static final class WristConstants {
    public static final int kWristMotorCanId = 25;

    public static final double kUpSpeed = 0.5; // now 25:1 raise speed
    public static final double kDownSpeed = -0.5;

    // Falcon / Talon FX uses rotations not ticks
    // numerator is conversion to degrees taking into account the gear ratio (25:1) and
    // sprocket ratio (40:12) - 83.33:1 Total
    public static final double kEncoderRotation2Degrees = (360.0 / (40.0 / 12.0));

    //    using single tolerance now for elbow, elevator and wrist
    //    public static final double kToleranceDegrees = 0.5;

    // Wrist will start against "down" hardstop
    public static final double kZeroOffset = 0.0;
    // TA TODO: Need to optimize this number
    public static final double kStartAngle = -91.0;
    public static final double kHomeAngle = -90.0;
    public static final double kHomeWithAlgaeAngle = -45.0;
    // Arm is too tall for Level 1 & 2, so have to tilt at a high angle to score there
    public static final double kLevel1Angle = -45.0; // have to shoot out over climber
    public static final double kLevel2Angle = -50.0; // -45 was too high
    public static final double kLevel3Angle = -42.0;
    public static final double kLevel4Angle = -75.0; //
    public static final double kAlgaeLowAngle = -45.0; // elevator at 0, so angle cant be higher
    public static final double kAlgaeHighAngle = -45.0; // was -35 - too low
    public static final double kCoralStationAngle = -90.0; // working this angle -65 was too
    public static final double kBargeNetAngle = +89.0;
    public static final double kProcessorAngle = -35.0; // was -30
    public static final double kClimbAngle = +0.0;
    public static final double kUnClimbAngle = +0.0;

    public static double kReverseSoftLimit = -90.0;
    public static double kForwardSoftLimit = 90.0;

    public static double kSmallMoveInches = 1.5;
  }

  //////////////////////////    HANDLER     /////////////////////////////////////////

  public static final class HandlerConstants {
    public static final int kHandlerLeftMotorCanId = 26;
    public static final int kHandlerRightMotorCanId = 27;

    public static final int kHandlerLimitSwitchId = 2;

    public static final boolean kHandlerInvert = false;

    public static final double kHandlerCoralInSpeed =
        0.10; // was .15, but too fast - coral went too far
    public static final double kHandlerCoralOutSpeed =
        0.10; // was .15, but too fast - coral bounced back
    public static final double kHandlerAlgaeInSpeed = -.15;
    public static final double kHandlerAlgaeOutSpeed = 0.75;
    public static final double kDesiredRotationsPerSecond = 100.0;

    public static final boolean kHoldForever = true;
  }
}
