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
    public static final double kEncoderRotation2Degrees = (360.0 / 6.0);
    public static final double kToleranceDegrees = 0.5;

    // The elbow has an absolute encoder that will give us the offset
    public static final double kZeroOffset = 0.0; // TA TODO - Need to Set based on emperical data
    public static final double kStartAngle = 90.0; // TA TODO - Need to Set based on emperical data
    public static final double kPreHomeAngle = 110.0;
    public static final double kHomeAngle = 130.0;
    public static final double kLevel1Angle = 90.0;
    public static final double kLevel2Angle = 100.0;
    public static final double kLevel3Angle = 90.0;
    public static final double kPreLevel4Angle = 105.0;
    public static final double kLevel4Angle = 90.2;
    public static final double kAlgaeAngle = 88.0;
    public static final double kCoralStationAngle = 135.0;
    public static final double kBargeNetAngle = 105.0;
    public static final double kProcessorAngle = 135.0;
    public static final double kClimbAngle = 175.0;
    public static final double kUnClimbAngle = 120.0;

    public static double kReverseSoftLimit = 30;
    public static double kForwardSoftLimit = 97;

    public static double kSmallMoveDegrees = 3;
    // public static final double kMaxAngle = (kForwardSoftLimit - 2);
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
    public static final double kToleranceInches = 0.5;
    // Elbow starts all the way down against the hard stop - this is the zero position
    // TA TODO: 0.0 values may have to increase a fraction so we dont "slam" the hard stop
    public static final double kZeroOffset = 0.0;
    public static final double kStartPos = 0.0;
    public static final double kHomePos = 0.0; // was 5
    public static final double kLevel1Pos = 0.0;
    public static final double kLevel2Pos = 0.0;
    public static final double kLevel3Pos = 10.0;
    public static final double kPreLevel4Pos = 33.0; // was 22, too low
    public static final double kLevel4Pos = 38.0;
    public static final double kAlgaeLowPos = 2.0;
    public static final double kAlgaeHighPos = 18.0;
    public static final double kCoralStationPos = 0.0;
    public static final double kBargeNetPos = 44.0;
    public static final double kProcessorPos = 0.0;
    public static final double kClimbPos = 6.0;
    public static final double kUnClimbPos = 6.0;

    public static double kReverseSoftLimit = 0;
    public static double kForwardSoftLimit = 51.0;

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
    public static final double kToleranceDegrees = 0.5;
    // Wrist will start against "down" hardstop
    public static final double kZeroOffset = 0.0;
    // TA TODO: Need to optimize this number
    public static final double kStartAngle = -91.0;
    public static final double kHomeAngle = -80.0;
    // Arm is too tall for Level 1 & 2, so have to tilt at a high angle to score there
    public static final double kLevel1Angle = -65.0;
    public static final double kLevel2Angle = -45.0;
    public static final double kLevel3Angle = -35.0;
    public static final double kLevel4Angle = -60.0;
    public static final double kAlgaeAngle = -35.0;
    public static final double kCoralStationAngle = -65.0;
    public static final double kBargeNetAngle = +65.0;
    public static final double kProcessorAngle = -45.0;
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

    public static final boolean kHandlerInvert = false;

    public static final double kHandlerCoralInSpeed = 0.5;
    public static final double kHandlerCoralOutSpeed = 0.75;
    public static final double kHandlerAlgaeInSpeed = -0.5;
    public static final double kHandlerAlgaeOutSpeed = 0.95;
    public static final double kDesiredRotationsPerSecond = 50.0;

    public static final int kHandlerLimitSwitchId = 2;
  }
}
