// Author: UMN Robotics Ri3d
// Last Updated : January 2024

package frc.robot.subsystems;

import static edu.wpi.first.units.Units.*;
import static frc.robot.ConstantsMechanisms.ElbowConstants.*;

import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.FeedbackConfigs;
import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.SensorDirectionValue;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ElbowSubsystem extends SubsystemBase {

  // Elbow Motor Controllers
  private TalonFX m_leftElbowMotor = new TalonFX(kLeftElbowMotorCanId, "rio"); // KRAKEN motor
  //  private RelativeEncoder ElbowEncoder =  m_leftElbowMotor.getEncoder();//
  private TalonFX m_rightElbowMotor = new TalonFX(kRightElbowMotorCanId, "rio"); // KRAKEN motor

  private final MotionMagicVoltage m_mmReqLt = new MotionMagicVoltage(0);
  private final MotionMagicVoltage m_mmReqRt = new MotionMagicVoltage(0);

  private CANcoder elbowCanCoder = new CANcoder(5, "1468_CANivore");
  private static final CANcoderConfiguration elbowCanCoderInitialConfigs =
      new CANcoderConfiguration();

  double ltSpeed, rtSpeed;

  private double currentLeftPos;
  private double currentRightPos;

  /** Subsystem for controlling the Elbow */
  public ElbowSubsystem() {

    TalonFXConfiguration cfg = new TalonFXConfiguration();

    /////////////////////////////// Sample Code ////////////////////////
    /* Configure gear ratio */
    // FeedbackConfigs fdb = cfg.Feedback;
    // fdb.SensorToMechanismRatio = 12.8; // 12.8 rotor rotations per mechanism rotation - example
    // program

    /* Configure Motion Magic */
    // MotionMagicConfigs mm = cfg.MotionMagic;
    // mm.withMotionMagicCruiseVelocity(RotationsPerSecond.of(5)) // 5 (mechanism) rotations per
    // second cruise
    //    .withMotionMagicAcceleration(RotationsPerSecondPerSecond.of(10)) // Take approximately 0.5
    // seconds to reach max vel
    // Take approximately 0.1 seconds to reach max accel
    //    .withMotionMagicJerk(RotationsPerSecondPerSecond.per(Second).of(100));
    /////////////////////////////// End Sample Code ////////////////////////

    /* Configure gear ratio */
    FeedbackConfigs fdb = cfg.Feedback;
    // TA - was 45:1, but now updaed to 60:1
    // fdb.SensorToMechanismRatio = 45.0; // 45 rotor rotations per mechanism rotation
    fdb.SensorToMechanismRatio =
        75.0; // 75 rotor rotations per mechanism rotation - gear box reduction

    /* Configure Motion Magic */
    // TODO: TA - Optimize MM for Elevator
    // 1.5V, 1A worked but was slow -
    // 1V, 5A worked but too fast
    MotionMagicConfigs mm = cfg.MotionMagic;
    mm.withMotionMagicCruiseVelocity(
            RotationsPerSecond.of(1.25)) // was 1.0, TA TODO: optimize for movement and climbing
        .withMotionMagicAcceleration(
            RotationsPerSecondPerSecond.of(
                2)) // Take approximately 0.5 seconds to reach max vel (was 2.25)
        // Take approximately 0.2 seconds to reach max accel
        .withMotionMagicJerk(RotationsPerSecondPerSecond.per(Second).of(0)); // was 200
    //    .withMotionMagicJerk(RotationsPerSecondPerSecond.per(Second).of(400)); // was 200

    Slot0Configs slot0 = cfg.Slot0;
    slot0.kS = 0.25; // Add 0.25 V output to overcome static friction
    slot0.kV = 0.12; // A velocity target of 1 rps results in 0.12 V output
    slot0.kA = 0.01; // An acceleration of 1 rps/s requires 0.01 V output
    slot0.kP = 60; // A position error of 0.2 rotations results in 12 V output
    slot0.kI = 0; // No output for integrated error
    slot0.kD = 0.5; // A velocity error of 1 rps results in 0.5 V output

    cfg.withCurrentLimits(
        new CurrentLimitsConfigs()
            .withStatorCurrentLimit(Amps.of(80))
            .withStatorCurrentLimitEnable(true));

    StatusCode statusLt = StatusCode.StatusCodeNotInitialized;
    StatusCode statusRt = StatusCode.StatusCodeNotInitialized;
    for (int i = 0; i < 5; ++i) {
      statusLt = m_leftElbowMotor.getConfigurator().apply(cfg);
      statusRt = m_rightElbowMotor.getConfigurator().apply(cfg);
      if (statusLt.isOK() && statusRt.isOK()) break;
    }
    if (!statusLt.isOK()) {
      System.out.println("Could not configure Left Elbow device. Error: " + statusLt.toString());
    }
    if (!statusRt.isOK()) {
      System.out.println("Could not configure Right Elbow device. Error: " + statusRt.toString());
    }

    // m_rightElbowMotor.setControl(new Follower(m_leftElbowMotor.getDeviceID(), true));
    // m_rightElbowMotor.setControl(new Follower(kLeftElbowMotorCanId, false));

    m_leftElbowMotor.setNeutralMode(NeutralModeValue.Brake);
    m_rightElbowMotor.setNeutralMode(NeutralModeValue.Brake);

    // Configure CANCoder
    CANcoderConfiguration cancoderConfig = elbowCanCoderInitialConfigs;
    cancoderConfig.MagnetSensor.MagnetOffset = kElbowCanCoderOffset;
    cancoderConfig.MagnetSensor.SensorDirection =
        kElbowCanCoderInverted
            ? SensorDirectionValue.Clockwise_Positive
            : SensorDirectionValue.CounterClockwise_Positive;
    elbowCanCoder.getConfigurator().apply(cancoderConfig);

    // TODO: TA  - need to set starting position relative to absolute encoder

    double tempCanCoderDegrees = elbowCanCoder.getAbsolutePosition().getValueAsDouble() * 360.0;

    if (tempCanCoderDegrees > 0.0) tempCanCoderDegrees = tempCanCoderDegrees - 360.0;
    double tempElbowAngle = tempCanCoderDegrees + 34.541;
    setElbowPosition((-tempElbowAngle) / kEncoderRotation2Degrees);
  }

  /* Set  Elbow motors to Brake Mode */
  public void setBrakeMode() {
    m_leftElbowMotor.setNeutralMode(NeutralModeValue.Brake);
    m_rightElbowMotor.setNeutralMode(NeutralModeValue.Brake);
  }

  /* Set  Elbow motors to Brake Mode */
  public void setCoastMode() {
    m_leftElbowMotor.setNeutralMode(NeutralModeValue.Coast);
    m_rightElbowMotor.setNeutralMode(NeutralModeValue.Coast);
  }

  /* Set power to the Elbow motor */
  public void setElbowSpeeds(double Speed) {
    m_leftElbowMotor.set(Speed);
    m_rightElbowMotor.set(-Speed); // right should follow without command
  }

  /* Set Elbow motor Position using Magic Motion*/
  public void setElbowMagicMoPos(double pos) {
    m_leftElbowMotor.setControl(m_mmReqLt.withPosition(-pos).withSlot(0));
    m_rightElbowMotor.setControl(m_mmReqRt.withPosition(pos).withSlot(0));
  }

  /* Set Elbow motor Position*/
  public void setElbowPosition(double pos) {
    m_leftElbowMotor.setPosition(Rotations.of(-pos));
    m_rightElbowMotor.setPosition(Rotations.of(pos));
  }

  public void stop() {
    m_leftElbowMotor.set(0);
    m_rightElbowMotor.set(0);
  }

  @Override
  public void periodic() {

    currentLeftPos = m_leftElbowMotor.getPosition().getValueAsDouble();
    currentRightPos = m_rightElbowMotor.getPosition().getValueAsDouble();

    // Put the speed on SmartDashboard

    SmartDashboard.putNumber(
        "Elbow LtMtr Angle",
        m_leftElbowMotor.getPosition().getValueAsDouble() * kEncoderRotation2Degrees);
    SmartDashboard.putNumber(
        "Elbow RtMtr Angle",
        m_rightElbowMotor.getPosition().getValueAsDouble() * kEncoderRotation2Degrees);
    SmartDashboard.putNumber("Elbow LtMtr Vel", m_leftElbowMotor.getVelocity().getValueAsDouble());
    SmartDashboard.putNumber("Elbow RtMtr Vel", m_rightElbowMotor.getVelocity().getValueAsDouble());
    SmartDashboard.putNumber(
        "Elbow LtMtr Tmp", m_leftElbowMotor.getDeviceTemp().getValueAsDouble());
    SmartDashboard.putNumber(
        "Elbow RtMtr Tmp", m_rightElbowMotor.getDeviceTemp().getValueAsDouble());

    SmartDashboard.putNumber(
        "ElbowAbsEnc Offset", elbowCanCoder.getAbsolutePosition().getValueAsDouble());

    SmartDashboard.putNumber(
        "ElbowAbsEnc Angle", elbowCanCoder.getAbsolutePosition().getValueAsDouble() * 360.0);
  }

  /* Set Elbow motor Position*/
  public double getLtElbowPosition() {
    //  SmartDashboard.putNumber("Elbow LtMtr Angle in GET", currentLeftPos *
    // kEncoderRotation2Degrees);
    return currentLeftPos;
  }

  /* Set Elbow motor Position*/
  public double getRtElbowPosition() {
    return currentRightPos;
  }
}
