// Author: UMN Robotics Ri3d
// Last Updated : January 2024

package frc.robot.subsystems;

import static edu.wpi.first.units.Units.*;
import static frc.robot.ConstantsMechanisms.WristConstants.*;

import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.FeedbackConfigs;
import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class WristSubsystem extends SubsystemBase {

  // Wrist Motor Controllers
  private TalonFX m_wristMotor = new TalonFX(kWristMotorCanId, "rio"); // KRAKEN motor

  private final MotionMagicVoltage m_mmReqLt = new MotionMagicVoltage(0);
  // private final MotionMagicVoltage m_mmReqRt = new MotionMagicVoltage(0);

  /* Keep a neutral out so we can disable the motor */
  // private final NeutralOut m_brake = new NeutralOut();

  //  private final Mechanisms m_mechanisms = new Mechanisms();
  double ltSpeed, rtSpeed;

  /** Subsystem for controlling the Wrist */
  public WristSubsystem() {

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
    fdb.SensorToMechanismRatio = 25.0; // 25 rotor rotations per mechanism rotation
    setWristPosition(kStartAngle / kEncoderRotation2Degrees);

    /* Configure Motion Magic */
    MotionMagicConfigs mm = cfg.MotionMagic;
    mm.withMotionMagicCruiseVelocity(
            RotationsPerSecond.of(4)) // 2 (mechanism) rotations per second cruise
        .withMotionMagicAcceleration(
            RotationsPerSecondPerSecond.of(20)) // Take approximately 0.2 seconds to reach max vel
        // Take approximately 0.2 seconds to reach max accel
        .withMotionMagicJerk(RotationsPerSecondPerSecond.per(Second).of(200));

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

    StatusCode status = StatusCode.StatusCodeNotInitialized;
    for (int i = 0; i < 5; ++i) {
      status = m_wristMotor.getConfigurator().apply(cfg);
      if (status.isOK()) break;
    }
    if (!status.isOK()) {
      System.out.println("Could not configure Left Wrist device. Error: " + status.toString());
    }

    m_wristMotor.setNeutralMode(NeutralModeValue.Brake);
  }

  /* Set  Wrist motors to Brake Mode */
  public void setBrakeMode() {
    m_wristMotor.setNeutralMode(NeutralModeValue.Brake);
  }

  /* Set  Wrist motors to Brake Mode */
  public void setCoastMode() {
    m_wristMotor.setNeutralMode(NeutralModeValue.Coast);
  }

  /* Set power to the Wrist motor */
  public void setWristSpeeds(double ltSpeed, double rtSpeed) {
    m_wristMotor.set(ltSpeed);
    //   m_rightWristMotor.set(rtSpeed);   // right should follow without command
  }

  /* Set Wrist motor Position using Magic Motion*/
  public void setWristMagicMoPos(double pos) {
    m_wristMotor.setControl(
        m_mmReqLt.withPosition(pos).withSlot(0).withOverrideBrakeDurNeutral(true));
  }

  /* Set Wrist motor Position*/
  public void setWristPosition(double pos) {
    m_wristMotor.setPosition(Rotations.of(pos));
  }

  /* Set Wrist motor Position*/
  public double getWristPosition() {
    return m_wristMotor.getPosition().getValueAsDouble();
  }

  public void stop() {
    m_wristMotor.set(0);
  }

  @Override
  public void periodic() {

    // Put the speed on SmartDashboard

    SmartDashboard.putNumber(
        "Wrist LtMtr Angle",
        m_wristMotor.getPosition().getValueAsDouble() * kEncoderRotation2Degrees);
    SmartDashboard.putNumber("Wrist LtMtr Vel", m_wristMotor.getVelocity().getValueAsDouble());
    SmartDashboard.putNumber("Wrist LtMtr Tmp", m_wristMotor.getDeviceTemp().getValueAsDouble());
  }
}
