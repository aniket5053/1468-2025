// Author: UMN Robotics Ri3d
// Last Updated : January 2024

package frc.robot.subsystems;

import static edu.wpi.first.units.Units.*;
import static frc.robot.ConstantsMechanisms.ElevatorConstants.*;

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

public class ElevatorSubsystem extends SubsystemBase {

  // Elevator Motor Controllers
  private TalonFX m_leftElevatorMotor = new TalonFX(kLeftElevMotorCanId, "rio"); // KRAKEN motor
  private TalonFX m_rightElevatorMotor = new TalonFX(kRightElevMotorCanId, "rio"); // KRAKEN motor

  private final MotionMagicVoltage m_mmReqLt = new MotionMagicVoltage(0);
  private final MotionMagicVoltage m_mmReqRt = new MotionMagicVoltage(0);
  /* Keep a neutral out so we can disable the motor */
  // private final NeutralOut m_brake = new NeutralOut();

  //  private final Mechanisms m_mechanisms = new Mechanisms();
  double ltSpeed, rtSpeed;

  /** Subsystem for controlling the Elevator */
  public ElevatorSubsystem() {

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
    fdb.SensorToMechanismRatio = 9.0; // 9 rotor rotations per mechanism rotation
    setElevatorPosition(kStartPos / kEncoderRotation2Inches);

    /* Configure Motion Magic */
    // TODO: TA - Optimize MM for Elevator
    // was 3V,10A smooth but too slow
    // 4V, 20A smooth but too fast
    MotionMagicConfigs mm = cfg.MotionMagic;
    mm.withMotionMagicCruiseVelocity(
            RotationsPerSecond.of(5)) // (mechanism) rotations per second cruise was 3.5
        .withMotionMagicAcceleration(
            RotationsPerSecondPerSecond.of(20)) // Take approximately 0.2 seconds to reach max vel
        // Take approximately 0.2 seconds to reach max accel
        .withMotionMagicJerk(RotationsPerSecondPerSecond.per(Second).of(0));
    //   .withMotionMagicJerk(RotationsPerSecondPerSecond.per(Second).of(200));

    Slot0Configs slot0 = cfg.Slot0;
    slot0.kG = 0.05; // fight the gravity!
    slot0.kS = 0.25; // Add 0.25 V output to overcome static friction
    slot0.kV = 0.12; // A velocity target of 1 rps results in 0.12 V output
    slot0.kA = 0.01; // An acceleration of 1 rps/s requires 0.01 V output
    slot0.kP = 60; // A position error of 0.2 rotations results in 12 V output
    slot0.kI = 0; // No output for integrated error
    slot0.kD = 0.5; // A velocity error of 1 rps results in 0.5 V output

    cfg.withCurrentLimits(
        new CurrentLimitsConfigs()
            .withStatorCurrentLimit(Amps.of(60))
            .withStatorCurrentLimitEnable(true));

    StatusCode statusLt = StatusCode.StatusCodeNotInitialized;
    StatusCode statusRt = StatusCode.StatusCodeNotInitialized;
    for (int i = 0; i < 5; ++i) {
      statusLt = m_leftElevatorMotor.getConfigurator().apply(cfg);
      statusRt = m_rightElevatorMotor.getConfigurator().apply(cfg);
      if (statusLt.isOK() && statusRt.isOK()) break;
    }
    if (!statusLt.isOK()) {
      System.out.println("Could not configure Left Elevator device. Error: " + statusLt.toString());
    }
    if (!statusRt.isOK()) {
      System.out.println(
          "Could not configure Right Elevator device. Error: " + statusRt.toString());
    }
    // Follower doesnt work for TalonFX
    // m_rightElevatorMotor.setControl(new Follower(m_leftElevatorMotor.getDeviceID(), true));

    m_leftElevatorMotor.setNeutralMode(NeutralModeValue.Brake);
    m_rightElevatorMotor.setNeutralMode(NeutralModeValue.Brake);
  }

  /* Set  Elevator motors to Brake Mode */
  public void setBrakeMode() {
    m_leftElevatorMotor.setNeutralMode(NeutralModeValue.Brake);
    m_rightElevatorMotor.setNeutralMode(NeutralModeValue.Brake);
  }

  /* Set  Elevator motors to Brake Mode */
  public void setCoastMode() {
    m_leftElevatorMotor.setNeutralMode(NeutralModeValue.Coast);
    m_rightElevatorMotor.setNeutralMode(NeutralModeValue.Coast);
  }

  /* Set power to the Elevator motor */
  public void setElevatorSpeeds(double speed) {
    m_leftElevatorMotor.set(speed);
    m_rightElevatorMotor.set(-speed); // right should follow without command
  }

  /* Set Elevator motor Position using Magic Motion*/
  public void setElevatorMagicMoPos(double pos) {
    m_leftElevatorMotor.setControl(m_mmReqLt.withPosition(-pos).withSlot(0));
    m_rightElevatorMotor.setControl(m_mmReqRt.withPosition(pos).withSlot(0));
  }

  /* Set Elevator motor Position*/
  public void setElevatorPosition(double pos) {
    m_leftElevatorMotor.setPosition(Rotations.of(pos));
    m_rightElevatorMotor.setPosition(Rotations.of(-pos));
  }

  /* Set Elevator motor Position*/
  public double getElevatorPosition() {
    return m_leftElevatorMotor.getPosition().getValueAsDouble();
  }

  public void stop() {
    m_leftElevatorMotor.set(0);
    m_rightElevatorMotor.set(0);
  }

  @Override
  public void periodic() {

    // Put the speed on SmartDashboard

    SmartDashboard.putNumber(
        "Elevator LtMtr Pos",
        m_leftElevatorMotor.getPosition().getValueAsDouble() * kEncoderRotation2Inches);
    SmartDashboard.putNumber(
        "Elevator RtMtr Pos",
        m_rightElevatorMotor.getPosition().getValueAsDouble() * kEncoderRotation2Inches);
    SmartDashboard.putNumber(
        "Elevator LtMtr Vel", m_leftElevatorMotor.getVelocity().getValueAsDouble());
    SmartDashboard.putNumber(
        "Elevator RtMtr Vel", m_rightElevatorMotor.getVelocity().getValueAsDouble());
    SmartDashboard.putNumber(
        "Elevator LtMtr Tmp", m_leftElevatorMotor.getDeviceTemp().getValueAsDouble());
    SmartDashboard.putNumber(
        "Elevator RtMtr Tmp", m_rightElevatorMotor.getDeviceTemp().getValueAsDouble());
  }
}
