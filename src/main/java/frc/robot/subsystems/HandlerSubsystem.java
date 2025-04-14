// Author: UMN Robotics Ri3d
// Last Updated : January 2024

package frc.robot.subsystems;

import static edu.wpi.first.units.Units.*;
import static frc.robot.ConstantsMechanisms.HandlerConstants.*;

import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.VelocityTorqueCurrentFOC;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class HandlerSubsystem extends SubsystemBase {

  // HANDLER Motor Controllers
  private TalonFX m_leftHandlerMotor = new TalonFX(kHandlerLeftMotorCanId, "rio"); // KRAKEN motor
  //  private RelativeEncoder handlerEncoder =  m_leftHandlerMotor.getEncoder();//
  private TalonFX m_rightHandlerMotor = new TalonFX(kHandlerRightMotorCanId, "rio"); // KRAKEN motor

  private DigitalInput m_limitSwitch = new DigitalInput(1);

  /* Start at velocity 0, use slot 0 */
  private final VelocityVoltage m_velocityVoltage = new VelocityVoltage(0).withSlot(0);
  /* Start at velocity 0, use slot 1 */
  private final VelocityTorqueCurrentFOC m_velocityTorque =
      new VelocityTorqueCurrentFOC(0).withSlot(1);
  /* Keep a neutral out so we can disable the motor */
  // private final NeutralOut m_brake = new NeutralOut();

  //  private final Mechanisms m_mechanisms = new Mechanisms();
  double ltSpeed, rtSpeed;

  /** Subsystem for controlling the HANDLER */
  public HandlerSubsystem() {

    TalonFXConfiguration configs = new TalonFXConfiguration();

    /* Voltage-based velocity requires a velocity feed forward to account for the back-emf of the motor */
    configs.Slot0.kS = 0.1; // To account for friction, add 0.1 V of static feedforward
    configs.Slot0.kV =
        0.12; // Kraken X60 is a 500 kV motor, 500 rpm per V = 8.333 rps per V, 1/8.33 = 0.12 volts
    // / rotation per second
    configs.Slot0.kP = 0.11; // An error of 1 rotation per second results in 0.11 V output
    configs.Slot0.kI = 0; // No output for integrated error
    configs.Slot0.kD = 0; // No output for error derivative
    // Peak output of 8 volts
    configs.Voltage.withPeakForwardVoltage(Volts.of(8)).withPeakReverseVoltage(Volts.of(-8));

    // /* Torque-based velocity does not require a velocity feed forward, as torque will accelerate
    // the rotor up to the desired velocity by itself */
    // configs.Slot1.kS = 2.5; // To account for friction, add 2.5 A of static feedforward
    // configs.Slot1.kP = 5; // An error of 1 rotation per second results in 5 A output
    // configs.Slot1.kI = 0; // No output for integrated error
    // configs.Slot1.kD = 0; // No output for error derivative
    // Peak output of 40 A
    configs.TorqueCurrent.withPeakForwardTorqueCurrent(Amps.of(40))
        .withPeakReverseTorqueCurrent(Amps.of(-40));

    configs.withCurrentLimits(
        new CurrentLimitsConfigs()
            .withStatorCurrentLimit(Amps.of(80))
            .withStatorCurrentLimitEnable(true));

    StatusCode statusLt = StatusCode.StatusCodeNotInitialized;
    StatusCode statusRt = StatusCode.StatusCodeNotInitialized;
    for (int i = 0; i < 5; ++i) {
      statusLt = m_leftHandlerMotor.getConfigurator().apply(configs);
      statusRt = m_rightHandlerMotor.getConfigurator().apply(configs);
      if (statusLt.isOK() && statusRt.isOK()) break;
    }
    if (!statusLt.isOK()) {
      System.out.println("Could not configure Left Handler device. Error: " + statusLt.toString());
    }
    if (!statusRt.isOK()) {
      System.out.println("Could not configure Right Handler device. Error: " + statusRt.toString());
    }
    // TA TODO: Having issues with follower motor, not sure why??????
    //   m_rightHandlerMotor.setControl(new Follower(m_leftHandlerMotor.getDeviceID(), true));

    m_leftHandlerMotor.setNeutralMode(NeutralModeValue.Brake);
    m_rightHandlerMotor.setNeutralMode(NeutralModeValue.Brake);
  }

  /* Set  HANDLER motors to Brake Mode */
  public void setBrakeMode() {
    m_leftHandlerMotor.setNeutralMode(NeutralModeValue.Brake);
    m_rightHandlerMotor.setNeutralMode(NeutralModeValue.Brake);
  }

  /* Set  HANDLER motors to Brake Mode */
  public void setCoastMode() {
    m_leftHandlerMotor.setNeutralMode(NeutralModeValue.Coast);
    m_rightHandlerMotor.setNeutralMode(NeutralModeValue.Coast);
  }

  /* Set power to the HANDLER motor */
  public void setHandlerVoltageVelos(double ltSpeed, double rtSpeed) {

    m_leftHandlerMotor.setControl(
        m_velocityVoltage.withVelocity(ltSpeed * kDesiredRotationsPerSecond));
    m_rightHandlerMotor.setControl(
        m_velocityVoltage.withVelocity(-rtSpeed * kDesiredRotationsPerSecond));
  }

  public void stop() {
    m_leftHandlerMotor.set(0);
    m_rightHandlerMotor.set(0);
  }

  public boolean getLimitSwitch() {
    return m_limitSwitch.get();
  }

  @Override
  public void periodic() {

    // Put the speed on SmartDashboard

    SmartDashboard.putNumber("HANDLER LtMtr Spd", m_leftHandlerMotor.get());
    SmartDashboard.putNumber("HANDLER RtMtr Spd", m_rightHandlerMotor.get());
    SmartDashboard.putNumber(
        "HANDLER LtMtr Vel", m_leftHandlerMotor.getVelocity().getValueAsDouble());
    SmartDashboard.putNumber(
        "HANDLER RtMtr Vel", m_rightHandlerMotor.getVelocity().getValueAsDouble());
    SmartDashboard.putNumber(
        "HANDLER LtMtr Tmp", m_leftHandlerMotor.getDeviceTemp().getValueAsDouble());
    SmartDashboard.putNumber(
        "HANDLER RtMtr Tmp", m_rightHandlerMotor.getDeviceTemp().getValueAsDouble());
    SmartDashboard.putBoolean("LimitSwitch", getLimitSwitch());
  }
}
