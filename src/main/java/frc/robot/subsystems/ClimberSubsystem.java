// Author: UMN Robotics Ri3d
// Last Updated : January 2024

package frc.robot.subsystems;

import static edu.wpi.first.units.Units.*;
import static frc.robot.ConstantsMechanisms.ClimberConstants.*;

import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ClimberSubsystem extends SubsystemBase {

  // Climber Motor Controllers
  private TalonFX m_climberMotor = new TalonFX(kClimberMotorCanId, "rio"); // KRAKEN motor

  // decide if Limit Sw needed or use the CageLimitSwitch in robot.java
  // private DigitalInput m_limitSwitch = new DigitalInput(5);

  /* Start at velocity 0, use slot 0 */
  private final VelocityVoltage m_velocityVoltage = new VelocityVoltage(0).withSlot(0);

  double climberSpeed;

  /** Subsystem for controlling the HANDLER */
  public ClimberSubsystem() {

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

    configs.withCurrentLimits(
        new CurrentLimitsConfigs()
            .withStatorCurrentLimit(Amps.of(80))
            .withStatorCurrentLimitEnable(true));

    StatusCode status = StatusCode.StatusCodeNotInitialized;

    for (int i = 0; i < 5; ++i) {
      status = m_climberMotor.getConfigurator().apply(configs);

      if (status.isOK()) break;
    }
    if (!status.isOK()) {
      System.out.println("Could not configure Left Handler device. Error: " + status.toString());
    }

    m_climberMotor.setNeutralMode(NeutralModeValue.Brake);
  }

  /* Set  motors to Brake Mode */
  public void setBrakeMode() {
    m_climberMotor.setNeutralMode(NeutralModeValue.Brake);
  }

  /* Set  motors to Coast Mode */
  public void setCoastMode() {
    m_climberMotor.setNeutralMode(NeutralModeValue.Coast);
  }

  /* Set  motors to Coast Mode */
  public double getClimberCurrent() {
    return m_climberMotor.getStatorCurrent().getValueAsDouble();
  }

  /* Set power to the Climber motor */
  public void setClimberVoltageVelos(double climberSpeed) {

    m_climberMotor.setControl(
        m_velocityVoltage.withVelocity(climberSpeed * kDesiredRotationsPerSecond));
  }

  public void stop() {
    m_climberMotor.set(0);
  }

  // public boolean getLimitSwitch() {
  //   return m_limitSwitch.get();
  // }

  @Override
  public void periodic() {

    double climberCurrent = getClimberCurrent();
    boolean currentHigh = false;
    if (climberCurrent > 70.0) currentHigh = true;
    SmartDashboard.putBoolean("CageCaptured", currentHigh);

    // Put the speed on SmartDashboard

    SmartDashboard.putNumber("CLIMBER Mtr Spd", m_climberMotor.get());
    SmartDashboard.putNumber("CLIMBER Mtr Vel", m_climberMotor.getVelocity().getValueAsDouble());
    SmartDashboard.putNumber("CLIMBER Mtr Tmp", m_climberMotor.getDeviceTemp().getValueAsDouble());
    // SmartDashboard.putBoolean("LimitSwitch", getLimitSwitch());
  }
}
