// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Aim extends SubsystemBase {

  private TalonFX aim = new TalonFX(Constants.AimID);
  private DutyCycleEncoder aimEncoder = new DutyCycleEncoder(Constants.revEncoderDIOPort); 
  private PIDController aimController = new PIDController(0.4, 0.00, 0.00);

  private TalonFXConfiguration cfg = new TalonFXConfiguration();

  private double aimSetpoint = getAimAngleDeg();

  /** Creates a new Aim. */
  public Aim() {

    cfg.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    cfg.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;

    aim.clearStickyFaults();
    aim.getConfigurator().apply(cfg);

    aimController.setTolerance(.5);

    new Thread(() -> {
      try {
        Thread.sleep(3000);
        aimSetpoint = getAimAngleDeg();
      } catch (Exception e) {
      }
    }).start();
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run

    double pid = aimController.calculate(getAimAngleDeg(), aimSetpoint);
    pid = MathUtil.clamp(MathUtil.applyDeadband(pid, 0.05), -1, 1);
    aim.set(-pid);
  }

  public void aimUp(double speedPercent) {
    aimSetpoint = aimSetpoint - (speedPercent * 0.15);
  }

  public void aimDown(double speedPercent) {
    aimSetpoint = aimSetpoint + (speedPercent * 0.15);
  }

  public void aimStop() {
    aimSetpoint = getAimAngleDeg();
  }

  public double getAimAngleDeg() {
    return Units.rotationsToDegrees(-aimEncoder.getAbsolutePosition())+308; // Set 0 to horozon
  }

  // Fine tune setpoints
  public void setAimUnderStage() {
    aimSetpoint = 40;
  }

  public void setAimAtSpeaker() {
    aimSetpoint = 60;
  }
}