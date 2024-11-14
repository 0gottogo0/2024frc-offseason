// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj.motorcontrol.Talon;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Intake extends SubsystemBase {

  private TalonFX intakeTop = new TalonFX(Constants.intakeTopID);
  //private TalonSRX intakeBottom = new TalonSRX(Constants.intakeBottomID);

  private TalonFXConfiguration cfg = new TalonFXConfiguration();

  /** Creates a new Intake. */
  public Intake() {

    cfg.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    cfg.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;

    intakeTop.clearStickyFaults();
    intakeTop.getConfigurator().apply(cfg);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
