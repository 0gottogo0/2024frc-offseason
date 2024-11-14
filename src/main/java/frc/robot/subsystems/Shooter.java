// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Shooter extends SubsystemBase {

  private TalonFX shooterTop = new TalonFX(Constants.shooterTopID);
  private TalonFX shooterBottom = new TalonFX(Constants.shooterBottomID);

  private TalonFXConfiguration cfgT = new TalonFXConfiguration();
  private TalonFXConfiguration cfgB = new TalonFXConfiguration();

  /** Creates a new Shooter. */
  public Shooter() {

    cfgT.MotorOutput.NeutralMode = NeutralModeValue.Coast;
    cfgT.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;

    cfgB.MotorOutput.NeutralMode = NeutralModeValue.Coast;
    cfgB.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;

    shooterTop.clearStickyFaults();
    shooterTop.getConfigurator().apply(cfgT);

    shooterBottom.clearStickyFaults();
    shooterBottom.getConfigurator().apply(cfgB);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  public void shoot(double speedPercent) {
    shooterTop.set(speedPercent);
    shooterBottom.set(speedPercent);
  }

  public void shootStop() {
    shooterTop.set(0);
    shooterBottom.set(0);
  }
}
