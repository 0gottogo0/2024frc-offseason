// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Elevator extends SubsystemBase {
  private TalonFX elevator = new TalonFX(Constants.elevatorID);

  private TalonFXConfiguration cfg = new TalonFXConfiguration();

  /** Creates a new Elevator. */
  public Elevator() {
    elevator.getConfigurator().apply(cfg);
    elevator.setInverted(true);
    elevator.setNeutralMode(NeutralModeValue.Brake);
    elevator.clearStickyFaults();
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  public void elevatorOut(double speedPercent) {
    elevator.set(speedPercent);
  }

  public void elevatorIn(double speedPercent) {
    elevator.set(-speedPercent);
  }

  public void elevatorStop() {
    elevator.set(0.00);
  }
}
