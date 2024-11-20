// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.TalonSRXControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Feed extends SubsystemBase {

  private TalonSRX feed = new TalonSRX(Constants.feedID);

  /** Creates a new Feed. */
  public Feed() {

    feed.configFactoryDefault();
    feed.setInverted(false);
    feed.clearStickyFaults();
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  public void feed() {
    feed.set(TalonSRXControlMode.PercentOutput, 0.5);
  }

  public void feedStop() {
    feed.set(TalonSRXControlMode.PercentOutput, 0.0);
  }
}
