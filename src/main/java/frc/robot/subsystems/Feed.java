// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.TalonSRXControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Feed extends SubsystemBase {

  private DigitalInput beamBreak = new DigitalInput(0);

  private TalonSRX feed = new TalonSRX(Constants.feedID);

  /** Creates a new Feed. */
  public Feed() {
    feed.configFactoryDefault();
    feed.setInverted(false);
    feed.setNeutralMode(NeutralMode.Brake);
    feed.clearStickyFaults();
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  public void feedBreak() {
    if (!beamBreak.get()) {
      feed.set(TalonSRXControlMode.PercentOutput, 0.0);
    } else {
      feed.set(TalonSRXControlMode.PercentOutput, 0.8);
    }
  }

  public void feed() {
    feed.set(TalonSRXControlMode.PercentOutput, 0.8);
  }

  public void feedStop() {
    feed.set(TalonSRXControlMode.PercentOutput, 0.0);
  }

  public Command runOnce(Runnable runnable, Object object) {
    // TODO Auto-generated method stub
    throw new UnsupportedOperationException("Unimplemented method 'runOnce'");
  }
}
