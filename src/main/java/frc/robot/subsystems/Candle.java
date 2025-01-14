// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.Random;

import com.ctre.phoenix.led.CANdle;
import com.ctre.phoenix.led.CANdle.LEDStripType;
import com.ctre.phoenix.led.CANdleConfiguration;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Candle extends SubsystemBase {

  private CANdle candle = new CANdle(Constants.candleID);
  private Random random = new Random();

  private CANdleConfiguration cfg = new CANdleConfiguration();

  /** Creates a new Lights. */
  public Candle() {
    cfg.statusLedOffWhenActive = true;
    cfg.stripType = LEDStripType.GRB;
    cfg.disableWhenLOS = false;
    
    candle.configFactoryDefault();
    candle.clearStickyFaults();
    candle.configAllSettings(cfg);
  }

  @Override
  public void periodic() {
  }

  public void setBlue() {
    candle.setLEDs(0, 0, 255);
    
  }

  public void setRed() {
    candle.setLEDs(255, 0, 0);
  }
}