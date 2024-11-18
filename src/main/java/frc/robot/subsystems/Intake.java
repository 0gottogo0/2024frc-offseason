// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Intake extends SubsystemBase {
  private TalonFX intakeTop = new TalonFX(Constants.intakeTopID);
  private TalonSRX intakeBottom = new TalonSRX(Constants.intakeBottomID);

 

  //private CANSparkMax intakeTopBack = new CANSparkMax(Constants.intakeTopBackID, MotorType.kBrushless);

  /*private DoubleSolenoid backIntakeArms = new DoubleSolenoid(PneumaticsModuleType.REVPH, 
    Constants.backIntakeArmsForwardID, Constants.backIntakeArmsBackwardID);*/

    private TalonFXConfiguration cfg = new TalonFXConfiguration();

  /** Creates a new Intake. */
  public Intake() {

    cfg.MotorOutput.NeutralMode = NeutralModeValue.Coast;
    
    intakeTop.getConfigurator().apply(cfg);
    intakeBottom.configFactoryDefault();

    intakeTop.setInverted(true);
    intakeBottom.setInverted(true);

    intakeTop.setNeutralMode(NeutralModeValue.Coast);

    intakeTop.clearStickyFaults();
    intakeBottom.clearStickyFaults();
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    
  }

  public void intakeBack() {
    intakeTop.set(0.6);
    intakeBottom.set(ControlMode.PercentOutput, 0.6);
  }

  public void intakeFront() {
    intakeTop.set(0.6);
    intakeBottom.set(ControlMode.PercentOutput, -0.6);
  }

  public void intakeStop() {
    intakeTop.set(0);
    intakeBottom.set(ControlMode.PercentOutput, 0);
  }
}