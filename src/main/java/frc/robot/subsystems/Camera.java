// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.LimelightHelpers;

public class Camera extends SubsystemBase {

  public static boolean doRejectUpdate = true;
  public static final String limeLight = "limelight";
  private static PIDController pidControllerX = new PIDController(0.1, 0., 0.0); // PID for Left, Right
  private static PIDController pidControllerY = new PIDController(0.1, 0., 0.0); // PID for Forward, Backward

  /** Creates a new Camera. */
  public Camera() {}

  @Override
  public void periodic() {
    // This method will be called once per scheduler run

    SmartDashboard.putNumber("Target X", moveInputX());
    SmartDashboard.putNumber("Target Y", moveInputY());
  }
  
  public double moveInputX() {
    double tx = LimelightHelpers.getTX(limeLight); // Get April Tag X
    return pidControllerX.calculate(tx, 0);
  }

  public double moveInputY() {
    double ty = LimelightHelpers.getTY(limeLight); // Get April Tag Y
    return pidControllerY.calculate(ty, 0);
  }
}
