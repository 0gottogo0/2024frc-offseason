// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.RotationsPerSecond;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.Camera;
import frc.robot.subsystems.Candle;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Shooter;

public class Robot extends TimedRobot {
  private Command m_autonomousCommand;

  private RobotContainer m_robotContainer;

  private Timer autoTimer = new Timer();

  private PIDController autoDriveControllerX = new PIDController(2, 0.00, 0.00);
  private PIDController autoDriveControllerY = new PIDController(2, 0.00, 0.00);
  private PIDController autoRotateController = new PIDController(0.25, 0.00, 0.00);

  private final CommandSwerveDrivetrain drivetrain = TunerConstants.DriveTrain;
  Camera camera = new Camera();
  Candle candle = new Candle();
  Intake intake = new Intake();
  Shooter shooter = new Shooter();

  private double MaxSpeed = TunerConstants.kSpeedAt12Volts.in(MetersPerSecond); // kSpeedAt12Volts desired top speed
  private double MaxAngularRate = RotationsPerSecond.of(.75).in(RadiansPerSecond);

  private final SwerveRequest.FieldCentric driveAuto = new SwerveRequest.FieldCentric()
  .withDeadband(MaxSpeed * 0.02).withRotationalDeadband(MaxAngularRate * 0.02) // Add a 10% deadband
  .withDriveRequestType(DriveRequestType.OpenLoopVoltage); // I want field-centric
                                                           // driving in open loop

  private final SwerveRequest.RobotCentric driveAutoTrack = new SwerveRequest.RobotCentric()
      .withDeadband(MaxSpeed * 0.02).withRotationalDeadband(MaxAngularRate * 0.02) // Add a 10% deadband
      .withDriveRequestType(DriveRequestType.OpenLoopVoltage); // I want field-centric
                                                               // driving in open loop

  private static final String autoDefault        = "Default";
  private static final String autoTestMove       = "TestMove";
  private static final String autoTestRotate     = "TestRotate";
  private static final String autoTestTrack      = "TestTrack";
  private static final String autoTest           = "Test";
  // Naming of the reef: https://www.chiefdelphi.com/t/naming-convention-for-the-reef/479883
  private static final String autoGH1            = "GH1";
  private static final String autoG4C4D4         = "G4C4D4";

  private String m_autoSelected;
  private final SendableChooser<String> m_autoChooser = new SendableChooser<>();

  @Override
  public void robotInit() {
    m_robotContainer = new RobotContainer();

    m_autoChooser.setDefaultOption("Do nothing", autoDefault);
    m_autoChooser.addOption(       "TEST: MOVE", autoTestMove);
    m_autoChooser.addOption(       "TEST: ROTATE", autoTestRotate);
    m_autoChooser.addOption(       "TEST: TRACK", autoTestTrack);
    m_autoChooser.addOption(       "TEST: ALL", autoTest);

    m_autoChooser.addOption(       "1 Coral Center, L1 (G/H1)", autoGH1);
    m_autoChooser.addOption(       "3 Coral Right, L4 (G4C4D4)", autoG4C4D4);

    SmartDashboard.putData("Auto Chooser", m_autoChooser);
  }

  @Override
  public void robotPeriodic() {
    CommandScheduler.getInstance().run();

    SmartDashboard.putNumber("Robot X", drivetrain.getState().Pose.getX());
    SmartDashboard.putNumber("Robot Y", drivetrain.getState().Pose.getY());
  }

  @Override
  public void disabledInit() {}

  @Override
  public void disabledPeriodic() {}

  @Override
  public void disabledExit() {}

  @Override
  public void autonomousInit() {
    m_autoSelected = m_autoChooser.getSelected();
    autoTimer.reset();
  }

  @Override
  public void autonomousPeriodic() {
    switch (m_autoSelected) {
      case autoDefault:
        break;
      case autoTestMove:
        System.out.println("Testing Drive!");
        while (!autoDrive(1, 0, 0, 0.1)) {}
        while (!autoDrive(1, 1, 0, 0.1))
        break;
      case autoTestRotate:
        System.out.println("Testing Rotation!");
        while (!autoRotate(90)) {}
        break;
      case autoTestTrack:
        System.out.println("Testing Tracking!");
        while (!autoTrack( 0, 0, 0, 0.1)) {}
        break;
      case autoTest:
        System.out.println("Testing Auto!");
        while (!autoDrive(2, 2, 180, 0.2))
        while (!autoTrack(0, 0, 180, 0.1))
        while (!autoDrive(0, 0, 180, 0.2))
        break;
      case autoGH1:
        while (!autoDrive(1.52, 0, 0, 0.1)) {candle.setRGB(0, 0, 255);}
        while (!autoWait(1)) {candle.setRGB(0, 255, 0);
                                       shooter.shoot(0.6);}
        while (!autoDrive(0, 0, 0, 0.1)) {candle.setRGB(0, 0, 255);
                                                                                    shooter.shootStop();}
        break;
      case autoG4C4D4:
        while (!autoDrive(1.52, 0.18, 0, 0.1)) {candle.setRGB(0, 0, 255);}
        while (!autoWait(1)) {candle.setRGB(0, 255, 0);
                                       shooter.shoot(0.6);}
        while (!autoDrive(1.22, 1.61, 0, 0.8)) {candle.setRGB(0, 0, 255);
                                                                                          shooter.shootStop();}
        while (!autoDrive(4.85, 2.72, -126, 0.8)) {candle.setRGB(255, 0, 255);}
        while (!autoDrive(6.28, 3.12, -126, 0.2)) {candle.setRGB(255, 0, 0);}
        while (!autoWait(1)) {candle.setRGB(0, 255, 0);
                                       intake.intakeBack();}
        while (!autoDrive(3.54, 0.91, -121, 0.1)) {candle.setRGB(0, 0, 255);
                                                                              intake.intakeStop();}
        while (!autoWait(1)) {candle.setRGB(0, 255, 0);
                                       shooter.shoot(0.6);}
        while (!autoDrive(6.28, 3.12, -126, 0.2)) {candle.setRGB(0, 0, 255);
                                                                              shooter.shootStop();}
        while (!autoWait(1)) {candle.setRGB(0, 255, 0);
                                       intake.intakeBack();}
        while (!autoWait(1)) {candle.setRGB(0, 0, 255);
                                       intake.intakeStop();}
    }
  }

  @Override
  public void autonomousExit() {}

  @Override
  public void teleopInit() {
    autoTimer.stop();
  }

  @Override
  public void teleopPeriodic() {}

  @Override
  public void teleopExit() {}

  @Override
  public void testInit() {
    CommandScheduler.getInstance().cancelAll();
  }

  @Override
  public void testPeriodic() {}

  @Override
  public void testExit() {}

  @Override
  public void simulationPeriodic() {}

  public boolean autoDrive(double targetX, double targetY, double targetRotation, double targetArea) {
    driveAuto.withVelocityX(MathUtil.clamp(MathUtil.applyDeadband(autoDriveControllerX.calculate(drivetrain.getState().Pose.getX(), targetX), targetArea), -Constants.autoSpeed, Constants.autoSpeed)) // Drive forward with negative Y (forward)
             .withVelocityY(MathUtil.clamp(MathUtil.applyDeadband(autoDriveControllerY.calculate(drivetrain.getState().Pose.getY(), targetY), targetArea), -Constants.autoSpeed, Constants.autoSpeed)) // Drive left with negative X (left)
             .withRotationalRate(MathUtil.clamp(autoRotateController.calculate(drivetrain.getState().Pose.getRotation().getDegrees(), targetRotation), -Constants.autoSpeed, Constants.autoSpeed));
    
    // Returns true if we get within the target area
    // Ignores rotation so auto is more "smooth"
    if ((Math.abs(drivetrain.getState().Pose.getX()) - targetX) < targetArea && (Math.abs(drivetrain.getState().Pose.getY()) - targetY) < targetArea) {
      return true;
    } else {
      return false;
    }
  }

  public boolean autoTrack(double targetX, double targetY, double targetRotation, double targetArea) {
    driveAutoTrack.withVelocityX(MathUtil.clamp(MathUtil.applyDeadband(autoDriveControllerX.calculate(camera.moveInputY(), targetX), targetArea), -Constants.autoSpeed, Constants.autoSpeed)) // Drive forward with negative Y (forward)
                  .withVelocityY(MathUtil.clamp(MathUtil.applyDeadband(autoDriveControllerY.calculate(camera.moveInputX(), targetY), targetArea), -Constants.autoSpeed, Constants.autoSpeed)) // Drive left with negative X (left)
                  .withRotationalRate(MathUtil.clamp(autoRotateController.calculate(drivetrain.getState().Pose.getRotation().getDegrees(), targetRotation), -Constants.autoSpeed, Constants.autoSpeed));
    
    // Returns true if we get within the target area
    // Does not ignores rotation so auto can lineup with target
    // It does result in a long if condition that is unreadable but it works (hopefully...)
    if ((Math.abs(camera.moveInputY()) - targetX) < targetArea && (Math.abs(camera.moveInputX()) - targetY) < targetArea && targetRotation < (targetRotation + 0.5) && targetRotation > (targetRotation - 0.5)) {
      return true;
    } else {
      return false;
    }
  }

  public boolean autoRotate(double targetRotation) {
    driveAuto.withVelocityX(0) // Drive forward with negative Y (forward)
             .withVelocityY(0) // Drive left with negative X (left)
             .withRotationalRate(MathUtil.clamp(autoRotateController.calculate(drivetrain.getState().Pose.getRotation().getDegrees(), targetRotation), -Constants.autoSpeed, Constants.autoSpeed));
    
    // Returns true if we get within a 0.5 degree area of the setpoint
    if (targetRotation < (targetRotation + 0.5) && targetRotation > (targetRotation - 0.5)) {
      return true;
    } else {
      return false;
    }
  }

  public boolean autoWait(double waitTime) {
    autoTimer.reset();
    autoTimer.start();

    if (autoTimer.get() > waitTime) {
      return true;
    } else {
      return false;
    }
  }
}
