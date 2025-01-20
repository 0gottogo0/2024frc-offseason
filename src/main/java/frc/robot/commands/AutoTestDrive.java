// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import static edu.wpi.first.units.Units.*;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotContainer;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.CommandSwerveDrivetrain;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class AutoTestDrive extends Command {
  /** Creates a new AutoTestDrive. */
  public AutoTestDrive() {
    // Use addRequirements() here to declare subsystem dependencies.
  }

  private final CommandSwerveDrivetrain drivetrain = TunerConstants.DriveTrain;

  private double MaxSpeed = TunerConstants.kSpeedAt12Volts.in(MetersPerSecond); // kSpeedAt12Volts desired top speed
  private double MaxAngularRate = RotationsPerSecond.of(.75).in(RadiansPerSecond);

  private final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
  .withDeadband(MaxSpeed * 0.02).withRotationalDeadband(MaxAngularRate * 0.02) // Add a 10% deadband
  .withDriveRequestType(DriveRequestType.OpenLoopVoltage); // I want field-centric
                                                           // driving in open loop

  private PIDController autoDriveControllerX = new PIDController(5, 0.00, 0.00);
  private PIDController autoDriveControllerY = new PIDController(5, 0.00, 0.00);
  private PIDController autoRotateController = new PIDController(0.25, 0.00, 0.00);

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    drivetrain.applyRequest(
      () -> drive.withVelocityX(0.5 * MaxSpeed) // Drive forward with negative Y (forward)
                 .withVelocityY(0 * MaxSpeed) // Drive left with negative X (left)
                 .withRotationalRate(0 * MaxAngularRate) // Drive counterclockwise with negative X (left)
    );
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
