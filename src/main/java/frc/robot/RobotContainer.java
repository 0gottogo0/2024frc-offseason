// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.ctre.phoenix6.Utils;
import com.ctre.phoenix6.mechanisms.swerve.SwerveRequest;
import com.ctre.phoenix6.mechanisms.swerve.SwerveModule.DriveRequestType;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.Aim;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.Feed;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Shooter;

public class RobotContainer {
  private double MaxSpeed = TunerConstants.kSpeedAt12VoltsMps; // kSpeedAt12VoltsMps desired top speed
  private double MaxAngularRate = 3 * Math.PI; // Tune

  /* Setting up bindings for necessary control of the swerve drive platform */
  private final CommandXboxController DriverController = new CommandXboxController(0); // My DriverController
  private final CommandXboxController ManipulatorController = new CommandXboxController(1);
  private final CommandSwerveDrivetrain drivetrain = TunerConstants.DriveTrain; // My drivetrain



  private final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
      .withDeadband(MaxSpeed * 0.1).withRotationalDeadband(MaxAngularRate * 0.1) // Add a 10% deadband
      .withDriveRequestType(DriveRequestType.OpenLoopVoltage); // I want field-centric
                                                               // driving in open loop
  private final SwerveRequest.SwerveDriveBrake brake = new SwerveRequest.SwerveDriveBrake();

  private final Telemetry logger = new Telemetry(MaxSpeed);
  
  public final Aim aim = new Aim();
  public final Shooter shooter = new Shooter();
  public final Intake intake = new Intake();
  public final Elevator elevator = new Elevator();
  public final Feed feed = new Feed();

  private final SendableChooser<Command> autoChooser;
  
  public RobotContainer() {

    NamedCommands.registerCommand("Shooter", shooter.runEnd(
      () -> shooter.shoot(.6),
      () -> shooter.shootStop()));

    NamedCommands.registerCommand("Feed W/ Beam Break", feed.runEnd(
      () -> feed.feedBreak(),
      () -> feed.feedStop()));

    NamedCommands.registerCommand("Feed", feed.runEnd(
      () -> feed.feed(),
      () -> feed.feedStop()));

    NamedCommands.registerCommand("Intake", intake.runEnd(
      () -> intake.intakeBack(),
      () -> intake.intakeStop()));

    autoChooser = AutoBuilder.buildAutoChooser("My Default Auto");

    SmartDashboard.putData("Auto Chooser", autoChooser);

    configureBindings();
  }

  private void configureBindings() {

    drivetrain.setDefaultCommand( // Drivetrain will execute this command periodically
        drivetrain.applyRequest(() ->
        drive.withVelocityX(
          MathUtil.applyDeadband(-DriverController.getLeftY(), 0.15) * MaxSpeed) // Drive forward with negative Y (forward)
             .withVelocityY(MathUtil.applyDeadband(-DriverController.getLeftX(), 0.15) * MaxSpeed) // Drive left with negative X (left)
             .withRotationalRate(-DriverController.getRightX() * MaxAngularRate) // Drive counterclockwise with negative X (left)
        ));

    DriverController.button(8).whileTrue(drivetrain.applyRequest(() -> brake));

    // reset the field-centric heading on left bumper press
    DriverController.button(7).onTrue(drivetrain.runOnce(() -> drivetrain.seedFieldRelative()));

    if (Utils.isSimulation()) {
      drivetrain.seedFieldRelative(new Pose2d(new Translation2d(), Rotation2d.fromDegrees(90)));
    }
    drivetrain.registerTelemetry(logger::telemeterize);

    ManipulatorController.leftTrigger().whileTrue(shooter.runEnd(
      () -> shooter.shoot(.6),
      () -> shooter.shootStop()));

    ManipulatorController.rightTrigger().whileTrue(feed.runEnd(
      () -> feed.feed(),
      () -> feed.feedStop()));

    DriverController.rightBumper().whileTrue(intake.runEnd(
      () -> intake.intakeBack(),
      () -> intake.intakeStop())
      .alongWith(feed.runEnd(
      () -> feed.feedBreak(),
      () -> feed.feedStop())));

    ManipulatorController.y().whileTrue(elevator.runEnd(
      () -> elevator.elevatorOut(0.5),
      () -> elevator.elevatorStop()));

    ManipulatorController.x().whileTrue(elevator.runEnd(
      () -> elevator.elevatorIn(0.5),
      () -> elevator.elevatorStop()));

    DriverController.a().whileTrue(aim.runOnce(
      () -> aim.setAimAtSpeaker()));

    DriverController.b().whileTrue(aim.runOnce(
      () -> aim.setAimUnderStage()));

    ManipulatorController.a().whileTrue(aim.runEnd(
      () -> aim.aimUp(.80),
      () -> aim.aimStop()));

    ManipulatorController.b().whileTrue(aim.runEnd(
      () -> aim.aimDown(.80),
      () -> aim.aimStop()));
  }

  public Command getAutonomousCommand() {
    return autoChooser.getSelected();
  }
}
