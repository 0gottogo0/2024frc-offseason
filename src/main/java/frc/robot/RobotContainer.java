// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.*;

import com.ctre.phoenix6.Utils;
import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;
// import com.pathplanner.lib.auto.AutoBuilder;
// import com.pathplanner.lib.auto.NamedCommands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;
import frc.robot.commands.AutoTestDrive;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.Aim;
import frc.robot.subsystems.Camera;
import frc.robot.subsystems.Candle;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.Feed;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.CommandSwerveDrivetrain.Test;

public class RobotContainer {
  private double MaxSpeed = TunerConstants.kSpeedAt12Volts.in(MetersPerSecond); // kSpeedAt12Volts desired top speed
  private double MaxAngularRate = RotationsPerSecond.of(.75).in(RadiansPerSecond);

  private SlewRateLimiter xLimiter = new SlewRateLimiter(18.0); //TODO: adjust these
  private SlewRateLimiter yLimiter = new SlewRateLimiter(18.0); //limit rate of change of joystick inputs
  private SlewRateLimiter rotLimiter = new SlewRateLimiter(35.0); //reduce brownouts

  /* Setting up bindings for necessary control of the swerve drive platform */
  private final CommandXboxController DriverController = new CommandXboxController(0); // My DriverController
  private final CommandXboxController ManipulatorController = new CommandXboxController(1);
  private final CommandSwerveDrivetrain drivetrain = TunerConstants.DriveTrain; // My drivetrain

  private final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
      .withDeadband(MaxSpeed * 0.1).withRotationalDeadband(MaxAngularRate * 0.1) // Add a 10% deadband
      .withDriveRequestType(DriveRequestType.OpenLoopVoltage); // I want field-centric
                                                               // driving in open loop

  private final SwerveRequest.RobotCentric driveTrack = new SwerveRequest.RobotCentric()
      .withDeadband(MaxSpeed * 0.02).withRotationalDeadband(MaxAngularRate * 0.02) // Add a 10% deadband
      .withDriveRequestType(DriveRequestType.OpenLoopVoltage); // I want field-centric
                                                               // driving in open loop

  private final SwerveRequest.SwerveDriveBrake brake = new SwerveRequest.SwerveDriveBrake();

  private final Command autoTestDrive = new AutoTestDrive();

  SendableChooser<Command> m_chooser = new SendableChooser<>();

  //private final Telemetry logger = new Telemetry(MaxSpeed); //uncomment for sysid
  
  public final Aim aim = new Aim();
  public final Shooter shooter = new Shooter();
  public final Intake intake = new Intake();
  public final Elevator elevator = new Elevator();
  public final Feed feed = new Feed();
  public final Candle candle = new Candle();
  public final Camera camera = new Camera();

  // private final SendableChooser<Command> autoChooser;
  
  public RobotContainer() {

    autoChooser = AutoBuilder.buildAutoChooser("Example Auto");

    SmartDashboard.putData("Auto Chooser", autoChooser);


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

    NamedCommands.registerCommand("Track", drivetrain.applyRequest(
      () -> driveTrack.withVelocityX(camera.moveInputY() + xLimiter.calculate(-MathUtil.applyDeadband(DriverController.getLeftY(), .1) * MaxSpeed)) // Drive forward with negative Y (forward)
                      .withVelocityY(yLimiter.calculate(-MathUtil.applyDeadband(DriverController.getLeftX(), 0.1) * MaxSpeed)) // Drive left with negative X (left)
                      .withRotationalRate(camera.moveInputX() + rotLimiter.calculate(-MathUtil.applyDeadband(DriverController.getRightX(), .1) * MaxAngularRate))));

    configureBindings();
  }

  private void configureBindings() {

    //sysid tests
    /*
    DriverController.x().and(DriverController.pov(0)).whileTrue(drivetrain.sysIdQuasistatic(Direction.kForward, Test.Translation));
    DriverController.x().and(DriverController.pov(180)).whileTrue(drivetrain.sysIdQuasistatic(Direction.kReverse, Test.Translation));

    DriverController.y().and(DriverController.pov(0)).whileTrue(drivetrain.sysIdDynamic(Direction.kForward, Test.Translation));
    DriverController.y().and(DriverController.pov(180)).whileTrue(drivetrain.sysIdDynamic(Direction.kReverse, Test.Translation));
    
    DriverController.a().and(DriverController.pov(0)).whileTrue(drivetrain.sysIdQuasistatic(Direction.kForward, Test.Rotation));
    DriverController.a().and(DriverController.pov(180)).whileTrue(drivetrain.sysIdQuasistatic(Direction.kReverse, Test.Rotation));

    DriverController.b().and(DriverController.pov(0)).whileTrue(drivetrain.sysIdDynamic(Direction.kForward, Test.Rotation));
    DriverController.b().and(DriverController.pov(180)).whileTrue(drivetrain.sysIdDynamic(Direction.kReverse, Test.Rotation));

    DriverController.rightBumper().and(DriverController.pov(0)).whileTrue(drivetrain.sysIdQuasistatic(Direction.kForward, Test.Steer));
    DriverController.rightBumper().and(DriverController.pov(180)).whileTrue(drivetrain.sysIdQuasistatic(Direction.kReverse, Test.Steer));

    DriverController.leftBumper().and(DriverController.pov(0)).whileTrue(drivetrain.sysIdDynamic(Direction.kForward, Test.Steer));
    DriverController.leftBumper().and(DriverController.pov(180)).whileTrue(drivetrain.sysIdDynamic(Direction.kReverse, Test.Steer));
    */

    //drivetrain.registerTelemetry(logger::telemeterize); //uncomment for sysid
    
    drivetrain.setDefaultCommand( // Drivetrain will execute this command periodically
        drivetrain.applyRequest(
          () -> drive.withVelocityX(xLimiter.calculate(MathUtil.applyDeadband(-DriverController.getLeftY(), 0.02) * MaxSpeed)) // Drive forward with negative Y (forward)
                     .withVelocityY(yLimiter.calculate(MathUtil.applyDeadband(-DriverController.getLeftX(), 0.02) * MaxSpeed)) // Drive left with negative X (left)
                     .withRotationalRate(rotLimiter.calculate(MathUtil.applyDeadband(-DriverController.getRightX(), 0.02) * MaxAngularRate)) // Drive counterclockwise with negative X (left)
        ));
     
    DriverController.button(8).whileTrue(drivetrain.applyRequest(() -> brake));
    
    // reset the field-centric heading on left bumper press
    //DriverController.button(7).onTrue(drivetrain.runOnce(() -> drivetrain.seedFieldCentric()));
    
    if (Utils.isSimulation()) {
      drivetrain.seedFieldCentric();
    }
    
    DriverController.a().whileTrue(drivetrain.applyRequest(
      () -> driveTrack.withVelocityX(camera.moveInputY() + xLimiter.calculate(-MathUtil.applyDeadband(DriverController.getLeftY(), .1) * MaxSpeed)) // Drive forward with negative Y (forward)
                      .withVelocityY(yLimiter.calculate(-MathUtil.applyDeadband(DriverController.getLeftX(), 0.1) * MaxSpeed)) // Drive left with negative X (left)
                      .withRotationalRate(camera.moveInputX() + rotLimiter.calculate(-MathUtil.applyDeadband(DriverController.getRightX(), .1) * MaxAngularRate))));

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
    
    DriverController.x().whileTrue(candle.runOnce(
      () -> candle.setBlue()));

    DriverController.y().whileTrue(candle.runOnce(
      () -> candle.setRed()));

    ManipulatorController.a().whileTrue(aim.runEnd(
      () -> aim.aimUp(.80),
      () -> aim.aimStop()));

    ManipulatorController.b().whileTrue(aim.runEnd(
      () -> aim.aimDown(.80),
      () -> aim.aimStop()));
      
  }

  public Command getAutonomousCommand() {
    // return autoChooser.getSelected();
    return m_chooser.getSelected();
  }
}
