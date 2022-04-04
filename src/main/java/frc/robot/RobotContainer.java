// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.List;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.RamseteController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.math.trajectory.constraint.DifferentialDriveVoltageConstraint;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.commands.teleop.*;
import frc.robot.misc.DashboardSettings;
import frc.robot.misc.LedSubsystem;
import frc.robot.misc.Constants.AutoConstants;
import frc.robot.misc.Constants.DriveConstants;
import frc.robot.commands.auto.autoCommands.*;
import frc.robot.commands.auto.commandGroups.*;
import frc.robot.subsystems.*;
import frc.robot.subsystems.drive.DrivetrainSubsystem;
import frc.robot.subsystems.drive.AutoTrajectories;
import edu.wpi.first.wpilibj2.command.*;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.POVButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;



public class RobotContainer {

  //Subsystems:
  private final DrivetrainSubsystem drivetrain = new DrivetrainSubsystem();
  private final LimelightSubsystem limelight = new LimelightSubsystem();
  private final IntakeSubsystem intakeSubsystem = new IntakeSubsystem();
  private final ShooterSubsystem shooterSubsystem = new ShooterSubsystem();
  private final ClimberSubsystem climberSubsystem = new ClimberSubsystem();
  private final LedSubsystem leds = new LedSubsystem();

  //Commands:
  private final IntakeCommand intakeCommand = new IntakeCommand(intakeSubsystem, shooterSubsystem);
  private final OuttakeCommand outtakeCommand = new OuttakeCommand(shooterSubsystem, intakeSubsystem);

  private final ShootCommand shootCommand = new ShootCommand(11027, 9990, shooterSubsystem, intakeSubsystem);
  private final ShootCommand lowShootCommand = new ShootCommand(8500, 3495, shooterSubsystem, intakeSubsystem);
  private final ShootCommand farShootCommand = new ShootCommand(13027, 11990, shooterSubsystem, intakeSubsystem);
  private final ShootCommand hailMaryShootCommand = new ShootCommand(15833, 19000, shooterSubsystem, intakeSubsystem);
  private final ShootCommand maxShootCommand = new ShootCommand(18500, 18500, shooterSubsystem, intakeSubsystem);

  private final LimelightCenterCommand limelightCenterCommand = new LimelightCenterCommand(limelight, drivetrain);//Command groups:

  //Command groups:
  private final TwoBallAutoSequence twoBallAuto = new TwoBallAutoSequence(drivetrain, shooterSubsystem, intakeSubsystem);
  private final OneBallAutoSequence oneBallAuto = new OneBallAutoSequence(drivetrain, shooterSubsystem);
  private final RamseteDriveCommand blueHuman_TwoBalls_Stage1 = new RamseteDriveCommand(drivetrain, Robot.trajectory1);
  // private final TrajectoryDrive trajectoryDrive = new TrajectoryDrive(drivetrain);

  //Controllers:
  private final XboxController xboxController = new XboxController(0);
  private final XboxController driveController  = new XboxController(1);

  //Choosers: 
  private SendableChooser<Command> autoChooser = new SendableChooser<>();
  private DashboardSettings subsystemViews = new DashboardSettings(limelight, drivetrain, shooterSubsystem);


  /** Example RAMSETE command now found in {@link AutoTrajectories} */
  // private RamseteCommand ramseteCommand = new RamseteCommand(
  //   AutoTrajectories.exampleTrajectory,
  //   drivetrain::getPose,
  //   new RamseteController(AutoConstants.kRamseteB, AutoConstants.kRamseteZeta),
  //   new SimpleMotorFeedforward(
  //     DriveConstants.ksVolts,
  //     DriveConstants.kvVoltSecondsPerMeter,
  //     DriveConstants.kaVoltSecondsSquaredPerMeter
  //   ),
  //   DriveConstants.kDriveKinematics,
  //   drivetrain::getWheelSpeeds,
  //   new PIDController(DriveConstants.kPDriveVel, 0, 0),
  //   new PIDController(DriveConstants.kPDriveVel, 0, 0),
  //   // RamseteCommand passes volts to the callback
  //   drivetrain::tankDriveVolts,
  //   drivetrain
  // );

  public RobotContainer() {
    
    configureButtonBindings();
    
    //Autochooser options:
    autoChooser.addOption("2 ball auto sequence", twoBallAuto);
    autoChooser.addOption("1 ball auto sequence", oneBallAuto);
    autoChooser.addOption("testBlueHumanAutoDrive", blueHuman_TwoBalls_Stage1);
    //autoChooser.addOption("Trajectory", trajectoryDrive);
    SmartDashboard.putData("Auto command selection", autoChooser);
    

    //Default Commands:
    drivetrain.setDefaultCommand(
      new DefaultDriveCommand(driveController::getLeftY, driveController::getRightX, drivetrain)
    );

    climberSubsystem.setDefaultCommand(
      new ClimberCommand(xboxController::getRightY, climberSubsystem)
    );
  }
  
  private void configureButtonBindings() {
    //Xbox buttons: 
    new JoystickButton(xboxController, XboxController.Button.kA.value)
      .whileHeld(intakeCommand);
    
    new JoystickButton(xboxController, XboxController.Button.kBack.value)
      .whileHeld(outtakeCommand);

    new JoystickButton(xboxController, XboxController.Button.kLeftBumper.value)
      .whileHeld(lowShootCommand);
    
    new JoystickButton(xboxController, XboxController.Button.kRightBumper.value)
      .whileHeld(shootCommand);
    
    new JoystickButton(xboxController, XboxController.Button.kB.value)
      .whileHeld(farShootCommand);
    
    new JoystickButton(xboxController, XboxController.Button.kStart.value)
      .whileHeld(hailMaryShootCommand);

    new JoystickButton(xboxController, XboxController.Button.kY.value)
      .whenPressed(climberSubsystem::togglePivot);

    new JoystickButton(xboxController, XboxController.Button.kX.value)
      .whenPressed(climberSubsystem::toggleSolenoid);

   
    //Drive controller buttons:
    new JoystickButton(driveController, XboxController.Button.kRightBumper.value)
      .whenPressed(drivetrain::toggleDriveGear);

    new JoystickButton(driveController, XboxController.Button.kLeftBumper.value)
      .whenPressed(drivetrain::toggleHalvedSpeed)
      .whenReleased(drivetrain::toggleHalvedSpeed);

    new JoystickButton(driveController, XboxController.Button.kStart.value)
      .whenPressed(drivetrain::toggleInvertDrive);

    new JoystickButton(driveController, XboxController.Button.kY.value)
      .toggleWhenPressed(limelightCenterCommand);
  }
  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    return autoChooser.getSelected();
  }
}