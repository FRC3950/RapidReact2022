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
import frc.robot.misc.Constants.AutoConstants;
import frc.robot.misc.Constants.DriveConstants;
import frc.robot.commands.auto.autoCommands.*;
import frc.robot.commands.auto.commandGroups.*;
import frc.robot.subsystems.*;
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

  //Commands:
  private final IntakeCommand intakeCommand = new IntakeCommand(intakeSubsystem, shooterSubsystem);
  private final OuttakeCommand outtakeCommand = new OuttakeCommand(shooterSubsystem, intakeSubsystem);

  private final ShootCommand shootCommand = new ShootCommand(11027, 9990, shooterSubsystem, intakeSubsystem);
  private final ShootCommand lowShootCommand = new ShootCommand(8500, 3495, shooterSubsystem, intakeSubsystem);
  private final ShootCommand farShootCommand = new ShootCommand(13027, 11990, shooterSubsystem, intakeSubsystem);
  private final ShootCommand hailMaryShootCommand = new ShootCommand(15833, 19000, shooterSubsystem, intakeSubsystem);

  private final CenterCommand centerCommand = new CenterCommand(drivetrain, limelight);

  //Auto commands:
  private final AutoEncoderDrive autoEncoderDrive = new AutoEncoderDrive(100000, 0.75, drivetrain);
  private final AutoShootCommand autoShootCommand = new AutoShootCommand(11027.0, 9990.0, 6, shooterSubsystem);
  

     
 // Create a voltage constraint to ensure we don't accelerate too fast
  //PUt straight inside
 

// Create config for trajectory
  TrajectoryConfig config =
    new TrajectoryConfig(AutoConstants.kMaxSpeedMetersPerSecond, AutoConstants.kMaxAccelerationMetersPerSecondSquared)
      .setKinematics(DriveConstants.kDriveKinematics) // Add kinematics to ensure max speed is actually obeyed
      .addConstraint(new DifferentialDriveVoltageConstraint( // Apply the voltage constraint
        new SimpleMotorFeedforward(
          DriveConstants.ksVolts,
          DriveConstants.kvVoltSecondsPerMeter,
          DriveConstants.kaVoltSecondsSquaredPerMeter),
          DriveConstants.kDriveKinematics,
          10
        )
      );

     // An example trajectory to follow.  All units in meters.
  Trajectory exampleTrajectory = TrajectoryGenerator.generateTrajectory(
    // Start at the origin facing the +X direction
    new Pose2d(0, 0, new Rotation2d(0)),
    // Pass through these two interior waypoints, making an 's' curve path
    List.of(new Translation2d(1, 1), new Translation2d(2, -1)),
    // End 3 meters straight ahead of where we started, facing forward
    new Pose2d(3, 0, new Rotation2d(0)),
    // Pass config
    config);

    

//Auto Trajectory
  RamseteCommand ramseteCommand = new RamseteCommand(
    exampleTrajectory,
    drivetrain::getPose,
    new RamseteController(AutoConstants.kRamseteB, AutoConstants.kRamseteZeta),
    new SimpleMotorFeedforward(
      DriveConstants.ksVolts,
      DriveConstants.kvVoltSecondsPerMeter,
      DriveConstants.kaVoltSecondsSquaredPerMeter
    ),
    DriveConstants.kDriveKinematics,
    drivetrain::getWheelSpeeds,
    new PIDController(DriveConstants.kPDriveVel, 0, 0),
    new PIDController(DriveConstants.kPDriveVel, 0, 0),
    // RamseteCommand passes volts to the callback
    drivetrain::tankDriveVolts,
    drivetrain
  );



  //Command groups:
  private final TwoBallAutoSequence twoBallAuto = new TwoBallAutoSequence(drivetrain, shooterSubsystem, intakeSubsystem);


  //Controllers:
  private final XboxController xboxController = new XboxController(0);
  private final XboxController driveController  = new XboxController(1);

  //Choosers: 
  private SendableChooser<Command> autoChooser = new SendableChooser<>();

 



  public RobotContainer() {
    
    configureButtonBindings();
    
    //Autochooser options:
    autoChooser.addOption("2 ball auto sequence", twoBallAuto);
    autoChooser.addOption("Test auto drive forward", autoEncoderDrive);
    autoChooser.addOption("Test auto shoot", autoShootCommand);
    autoChooser.addOption("Trajectory", ramseteCommand);

    //Smartdashboard:
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

   
    //Joystick buttons:
    new JoystickButton(driveController, XboxController.Button.kRightBumper.value)
      .whenPressed(drivetrain::toggleDriveGear);

    new JoystickButton(driveController, XboxController.Button.kStart.value)
      .whenPressed(drivetrain::toggleInvertDrive);

    new JoystickButton(driveController, XboxController.Button.kLeftBumper.value)
      .whenPressed(drivetrain::toggleHalvedSpeed);

   // new JoystickButton(drivestick, 2)
     // .whenPressed(centerCommand);
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