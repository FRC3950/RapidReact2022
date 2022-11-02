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
import frc.robot.misc.Constants.AutoConstants;
import frc.robot.misc.Constants.DriveConstants;

import frc.robot.subsystems.drive.DrivetrainSubsystem;
import frc.robot.subsystems.drive.AutoTrajectories;
import edu.wpi.first.wpilibj2.command.*;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.POVButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;



public class RobotContainer {

  //See NYC branch for full container with commands, groups, multiple systems, etc.,

  //Subsystems:
  private final DrivetrainSubsystem drivetrain = new DrivetrainSubsystem();


  //Commands:

  //Command groups:
 

  /** Example RAMSETE command now found in {@link AutoTrajectories} */
  private RamseteCommand ramseteCommand = new RamseteCommand(
    //AutoTrajectories.exampleTrajectory,
    Robot.trajectory1,
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


  //Controllers:
  // private final XboxController xboxController = new XboxController(0);
  private final XboxController driveController  = new XboxController(0);

  //Choosers: 
  private SendableChooser<Command> autoChooser = new SendableChooser<>();


  

  public RobotContainer() {
    
    configureButtonBindings();
    
    //Autochooser options:
    
      //autoChooser.addOption("2 ball auto sequence", twoBallAuto);

      //autoChooser.addOption("ram Inline Command straight", ramseteCommand);
      //autoChooser.addOption("Trajectory", trajectoryDrive);

    
    SmartDashboard.putData("Auto command selection", autoChooser);
    SmartDashboard.putData("reset heading", new InstantCommand(drivetrain::resetHeadingEncoder,drivetrain));

    
    //Default Commands:
    drivetrain.setDefaultCommand(
      new DefaultDriveCommand(driveController::getLeftY, driveController::getRightX, drivetrain)
    );

   
  }
  
  private void configureButtonBindings() {
    //Xbox buttons: 

    
    //Drive controller buttons:
    new JoystickButton(driveController, XboxController.Button.kRightBumper.value)
      .whenPressed(drivetrain::toggleDriveGear);

  

      /*
      Add commands for centering, turn to heading, etc.
      
      */


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