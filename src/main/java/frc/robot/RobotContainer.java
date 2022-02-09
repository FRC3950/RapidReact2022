// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.commands.teleop.*;
import frc.robot.misc.ShooterMotorsOn;
import frc.robot.commands.auto.*;
import frc.robot.subsystems.*;
import edu.wpi.first.wpilibj2.command.*;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {

  //Subsystems:
  private final DrivetrainSubsystem drivetrain = new DrivetrainSubsystem();
  private final ShooterSubsystem shooterSubsystem = new ShooterSubsystem();

  //Commands:
  private final ShooterMotorsOn shooterMotorsOn = new ShooterMotorsOn(shooterSubsystem, .8, 11000, 10500);

  //Command groups:
  private final DriveSequence autoDriveSequence = new DriveSequence(drivetrain);

  //Controllers:
  private final XboxController xboxController = new XboxController(0);

  public RobotContainer() {
    configureButtonBindings();

    //Smartdashboard Buttons:
    SmartDashboard.putData("shooter", shooterMotorsOn);

    //Default Commands:
    drivetrain.setDefaultCommand(
      new DefaultDriveCommand(xboxController::getLeftY, xboxController::getRightX, drivetrain)
    );
  }
  
  private void configureButtonBindings() {}
  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    return autoDriveSequence;
  }
}
