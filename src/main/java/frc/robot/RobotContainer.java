// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.commands.teleop.*;
import frc.robot.misc.ShooterMotorsOn;
import frc.robot.commands.auto.autoCommands.*;
import frc.robot.commands.auto.commandGroups.*;
import frc.robot.subsystems.*;
import edu.wpi.first.wpilibj2.command.*;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;


public class RobotContainer {

  //Subsystems:
  private final DrivetrainSubsystem drivetrain = new DrivetrainSubsystem();
  private final IntakeSubsystem intakeSubsystem = new IntakeSubsystem();
  private final ShooterSubsystem shooterSubsystem = new ShooterSubsystem();
  private final ClimberSubsystem climberSubsystem = new ClimberSubsystem();

  //Commands:
  private final IntakeCommand intakeCommand = new IntakeCommand(intakeSubsystem, shooterSubsystem);
  private final ToggleSolenoidCommand toggle = new ToggleSolenoidCommand(intakeSubsystem);
  private final ShootCommand shootCommand = new ShootCommand(shooterSubsystem);
  private final OuttakeCommand outtakeCommand = new OuttakeCommand(shooterSubsystem, intakeSubsystem);
  private final CenterCommand centerCommand = new CenterCommand(drivetrain);


  //Auto commands:
  private final AutoEncoderDrive autoEncoderDrive = new AutoEncoderDrive(200000, 0.5, drivetrain);
  private final AutoShootCommand autoShootCommand = new AutoShootCommand(shooterSubsystem, 6);

  //Command groups:
  private final DriveSequence autoDriveSequence = new DriveSequence(drivetrain, shooterSubsystem);

  //Controllers:
  private final XboxController xboxController = new XboxController(0);
  private final Joystick drivestick  = new Joystick(1);

  //Choosers: 
  private SendableChooser<Command> autoChooser = new SendableChooser<>();


  public RobotContainer() {
    configureButtonBindings();
    
    //Autochooser options:
    autoChooser.addOption("Test auto sequence", autoDriveSequence);
    autoChooser.addOption("Test auto drive forward", autoEncoderDrive);
    autoChooser.addOption("Test auto shoot", autoShootCommand);

    //Smartdashboard:
    SmartDashboard.putData("Auto command selection", autoChooser);

    //Default Commands:
    drivetrain.setDefaultCommand(
      new DefaultDriveCommand(drivestick::getTwist, drivestick::getY, drivetrain)
    );

    climberSubsystem.setDefaultCommand(
      new ClimberCommand(climberSubsystem, xboxController::getRightY)
    );
  }
  
  private void configureButtonBindings() {
    //Xbox buttons: 
    new JoystickButton(xboxController, XboxController.Button.kA.value)
      .whileHeld(intakeCommand);
    
    new JoystickButton(xboxController, XboxController.Button.kBack.value)
      .whileHeld(outtakeCommand);

    new JoystickButton(xboxController, XboxController.Button.kB.value)
      .whenPressed(() -> intakeSubsystem.setSolenoid(Value.kForward));


    new JoystickButton(xboxController, XboxController.Button.kX.value)
      .whileHeld(shootCommand);

    new JoystickButton(xboxController, XboxController.Button.kLeftBumper.value)
      .whenPressed(shooterSubsystem::decrementTargetVelocity);
    
    new JoystickButton(xboxController, XboxController.Button.kRightBumper.value)
      .whenPressed(shooterSubsystem::incrementTargetVelocity);


    // new JoystickButton(xboxController, XboxController.Axis.kRightY.value)
    //   .whileHeld(climberCommand);

    new JoystickButton(xboxController, XboxController.Button.kY.value)
      .whenPressed(climberSubsystem::togglePivot);

    
    //Joystick buttons:
    new JoystickButton(drivestick, 5)
      .whenPressed(drivetrain::toggleDriveGear);

    new JoystickButton(drivestick, 2)
      .whenPressed(centerCommand);
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