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

  //Command groups:
  private final TwoBallAutoSequence twoBallAuto = new TwoBallAutoSequence(drivetrain, shooterSubsystem, intakeSubsystem);


  //Controllers:
  private final XboxController xboxController = new XboxController(0);
  private final Joystick drivestick  = new Joystick(1);

  //Choosers: 
  private SendableChooser<Command> autoChooser = new SendableChooser<>();

 



  public RobotContainer() {
    
    configureButtonBindings();
    
    //Autochooser options:
    autoChooser.addOption("2 ball auto sequence", twoBallAuto);
    autoChooser.addOption("Test auto drive forward", autoEncoderDrive);
    autoChooser.addOption("Test auto shoot", autoShootCommand);

    //Smartdashboard:
    SmartDashboard.putData("Auto command selection", autoChooser);

    //Default Commands:
    drivetrain.setDefaultCommand(
      new DefaultDriveCommand(drivestick::getTwist, drivestick::getY, drivetrain)
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
    new JoystickButton(drivestick, 6)
      .whenPressed(climberSubsystem::toggleSolenoid);

    new JoystickButton(drivestick, 5)
      .whenPressed(drivetrain::toggleDriveGear);

    new JoystickButton(drivestick, 7)
      .whenPressed(drivetrain::toggleInvertDrive);

    new JoystickButton(drivestick, 11)
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