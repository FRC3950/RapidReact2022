// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.auto.autoCommands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.RamseteController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.RamseteCommand;
import frc.robot.misc.Constants.AutoConstants;
import frc.robot.misc.Constants.DriveConstants;
import frc.robot.subsystems.drive.DrivetrainSubsystem;

public class RamseteDriveCommand extends RamseteCommand {

  // final boolean resetPosition;
  final Trajectory trajectory;
  final DrivetrainSubsystem drivetrain;
  /** Creates a new RamseteDriveCommand. */
  public RamseteDriveCommand(DrivetrainSubsystem drivetrain, Trajectory trajectory) {
    super(
      trajectory,
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

// this.resetPosition = true;
    this.trajectory = trajectory;
    this.drivetrain = drivetrain;

    // Use addRequirements() here to declare subsystem dependencies.

  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {

    super.initialize();
    drivetrain.setLowGear();

  //       if (resetPosition) {
  //           drivetrain.resetOdometry(trajectory.getInitialPose());
  // 
  }
  // Called every time the scheduler runs while the command is scheduled.
  //Not going to overide other methods...


  // @Override
  // public void execute() {}

  // // Called once the command ends or is interrupted.
  // @Override
  // public void end(boolean interrupted) {}

  // // Returns true when the command should end.
  // @Override
  // public boolean isFinished() {
  //   return false;
  // }
}

//My assumption is we inherit the Ramsette Commands End/isFinished by intializing super();