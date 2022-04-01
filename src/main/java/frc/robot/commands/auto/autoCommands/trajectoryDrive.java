// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.auto.autoCommands;

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
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.RamseteCommand;
import frc.robot.misc.Constants.AutoConstants;
import frc.robot.misc.Constants.DriveConstants;
import frc.robot.subsystems.drive.AutoTrajectories;
import frc.robot.subsystems.drive.DrivetrainSubsystem;


//TODO: Organize all of this stuff 

public class TrajectoryDrive extends CommandBase {
  DrivetrainSubsystem drivetrain;
  Trajectory exampleTrajectory;
  RamseteCommand ramseteCommand;

  /** Creates a new TrajectoryDrive. */
  public TrajectoryDrive (DrivetrainSubsystem drivetrain) {
    this.drivetrain = drivetrain;
    addRequirements(drivetrain);
    // Create a voltage constraint to ensure we don't accelerate too fast
    var autoVoltageConstraint =
        new DifferentialDriveVoltageConstraint(
            new SimpleMotorFeedforward(
                DriveConstants.ksVolts,
                DriveConstants.kvVoltSecondsPerMeter,
                DriveConstants.kaVoltSecondsSquaredPerMeter),
            DriveConstants.kDriveKinematics,
            10);

// Create config for trajectory
    TrajectoryConfig config = AutoTrajectories.config;
            // An example trajectory to follow.  All units in meters.
    Trajectory exampleTrajectory = AutoTrajectories.exampleTrajectory;
    // TrajectoryGenerator.generateTrajectory(
    //     // Start at the origin facing the +X direction
    //     new Pose2d(0, 0, new Rotation2d(0)),
    //     // Pass through these two interior waypoints, making an 's' curve path
    //     List.of(new Translation2d(1, 1), new Translation2d(2, -1)),
    //     // End 3 meters straight ahead of where we started, facing forward
    //     new Pose2d(3, 0, new Rotation2d(0)),
    //     // Pass config
    //     config);

    RamseteCommand ramseteCommand =
      new RamseteCommand(
          exampleTrajectory,
          drivetrain::getPose,
          new RamseteController(AutoConstants.kRamseteB, AutoConstants.kRamseteZeta),
          new SimpleMotorFeedforward(
              DriveConstants.ksVolts,
              DriveConstants.kvVoltSecondsPerMeter,
              DriveConstants.kaVoltSecondsSquaredPerMeter),
          DriveConstants.kDriveKinematics,
          drivetrain::getWheelSpeeds,
          new PIDController(DriveConstants.kPDriveVel, 0, 0),
          new PIDController(DriveConstants.kPDriveVel, 0, 0),
          // RamseteCommand passes volts to the callback
          drivetrain::tankDriveVolts,
          drivetrain);   
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(drivetrain);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {

    
// Reset odometry to the starting pose of the trajectory.
    drivetrain.resetOdometry(AutoTrajectories.exampleTrajectory.getInitialPose());

// Run path following command, then stop at the end.
    ramseteCommand.andThen(() -> drivetrain.tankDriveVolts(0, 0));
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {

    drivetrain.tankDriveVolts(0, 0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
