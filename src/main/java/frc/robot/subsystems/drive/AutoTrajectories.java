package frc.robot.subsystems.drive;

import java.util.List;

import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.math.trajectory.constraint.DifferentialDriveVoltageConstraint;
import frc.robot.misc.Constants;
import frc.robot.misc.Constants.AutoConstants;
import frc.robot.misc.Constants.DriveConstants;

public abstract class AutoTrajectories {

    public static TrajectoryConfig config = 
    new TrajectoryConfig(Constants.AutoConstants.kMaxSpeedMetersPerSecond, Constants.AutoConstants.kMaxAccelerationMetersPerSecondSquared)
      .setKinematics(Constants.DriveConstants.kDriveKinematics) // Add kinematics to ensure max speed is actually obeyed
      .addConstraint(
        new DifferentialDriveVoltageConstraint( // Apply the voltage constraint
            new SimpleMotorFeedforward(
                Constants.DriveConstants.ksVolts,
                Constants.DriveConstants.kvVoltSecondsPerMeter,
                Constants.DriveConstants.kaVoltSecondsSquaredPerMeter
            ),
            Constants.DriveConstants.kDriveKinematics,
            10
        ));

       // An example trajectory to follow.  All units in meters.
    public static Trajectory exampleTrajectory = 
    TrajectoryGenerator.generateTrajectory(
        new Pose2d(0, 0, new Rotation2d(0)), // Start at the origin facing the +X direction
        List.of(new Translation2d(1, 1), new Translation2d(2, -1)), // Pass through these two interior waypoints, making an 's' curve path
        new Pose2d(3, 0, new Rotation2d(0)), // End 3 meters straight ahead of where we started, facing forward
        config //pass config
    ); 
}
