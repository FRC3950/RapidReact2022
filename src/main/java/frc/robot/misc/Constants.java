// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.misc;

import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
    
//Drivetrain
    public static int leftM = 0;
    public static int leftS = 1;
    public static int rightM = 2;
    public static int rightS = 3;

    public static int[] driveSol = {1,5}; //Solenoid

    public static final double kWidthMeters = 0.5;

//Conveyor
    public static int conveyor = 4;
    public static int indexer = 19;
    public static int sensor = 1;

//Shooter 
    public static int top = 8;
    public static int bottom = 7;

//Intake
    public static int intake = 9;
    public static int[] solenoid = {0,4};

//Climber
    public static int[] winch = {2,6};
    public static int[] pivot = {3,7};

    public static int master = 6;
    public static int follower = 5;

//Solenoid States:

    public static class States{
        public static Value DOWN = Value.kReverse;
        public static Value UP = Value.kForward;

        public static Value HIGH = Value.kReverse;
        public static Value LOW = Value.kForward;

        public static Value LOCKED = Value.kForward;
        public static Value UNLOCKED = Value.kReverse;
    }

    public static final class DriveConstants {
       
        public static final double kTrackwidthMeters = 0.457;
        public static final DifferentialDriveKinematics kDriveKinematics =
            new DifferentialDriveKinematics(kTrackwidthMeters);
    
        public static final int kEncoderCPR = 2048;
        public static final double kWheelDiameterMeters = Units.inchesToMeters(4);
        public static final double kEncoderDistancePerPulse =
            // Assumes the encoders are directly mounted on the wheel shafts
            (kWheelDiameterMeters * Math.PI) / (double) kEncoderCPR;
    
        // These are example values only - DO NOT USE THESE FOR YOUR OWN ROBOT!
        // These characterization values MUST be determined either experimentally or theoretically
        // for *your* robot's drive.
        // The Robot Characterization Toolsuite provides a convenient tool for obtaining these
        // values for your robot.
        public static final double ksVolts = 0.1088;
        public static final double kvVoltSecondsPerMeter = 5.029;
        public static final double kaVoltSecondsSquaredPerMeter = 0.44158;
    
        // Example value only - as above, this must be tuned for your drive!
        public static final double kPDriveVel = 0.66725;
    }

    public static final class AutoConstants {
        public static final double kMaxSpeedMetersPerSecond = 0.5;
        public static final double kMaxAccelerationMetersPerSecondSquared = 1.5;
        
        // Reasonable baseline values for a RAMSETE follower in units of meters and seconds
        public static final double kRamseteB = 2;
        public static final double kRamseteZeta = 0.7;
    }
}
