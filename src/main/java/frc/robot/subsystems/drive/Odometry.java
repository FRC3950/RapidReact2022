package frc.robot.subsystems.drive;
import edu.wpi.first.math.util.Units;
import frc.robot.subsystems.drive.DrivetrainSubsystem;

public abstract class Odometry extends DrivetrainSubsystem{

    public static synchronized int distanceToNativeUnits(double positionMeters){ 

      double wheelRotations = positionMeters/(2 * Math.PI * Units.inchesToMeters(2)); 
      double motorRotations = wheelRotations * DrivetrainSubsystem.kGearRatio; 
    
      int sensorCounts = (int)(motorRotations * 2048); 
    
      return sensorCounts; 
    
    } 
    
    public static synchronized int velocityToNativeUnits(double velocityMetersPerSecond){ 
    
      double wheelRotationsPerSecond = velocityMetersPerSecond/(2 * Math.PI * Units.inchesToMeters(2)); 
    
      double motorRotationsPerSecond = wheelRotationsPerSecond * DrivetrainSubsystem.kGearRatio;  
      double motorRotationsPer100ms = motorRotationsPerSecond / 10; 
    
      int sensorCountsPer100ms = (int)(motorRotationsPer100ms * 2048); 
    
      return sensorCountsPer100ms;
    } 
    
    public static double nativeUnitsToVelocity(double ticksPer100ms){ 
    
      double rotationPer100ms = ticksPer100ms / 2048;
      double rotationPerSec = rotationPer100ms * 10;
      double wheelRotPerSec = rotationPerSec / DrivetrainSubsystem.kGearRatio;
      double wheelVelocity = wheelRotPerSec * (2 * Math.PI * Units.inchesToMeters(2));
    
      return wheelVelocity; // In Meters per second
    } 
    
    public static double nativeUnitsToDistanceMeters(double sensorCounts){ 
    
      double motorRotations = (double) sensorCounts / 2048; 
    
      double wheelRotations = motorRotations / DrivetrainSubsystem.kGearRatio; 
    
      double positionMeters = wheelRotations * (2 * Math.PI * Units.inchesToMeters(2)); 
    
      return positionMeters;
    } 

      public static double getAverageEncoderCount(){  
        return (leftM.getSelectedSensorPosition() + rightM.getSelectedSensorPosition() ) / 2.0;  //Might not be + 
      } 
    
      /**
       * Gets the average distance of the two encoders in METERS.
       *
       * @return the average of the two encoder readings in METERS
       */
      public static double getAverageEncoderDistance() {
        return Odometry.nativeUnitsToDistanceMeters(getAverageEncoderCount());
      }

}
