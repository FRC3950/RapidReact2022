// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.misc.DashboardSettings;

public class LimelightSubsystem extends SubsystemBase {
  /** Creates a new LimelightSubsystem. */

  private final NetworkTable table = NetworkTableInstance.getDefault().getTable("limelight");

  double tx;
  double ty;
  int tv;

  public LimelightSubsystem() {}

  @Override
  public void periodic() {
    tx = table.getEntry("tx").getDouble(0.0);
    ty = table.getEntry("ty").getDouble(0.0);
    tv = (int) table.getEntry("tv").getDouble(0.0);

    if(true){
      SmartDashboard.putNumber("tx", tx);
      SmartDashboard.putNumber("ty", ty);
      SmartDashboard.putNumber("tv", tv);
    }
  }


  public double getHorizOffset(){
    return tx;
  }

  public double getVertOffset(){
    return ty;
  }
  
  public double[] getTargetVelocities(){
    if(hasTarget()){
      return new double[] {
        (-63.02 ) * getVertOffset() + 12870,
        (-212.7) * getVertOffset() + 14970
      }; 
    }
    else {
      return new double[] {0,0};
    }
  }

  public boolean isWithinRange(){
    return (tx < 0.55 && tx > -0.55);
  }
  
  //TODO: set velocities in initialize
  public double[] lookupTable(){
    if(getVertOffset() <= 20.5 && getVertOffset() >= 17){
      return new double[] {
        11027 + 600, 9990 + 600
      };
    }
    else {
      return new double[] {
        11027 + 600, 9990 + 600
      };
    }
  }
  public boolean hasTarget(){
    return (tv == 1);
  }
}
