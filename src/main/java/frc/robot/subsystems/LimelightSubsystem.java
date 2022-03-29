// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class LimelightSubsystem extends SubsystemBase {
  /** Creates a new LimelightSubsystem. */

  private static LimelightSubsystem instance;
  private final NetworkTable table = NetworkTableInstance.getDefault().getTable("limelight");

  double tx = table.getEntry("tx").getDouble(0.0);
  double ty = table.getEntry("ty").getDouble(0.0);
  double tv = table.getEntry("tv").getDouble(0.0);

  public LimelightSubsystem() {}

  public static synchronized LimelightSubsystem getInstance(){
    if(instance == null){
      instance = new LimelightSubsystem();
    }
    return instance;
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
