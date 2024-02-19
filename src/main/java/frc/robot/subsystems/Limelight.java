// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.Optional;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.LimelightHelpers;

public class Limelight extends SubsystemBase {
  public static boolean tv;
  public static double tx;
  

  /** Creates a new Limelight. */
  public Limelight() {
    System.out.println("----------------------------Limelight Subsystem------------------------------");
    Optional<Alliance> alliance = DriverStation.getAlliance();
    if (alliance.isPresent()) {
      if (alliance.get() == Alliance.Red) {
        LimelightHelpers.setPipelineIndex("limelight", 1);
      }
      if (alliance.get() == Alliance.Blue){
        LimelightHelpers.setPipelineIndex("limelight", 0);
      }
    }
  }


  public static double FindDistance(){
      double ty = LimelightHelpers.getTY("");
      double limelightMountHeight = Constants.LimelightConstants.limelightMoundHeight;
      double limelightMountPitch = Constants.LimelightConstants.limelightMountPitch;

      double angleToGoalDegrees = limelightMountPitch + ty;
      double angleToGoalRadians = angleToGoalDegrees * (3.14159 / 180.0);

      double distanceFromTarget = (57.13 - limelightMountHeight) / Math.tan(angleToGoalRadians);
      SmartDashboard.putNumber("Li DistanceFromTarget", distanceFromTarget);

      return distanceFromTarget;
  }

  public static double txSlowly(){
    double TX = tx * 0.1;
    return TX;
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    tx = LimelightHelpers.getTX("");
    tv = LimelightHelpers.getTV("");
SmartDashboard.putBoolean("Apriltag Visible?", tv);
SmartDashboard.putNumber("X Offset", tx);
  }
}
