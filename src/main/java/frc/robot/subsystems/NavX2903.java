/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.command.Subsystem;
import frc.robot.Robot;

/**
 * Super intense gyro thing
 */
public class NavX2903 extends Subsystem {
  // Put methods for controlling this subsystem
  // here. Call these from Commands.
  private boolean collisionDetected = false;
  private double last_world_linear_accel_x;
  private double last_world_linear_accel_y;
  
  final static double kCollisionThreshold_DeltaG = 0.5f;

  @Override
  public void initDefaultCommand() {
    // Set the default command for a subsystem here.
    // setDefaultCommand(new MySpecialCommand());
  }

  public boolean isColliding() {
    collisionDetected = false;
    double curr_world_linear_accel_x = Robot.ahrs.getWorldLinearAccelX();
    double currentJerkX = curr_world_linear_accel_x - last_world_linear_accel_x;
    last_world_linear_accel_x = curr_world_linear_accel_x;
    double curr_world_linear_accel_y = Robot.ahrs.getWorldLinearAccelY();
    double currentJerkY = curr_world_linear_accel_y - last_world_linear_accel_y;
    last_world_linear_accel_y = curr_world_linear_accel_y;

    if ((Math.abs(currentJerkX) > kCollisionThreshold_DeltaG)
        || (Math.abs(currentJerkY) > kCollisionThreshold_DeltaG)) {
      collisionDetected = true;
    }
    return collisionDetected;
  }

}
