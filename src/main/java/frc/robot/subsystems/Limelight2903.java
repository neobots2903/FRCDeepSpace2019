/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.PIDSource;
import edu.wpi.first.wpilibj.PIDSourceType;
import edu.wpi.first.wpilibj.command.Subsystem;
import edu.wpi.first.networktables.NetworkTable;

/**
 * An example subsystem.  You can replace me with your own Subsystem.
 */
public class Limelight2903 extends Subsystem implements PIDSource {
  // Put methods for controlling this subsystem
  // here. Call these from Commands.
  NetworkTable table;
  public NetworkTableEntry tx; //Horizontal Offset From Crosshair To Target (-27 degrees to 27 degrees)
  public NetworkTableEntry ta; //Target Area (0% of image to 100% of image)
  public NetworkTableEntry tv; //Whether the limelight has any valid targets (0 or 1)

  @Override
  public void initDefaultCommand() {
    // Set the default command for a subsystem here.
    // setDefaultCommand(new MySpecialCommand());
  }

  public void init() {
    table = NetworkTableInstance.getDefault().getTable("limelight");
    tx = table.getEntry("tx");
    ta = table.getEntry("ta");
    tv = table.getEntry("tv");
    setLight(false);
  }

  public void setCargoMode() {
    setLight(false);
    table.getEntry("pipeline").setNumber(1); // sets cargo pipeline
  }

  public void setTargetMode() {
    setLight(true);
    table.getEntry("pipeline").setNumber(0); // sets vision target pipeline
  }

  public void setLight(boolean state) {
    table.getEntry("ledMode").setNumber((state) ? 3 : 1); // forces LED off
  }

  double getEntryValue(NetworkTableEntry entry) {
    return entry.getNumber(0).doubleValue();
  }

  public double getTX(){
    return getEntryValue(tx);
  }

  public double getTA(){
    return getEntryValue(ta);
  }

  @Override
  public void setPIDSourceType(PIDSourceType pidSource) {
  }

  @Override
  public PIDSourceType getPIDSourceType() {
    return PIDSourceType.kDisplacement;
  }

  @Override
  public double pidGet() {
    return getTX();
}
}
