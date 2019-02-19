/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.command.Subsystem;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.networktables.NetworkTable;

/**
 * An example subsystem. You can replace me with your own Subsystem.
 */
public class Limelight2903 extends Subsystem {
  // Put methods for controlling this subsystem
  // here. Call these from Commands.
  NetworkTable table;
  public NetworkTableEntry tx; // Horizontal Offset From Crosshair To Target (-27 degrees to 27 degrees)
  public NetworkTableEntry ta; // Target Area (0% of image to 100% of image)
  public NetworkTableEntry tv; // Whether the limelight has any valid targets (0 or 1)
  public NetworkTableEntry ts; // Skew or rotation (-90 degrees to 0 degrees)

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
    ts = table.getEntry("ts");
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

  double getEntryDouble(NetworkTableEntry entry) {
    return entry.getDouble(0);
  }

  double[] getEntryArray(NetworkTableEntry entry) {
    return entry.getDoubleArray(new double[7]);
  }

  public double getTS() {
    double value = getEntryDouble(ts);
    if (value < -45) value += 90;
    SmartDashboard.putNumber("TS", value);
    return value;
  }

  public double getTX() {
    double value = getEntryDouble(tx);
    SmartDashboard.putNumber("TX", value);
    return value;
  }

  public double getTV() {
    double value = getEntryDouble(tv);
    SmartDashboard.putNumber("TV", value);
    return value;
  }

  public double getTA() {
    double value = getEntryDouble(ta);
    SmartDashboard.putNumber("TA", value);
    return value;
  }
}
