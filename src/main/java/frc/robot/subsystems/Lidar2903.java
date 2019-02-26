package frc.robot.subsystems;

import edu.wpi.first.wpilibj.SerialPort;

public class Lidar2903 {

  public SerialPort arduino;
  int leftDistance = 0;
  int leftStatus = 0;
  int centerDistance = 0;
  int centerStatus = 0;
  int rightDistance = 0;
  int rightStatus = 0;

  public Lidar2903() {
    try {
      arduino = new SerialPort(9600, SerialPort.Port.kUSB);
    } catch (Exception ex) { }
  }

  public void updateData() {
    String dist = arduino.readString();
    String[] distLines = dist.split("\r\n");
    String[] actuallyImportant = (distLines.length >= 7)
        ? new String[] { distLines[distLines.length - 1], distLines[distLines.length - 2],
            distLines[distLines.length - 3], distLines[distLines.length - 4], distLines[distLines.length - 5],
            distLines[distLines.length - 6], distLines[distLines.length - 7], }
        : distLines;

    for (String line : actuallyImportant)
      try {
        if (!line.isBlank() && !line.isEmpty()) {
          if (line.startsWith("D1:") && line.contains("!"))
            leftDistance = Integer.parseInt(line.substring(3, line.indexOf("!")));
          else if (line.startsWith("D2:") && line.contains("!"))
            centerDistance = Integer.parseInt(line.substring(3, line.indexOf("!")));
          else if (line.startsWith("D3:") && line.contains("!"))
            rightDistance = Integer.parseInt(line.substring(3, line.indexOf("!")));
          else if (line.startsWith("S1:") && line.contains("!"))
            leftStatus = Integer.parseInt(line.substring(3, line.indexOf("!")));
          else if (line.startsWith("S2:") && line.contains("!"))
            centerStatus = Integer.parseInt(line.substring(3, line.indexOf("!")));
          else if (line.startsWith("S3:") && line.contains("!"))
            rightStatus = Integer.parseInt(line.substring(3, line.indexOf("!")));
        }
      } catch (Exception ex) {
      }
  }

  public enum LidarPosition {
    Left, Center, Right
  }

  public int getDistance(LidarPosition pos) {
    updateData();
    if (pos == LidarPosition.Left)
      return leftDistance;
    else if (pos == LidarPosition.Center)
      return centerDistance;
    else if (pos == LidarPosition.Right)
      return rightDistance;
    else
      return 0;
  }

  public int getStatus(LidarPosition pos) {
    updateData();
    if (pos == LidarPosition.Left)
      return leftStatus;
    else if (pos == LidarPosition.Center)
      return centerStatus;
    else if (pos == LidarPosition.Right)
      return rightStatus;
    else
      return 0;
  }

  public void initDefaultCommand() {
    // Set the default command for a subsystem here.
    // setDefaultCommand(new MySpecialCommand());
  }
}
