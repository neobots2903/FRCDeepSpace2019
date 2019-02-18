package frc.robot.subsystems;

import edu.wpi.first.wpilibj.SerialPort;

public class Lidar2903{

  public SerialPort arduino;
  int distance1 = 0;
  int status1 = 0;
  int distance2 = 0;
  int status2 = 0;
  int distance3 = 0;
  int status3 = 0;
      
  public Lidar2903() {  
    arduino = new SerialPort(9600,SerialPort.Port.kUSB);
  }

  public void updateData() {
    String dist = arduino.readString();
    String[] distLines = dist.split("\r\n");
    String[] actuallyImportant = (distLines.length >= 7) ? 
    new String[] {
      distLines[distLines.length-1],
      distLines[distLines.length-2],
      distLines[distLines.length-3],
      distLines[distLines.length-4],
      distLines[distLines.length-5],
      distLines[distLines.length-6],
      distLines[distLines.length-7],
    } :
    distLines;
    
    for(String line : actuallyImportant)
    try {
      if (!line.isBlank() && !line.isEmpty()) {
        if (line.startsWith("D1:") && line.contains("!"))
        distance1 = Integer.parseInt(line.substring(3,line.indexOf("!")));
        else if (line.startsWith("D2:") && line.contains("!"))
        distance2 = Integer.parseInt(line.substring(3,line.indexOf("!")));
        else if (line.startsWith("D3:") && line.contains("!"))
        distance3 = Integer.parseInt(line.substring(3,line.indexOf("!")));
        else if (line.startsWith("S1:") && line.contains("!"))
        status1 = Integer.parseInt(line.substring(3,line.indexOf("!")));
        else if (line.startsWith("S2:") && line.contains("!"))
        status2 = Integer.parseInt(line.substring(3,line.indexOf("!")));
        else if (line.startsWith("S3:") && line.contains("!"))
        status3 = Integer.parseInt(line.substring(3,line.indexOf("!")));
      }
    } catch (Exception ex) {}
  }

  public enum LidarPosition {
    Left,
    Center,
    Right
  }

  public int getDistance(LidarPosition pos) {
    updateData();
    if (pos == LidarPosition.Left)
      return distance1;
    else if (pos == LidarPosition.Center)
      return distance2;
    else if (pos == LidarPosition.Right)
      return distance3;
    else
      return 0;
  }

  public int getStatus(LidarPosition pos) {
    updateData();
    if (pos == LidarPosition.Left)
      return status1;
    else if (pos == LidarPosition.Center)
      return status2;
    else if (pos == LidarPosition.Right)
      return status3;
    else
      return 0;
  }

  public void initDefaultCommand() {
      // Set the default command for a subsystem here.
      //setDefaultCommand(new MySpecialCommand());
  }
}
