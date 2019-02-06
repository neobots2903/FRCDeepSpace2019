package frc.robot.subsystems;

import edu.wpi.first.wpilibj.SerialPort;

public class Lidar2903{

  public SerialPort arduino;
  int lastDistance = 0;
  int lastOutlier = 0;
  int lastStatus = 0;
      
  public Lidar2903() {  
    arduino = new SerialPort(9600,SerialPort.Port.kUSB);
  }

  public void updateData() {
    String dist = arduino.readString();
    String[] distLines = dist.split("\r\n");
    String[] actuallyImportant = (distLines.length >= 3) ? 
    new String[] {
      distLines[distLines.length-1],
      distLines[distLines.length-2],
      distLines[distLines.length-3],
    } :
    distLines;
    
    for(String line : actuallyImportant)
    try {
      if (!line.isBlank() && !line.isEmpty()) {
        if (line.startsWith("D:") && line.contains("!"))
        lastDistance = Integer.parseInt(line.substring(2,line.indexOf("!")));
        else if (line.startsWith("S:") && line.contains("!"))
        lastStatus = Integer.parseInt(line.substring(2,line.indexOf("!")));
      }
    } catch (Exception ex) {}
  }

  public int getDistance() {
    updateData();
    return lastDistance;
  }

  public int getStatus() {
    updateData();
    return lastStatus;
  }

  public void initDefaultCommand() {
      // Set the default command for a subsystem here.
      //setDefaultCommand(new MySpecialCommand());
  }
}
