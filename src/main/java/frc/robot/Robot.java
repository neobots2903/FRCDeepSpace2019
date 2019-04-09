/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import com.kauailabs.navx.frc.AHRS;

import edu.wpi.cscore.UsbCamera;
import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.PIDController;
import edu.wpi.first.wpilibj.PIDOutput;
import edu.wpi.first.wpilibj.PIDSource;
import edu.wpi.first.wpilibj.PIDSourceType;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.command.Scheduler;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.subsystems.ExampleSubsystem;
import frc.robot.subsystems.Lidar2903.LidarPosition;
import frc.robot.commands.*;
import frc.robot.subsystems.*;

/**
 * The VM is configured to automatically run this class, and to call the
 * functions corresponding to each mode, as described in the TimedRobot
 * documentation. If you change the name of this class or the package after
 * creating this project, you must also update the build.gradle file in the
 * project.
 */
public class Robot extends TimedRobot {
  public static ExampleSubsystem m_subsystem = new ExampleSubsystem();
  public static OI m_oi;

  public static TeleOp teleopCommand;
  public static Drive2903 driveSubsystem;
  public static SwerveDrive2903 swerveDriveSubsystem;
  public static NavX2903 navXSubsystem;

  public static PIDController gyroController;
  public static PIDController visionTurnController;
  public static PIDController visionStrafeController;
  public static PIDController dartController;
  public static PIDController wristController;
  public static PIDController lidarController;

  public static PIDSource visionTurnSource;
  public static PIDSource visionStrafeSource;
  public static PIDSource dartSource;
  public static PIDSource wristSource;
  public static PIDSource lidarSource;

  public static PIDOutput visionTurnOutput;
  public static PIDOutput visionStrafeOutput;
  public static PIDOutput gyroOutput;
  public static PIDOutput dartOutput;
  public static PIDOutput wristOutput;
  public static PIDOutput lidarOutput;

  public static LineSensor2903 lineSubsystem;
  public static Lidar2903 lidarSubsystem;
  public static Limelight2903 limelightSubsystem;
  public static PickUpArm2903 pickUpArmSubsystem;
  public static Ramp2903 rampSubsystem;

  public static AHRS ahrs;

  public static Joystick driveJoy;
  public static Joystick opJoy;

  public static class PIDF {
    public double vkP;
    public double vkI;
    public double vkD;
    public double vkF;

    public PIDF(double p, double i, double d, double f) {
      vkP = p;
      vkI = i;
      vkD = d;
      vkF = f;
    }
  }

  public static PIDF visionTurnPIDF = new PIDF(0.5, 0, 0, 0);
  public static PIDF visionStrafePIDF = new PIDF(0.1, 0, 0, 0);
  public static PIDF dartPIDF = new PIDF(0.006, 0, 0, 0); //P = 0.003
  public static PIDF wristPIDF = new PIDF(0.0005, 0, 0, 0);
  public static PIDF gyroPIDF = new PIDF(0.08, 0, 0, 0);
  public static PIDF lidarPIDF = new PIDF(0.08, 0, 0, 0);

  public static final double kToleranceDegrees = 1.0;

  public static double gyroPIDTurn = 0;
  public static double visionTurnValue = 0;
  public static double visionStrafeValue = 0;
  public static double lidarTurnValue = 0;
  public static double dartValue = 0;
  public static double wristValue = 0;

  public static final int IMG_WIDTH = 640;
  public static final int IMG_HEIGHT = 480;
  public static CameraServer cserver = CameraServer.getInstance();
  public static UsbCamera camera;

  Command m_autonomousCommand;
  SendableChooser<Command> m_chooser = new SendableChooser<>();

  /**
   * This function is run when the robot is first started up and should be used
   * for any initialization code.
   */
  @Override
  public void robotInit() {
    m_oi = new OI();
    driveSubsystem = new Drive2903();
    swerveDriveSubsystem = new SwerveDrive2903();
    navXSubsystem = new NavX2903();
    teleopCommand = new TeleOp();

    lidarSubsystem = new Lidar2903();
    lineSubsystem = new LineSensor2903();
    limelightSubsystem = new Limelight2903();
    pickUpArmSubsystem = new PickUpArm2903();
    rampSubsystem = new Ramp2903();

    visionTurnSource = new VisionTurnSource();
    visionStrafeSource = new VisionStrafeSource();
    dartSource = new DartSource();
    wristSource = new WristSource();
    lidarSource = new LidarSource();

    visionTurnOutput = new VisionTurnOutput();
    visionStrafeOutput = new VisionStrafeOutput();
    gyroOutput = new GyroPIDOutput();
    dartOutput = new DartOutput();
    wristOutput = new WristOutput();
    lidarOutput = new LidarPIDOutput();

    driveSubsystem.init();
    lineSubsystem.init();
    limelightSubsystem.init();

    camera = cserver.startAutomaticCapture();
    camera.setResolution(320, 240);

    try {
      ahrs = new AHRS(SPI.Port.kMXP);
    } catch (RuntimeException ex) {
      DriverStation.reportError("Error instantiating navX MXP:  " + ex.getMessage(), true);
    }

    visionTurnController = new PIDController(visionTurnPIDF.vkP, visionTurnPIDF.vkI, visionTurnPIDF.vkD,
        visionTurnPIDF.vkF, visionTurnSource, visionTurnOutput);
    visionTurnController.setInputRange(-45f, 45f);
    visionTurnController.setOutputRange(-1.0, 1.0);
    visionTurnController.setAbsoluteTolerance(kToleranceDegrees);
    visionTurnController.setContinuous(false);

    visionStrafeController = new PIDController(visionStrafePIDF.vkP, visionStrafePIDF.vkI, visionStrafePIDF.vkD,
        visionStrafePIDF.vkF, visionStrafeSource, visionStrafeOutput);
    visionTurnController.setInputRange(-27f, 27f);
    visionTurnController.setOutputRange(-1.0, 1.0);
    visionTurnController.setAbsoluteTolerance(kToleranceDegrees);
    visionTurnController.setContinuous(false);

    gyroController = new PIDController(gyroPIDF.vkP, gyroPIDF.vkI, gyroPIDF.vkD, gyroPIDF.vkF, ahrs, gyroOutput);
    gyroController.setInputRange(-180.0f, 180.0f);
    gyroController.setOutputRange(-1.0, 1.0);
    gyroController.setAbsoluteTolerance(kToleranceDegrees);
    gyroController.setContinuous(true);

    dartController = new PIDController(dartPIDF.vkP, dartPIDF.vkI, dartPIDF.vkD,dartPIDF.vkF,
    dartSource,dartOutput);
    gyroController.setInputRange(-3000.0f, 0.0f);
    gyroController.setOutputRange(-1.0, 1.0);
    gyroController.setAbsoluteTolerance(kToleranceDegrees);
    gyroController.setContinuous(false);

    wristController = new PIDController(wristPIDF.vkP, wristPIDF.vkI, wristPIDF.vkD,wristPIDF.vkF,
    wristSource,wristOutput);
    gyroController.setInputRange(-5000.0f, 0.0f);
    gyroController.setOutputRange(-0.70, 0.70);
    gyroController.setAbsoluteTolerance(kToleranceDegrees);
    gyroController.setContinuous(false);

    lidarController = new PIDController(lidarPIDF.vkP, lidarPIDF.vkI, lidarPIDF.vkD,lidarPIDF.vkF,
    lidarSource,lidarOutput);
    gyroController.setInputRange(-180.0f, 180.0f);
    gyroController.setOutputRange(-1.0, 1.0);
    gyroController.setAbsoluteTolerance(kToleranceDegrees);
    gyroController.setContinuous(false);

    visionTurnController.setSetpoint(0);
    visionStrafeController.setSetpoint(0);
    dartController.setSetpoint(pickUpArmSubsystem.getElbow());
    wristController.setSetpoint(pickUpArmSubsystem.getWrist());
    //lidarController.setSetpoint(0);

    SmartDashboard.putNumber("VisTurn kP", visionTurnPIDF.vkP);
    SmartDashboard.putNumber("VisTurn kI", visionTurnPIDF.vkI);
    SmartDashboard.putNumber("VisTurn kD", visionTurnPIDF.vkD);
    SmartDashboard.putNumber("VisTurn kF", visionTurnPIDF.vkF);

    SmartDashboard.putNumber("VisStrf kP", visionStrafePIDF.vkP);
    SmartDashboard.putNumber("VisStrf kI", visionStrafePIDF.vkI);
    SmartDashboard.putNumber("VisStrf kD", visionStrafePIDF.vkD);
    SmartDashboard.putNumber("VisStrf kF", visionStrafePIDF.vkF);

    SmartDashboard.putNumber("Gyro kP", gyroPIDF.vkP);
    SmartDashboard.putNumber("Gyro kI", gyroPIDF.vkI);
    SmartDashboard.putNumber("Gyro kD", gyroPIDF.vkD);
    SmartDashboard.putNumber("Gyro kF", gyroPIDF.vkF);

    SmartDashboard.putNumber("Lidar kP", lidarPIDF.vkP);
    SmartDashboard.putNumber("Lidar kI", lidarPIDF.vkI);
    SmartDashboard.putNumber("Lidar kD", lidarPIDF.vkD);
    SmartDashboard.putNumber("Lidar kF", lidarPIDF.vkF);

    driveJoy = new Joystick(RobotMap.DriveJoy);
    opJoy = new Joystick(RobotMap.OpJoy);

    m_chooser.setDefaultOption("Drive Straight", new Autonomous());
    m_chooser.addOption("Vision Auto Align", new VisionAutoAlign());
    m_chooser.addOption("Cargo Vision Test", new CargoVisionTest());
    m_chooser.addOption("Gyro Test", new GyroTest());
    m_chooser.addOption("Line Follower", new LineFollower());
    SmartDashboard.putData("Auto mode", m_chooser);
  }

  /**
   * This function is called every robot packet, no matter the mode. Use this for
   * items like diagnostics that you want ran during disabled, autonomous,
   * teleoperated and test.
   *
   * <p>
   * This runs after the mode specific periodic functions, but before LiveWindow
   * and SmartDashboard integrated updating.
   */
  @Override
  public void robotPeriodic() {
  }

  /**
   * This function is called once each time the robot enters Disabled mode. You
   * can use it to reset any subsystem information you want to clear when the
   * robot is disabled.
   */
  @Override
  public void disabledInit() {
  }

  @Override
  public void disabledPeriodic() {
    Scheduler.getInstance().run();
  }

  /**
   * This autonomous (along with the chooser code above) shows how to select
   * between different autonomous modes using the dashboard. The sendable chooser
   * code works with the Java SmartDashboard. If you prefer the LabVIEW Dashboard,
   * remove all of the chooser code and uncomment the getString code to get the
   * auto name from the text box below the Gyro
   *
   * <p>
   * You can add additional auto modes by adding additional commands to the
   * chooser code above (like the commented example) or additional comparisons to
   * the switch structure below with additional strings & commands.
   */
  @Override
  public void autonomousInit() {
    m_autonomousCommand = m_chooser.getSelected();

    /*
     * String autoSelected = SmartDashboard.getString("Auto Selector", "Default");
     * switch(autoSelected) { case "My Auto": autonomousCommand = new
     * MyAutoCommand(); break; case "Default Auto": default: autonomousCommand = new
     * ExampleCommand(); break; }
     */

    // schedule the autonomous command (example)
    if (m_autonomousCommand != null) {
      m_autonomousCommand.start();
    }
  }

  /**
   * This function is called periodically during autonomous.
   */
  @Override
  public void autonomousPeriodic() {
    //Scheduler.getInstance().run();
    Scheduler.getInstance().run();
    teleopCommand.start();
  }

  class VisionTurnSource implements PIDSource {

    @Override
    public void setPIDSourceType(PIDSourceType pidSource) {
    }

    @Override
    public PIDSourceType getPIDSourceType() {
      return PIDSourceType.kDisplacement;
    }

    @Override
    public double pidGet() {
      return limelightSubsystem.getTS();
    }
  }

  class VisionStrafeSource implements PIDSource {

    @Override
    public void setPIDSourceType(PIDSourceType pidSource) {
    }

    @Override
    public PIDSourceType getPIDSourceType() {
      return PIDSourceType.kDisplacement;
    }

    @Override
    public double pidGet() {
      return limelightSubsystem.getTX();
    }
  }

  class DartSource implements PIDSource {

    @Override
    public void setPIDSourceType(PIDSourceType pidSource) {
    }

    @Override
    public PIDSourceType getPIDSourceType() {
      return PIDSourceType.kDisplacement;
    }

    @Override
    public double pidGet() {
      return pickUpArmSubsystem.getElbow();
    }
  }

  class WristSource implements PIDSource {

    @Override
    public void setPIDSourceType(PIDSourceType pidSource) {
    }

    @Override
    public PIDSourceType getPIDSourceType() {
      return PIDSourceType.kDisplacement;
    }

    @Override
    public double pidGet() {
      return pickUpArmSubsystem.getWrist();
    }
  }

  class LidarSource implements PIDSource {

    @Override
    public void setPIDSourceType(PIDSourceType pidSource) {
    }

    @Override
    public PIDSourceType getPIDSourceType() {
      return PIDSourceType.kDisplacement;
    }

    @Override
    public double pidGet() {
      return lidarSubsystem.getDistance(LidarPosition.Left) - lidarSubsystem.getDistance(LidarPosition.Right);
    }
  }

 class VisionTurnOutput implements PIDOutput {

    @Override
    public void pidWrite(double output) {
      Robot.visionTurnValue = output;
    }
  }

  class VisionStrafeOutput implements PIDOutput {

    @Override
    public void pidWrite(double output) {
      Robot.visionStrafeValue = output;
    }
  }

  class GyroPIDOutput implements PIDOutput {

    @Override
    public void pidWrite(double output) {
      Robot.gyroPIDTurn = output;
    }
  }

  class DartOutput implements PIDOutput {

    @Override
    public void pidWrite(double output) {
      Robot.dartValue = output;
    }
  }

  class WristOutput implements PIDOutput {

    @Override
    public void pidWrite(double output) {
      Robot.wristValue = output;
    }
  }

class LidarPIDOutput implements PIDOutput {

  @Override
  public void pidWrite(double output) {
    Robot.lidarTurnValue = output;
  }
}

  @Override
  public void teleopInit() {
    // This makes sure that the autonomous stops running when
    // teleop starts running. If you want the autonomous to
    // continue until interrupted by another command, remove
    // this line or comment it out.
    if (m_autonomousCommand != null) {
      m_autonomousCommand.cancel();
    }
    if (teleopCommand != null) {
      teleopCommand.cancel();
    }
  }

  /**
   * This function is called periodically during operator control.
   */
  @Override
  public void teleopPeriodic() {
    Scheduler.getInstance().run();
    teleopCommand.start();
  }

  /**
   * This function is called periodically during test mode.
   */
  @Override
  public void testPeriodic() {
  }
}
