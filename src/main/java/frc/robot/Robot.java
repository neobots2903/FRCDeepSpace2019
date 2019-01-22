/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.PIDController;
import edu.wpi.first.wpilibj.PIDOutput;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.command.Scheduler;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.subsystems.ExampleSubsystem;
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
  public static NavX2903 navXSubsystem;

  public static PIDController gyroController;
  public static PIDController visionController;

  public static PIDOutput visionOutput;
  public static PIDOutput gyroOutput;

  public static LineSensor2903 lineSubsystem;
  public static Limelight2903 limelightSubsystem;

  public static AHRS ahrs;

  public static Joystick driveJoy;
  public static Joystick opJoy;

  public static final double vKP = 0.03;
  public static final double vKI = 0;
  public static final double vKD = 0;
  public static final double vKF = 0;
  
  public static final double kP = 0.03;
  public static final double kI = 0;
  public static final double kD = 0;
  public static final double kF = 0;
  public static final double kToleranceDegrees = 1.0;

  public static double gyroPIDTurn = 0;
  public static double visionPIDTurn = 0;

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
    navXSubsystem = new NavX2903();
    teleopCommand = new TeleOp();
    //lineSubsystem = new LineSensor2903();
    limelightSubsystem = new Limelight2903();

    visionOutput = new VisionPIDOutput();
    gyroOutput = new GyroPIDOutput();

    driveSubsystem.init();
    //lineSubsystem.init();
    limelightSubsystem.init();

    try {
      ahrs = new AHRS(SPI.Port.kMXP);
    } catch (RuntimeException ex) {
      DriverStation.reportError("Error instantiating navX MXP:  " + ex.getMessage(), true);
    }
   
    visionController = new PIDController(vKP, vKI, vKD, vKF, limelightSubsystem, visionOutput);
    visionController.setInputRange(-27f, 27f);
    visionController.setOutputRange(-1.0, 1.0);
    visionController.setAbsoluteTolerance(kToleranceDegrees);
    visionController.setContinuous(true);

    gyroController = new PIDController(kP,kI,kD,kF,ahrs,gyroOutput);
    gyroController.setInputRange(-180.0f, 180.0f);
    gyroController.setOutputRange(-1.0, 1.0);
    gyroController.setAbsoluteTolerance(kToleranceDegrees);
    gyroController.setContinuous(true);

    SmartDashboard.putNumber("kP",kP);
    SmartDashboard.putNumber("kI",kI);
    SmartDashboard.putNumber("kD",kD);
    SmartDashboard.putNumber("kF",kF);

    driveJoy = new Joystick(RobotMap.DriveJoy);
    opJoy = new Joystick(RobotMap.OpJoy);

    m_chooser.setDefaultOption("Default Auto", new Autonomous());
    m_chooser.addOption("Cargo Vision Test", new CargoVisionTest());
    m_chooser.addOption("Target Vision Test", new TargetVisionTest());
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
    Scheduler.getInstance().run();
  }

 class VisionPIDOutput implements PIDOutput {

    @Override
    public void pidWrite(double output) {
      Robot.visionPIDTurn = output;
    }
}

 class GyroPIDOutput implements PIDOutput {

  @Override
  public void pidWrite(double output) {
    Robot.gyroPIDTurn = output;
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
