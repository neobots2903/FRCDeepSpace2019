package frc.robot.subsystems;

import frc.robot.Robot;
import frc.robot.RobotMap;

import edu.wpi.first.wpilibj.command.Subsystem;
import edu.wpi.first.wpilibj.interfaces.Potentiometer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.AnalogPotentiometer;
import edu.wpi.first.wpilibj.DigitalInput;



public class PickUpArm2903 extends Subsystem implements Runnable{
    double speed = 0.5;
    double elbowError = 2;
    private TalonSRX leftIntakeMotor;
    private TalonSRX rightIntakeMotor;
    private TalonSRX wristMotor;
    private TalonSRX elbowMotor;
    public Solenoid panelRetract;
    public Solenoid panelEject;
    public Potentiometer elbowPotentiometer;
    public DigitalInput topHall;
    public DigitalInput bottomHall;


    //THESE ARE NOT SET! REEEEEEE
    final int ELBOW_MAX = 10;
    final int ELBOW_MIN = 0;
    final int WRIST_MAX = 999999999;
    final int WRIST_MIN = -999999999;

    final int ELBOW_TOP = 0;
    final int WRIST_TOP = 0;

    final int ELBOW_MIDDLE = 0;
    final int WRIST_MIDDLE = 0;

    final int ELBOW_BOTTOM = 0;
    final int WRIST_BOTTOM = 0;

    final int ELBOW_FLOOR = 0;
    final int WRIST_FLOOR = 0;

    @Override
    protected void initDefaultCommand() {
    }
    public PickUpArm2903(){
        Thread thread = new Thread(this);
        elbowPotentiometer = new AnalogPotentiometer(0, 360, 30);
        leftIntakeMotor = new TalonSRX(RobotMap.TBD);
        rightIntakeMotor = new TalonSRX(RobotMap.TBD);
        wristMotor = new TalonSRX(RobotMap.WristMotor);
        wristMotor.configSelectedFeedbackSensor(FeedbackDevice.PulseWidthEncodedPosition, 0, 10);
        elbowMotor = new TalonSRX(RobotMap.DartMotor);
        panelRetract = new Solenoid(RobotMap.panelRetract);
        panelEject = new Solenoid(RobotMap.panelEject);
        topHall = new DigitalInput(RobotMap.upperHall);
        bottomHall = new DigitalInput(RobotMap.bottomHall);
        thread.start();
    }

    public void TakeIn() {
        leftIntakeMotor.set(ControlMode.PercentOutput ,speed);
        rightIntakeMotor.set(ControlMode.PercentOutput ,speed);
    }
    public void TakeOut() {
        leftIntakeMotor.set(ControlMode.PercentOutput ,-speed);
        rightIntakeMotor.set(ControlMode.PercentOutput ,-speed);
    }
    public void TakeStop() {
        leftIntakeMotor.set(ControlMode.PercentOutput ,0);
        rightIntakeMotor.set(ControlMode.Position ,0);
    }
    public void WristSet(int targetPos){
        if(targetPos > WRIST_MAX)
            targetPos = WRIST_MAX;
        else if(targetPos < WRIST_MIN)
            targetPos = WRIST_MIN;
        wristMotor.set(ControlMode.Position, targetPos);
        SmartDashboard.putNumber("Wrist Encoder Value:", wristMotor.getSelectedSensorPosition());
    }

    public void WristSetHa(double speed) {
        wristMotor.set(ControlMode.PercentOutput, speed);
        SmartDashboard.putNumber("Wrist Encoder Value:", wristMotor.getSelectedSensorPosition());
    }
    
  /*  public void WristUp(){
        wristMotor.set(ControlMode.PercentOutput ,speed);
        SmartDashboard.putNumber("Wrist Encoder Value:", wristMotor.getSelectedSensorPosition());
    }
    public void WristDown(){
        wristMotor.set(ControlMode.PercentOutput ,-speed);
        SmartDashboard.putNumber("Wrist Encoder Value:", wristMotor.getSelectedSensorPosition());
    } */
    public void WristStop(){
        wristMotor.set(ControlMode.PercentOutput, 0);
    }

    public double getElbow() {
    return elbowPotentiometer.get();
    }
    
    public void ElbowSet(){
        if (topHall.get() && Robot.dartValue > 0) return;
        if (bottomHall.get() && Robot.dartValue < 0) return;
        elbowMotor.set(ControlMode.PercentOutput, Robot.dartValue);
        SmartDashboard.putNumber("Elbow Encoder Value:", getElbow());
    }

    public void ElbowSetHa(double speed) {
        if (!topHall.get() && speed > 0) return;
        if (!bottomHall.get() && speed < 0) return;
        elbowMotor.set(ControlMode.PercentOutput, speed);
        SmartDashboard.putNumber("Elbow Encoder Value:", getElbow());
    }

    public void SetElbowTarget(int t){
        if(t > ELBOW_MAX)
            Robot.dartController.setSetpoint(ELBOW_MAX);
        else if(t < ELBOW_MIN)
            Robot.dartController.setSetpoint(ELBOW_MIN);
        else 
            Robot.dartController.setSetpoint(t);
    }

   /* public void ElbowUp(){ 
        elbowMotor.set(ControlMode.PercentOutput ,speed);
        SmartDashboard.putNumber("Potentiometer Value:",elbowPotentiometer.getValue());
    }
    public void ElbowDown(){
        elbowMotor.set(ControlMode.PercentOutput ,-speed);
        SmartDashboard.putNumber("Potentiometer Value:",elbowPotentiometer.getValue());
    } */
    public void ElbowStop(){
        elbowMotor.set(ControlMode.PercentOutput, 0);
    }
    public void punch(){
        eject();
        try {
            Thread.sleep(100);
        } catch (InterruptedException e) {
            e.printStackTrace();
        }
        retract();
    }

    public void retract() {
        panelRetract.set(false);
        panelEject.set(true);
    }

    public void eject() {
        panelRetract.set(true);
        panelEject.set(false);
    }

    @Override
    public void run() {
        //ElbowSet();
    }
    
}