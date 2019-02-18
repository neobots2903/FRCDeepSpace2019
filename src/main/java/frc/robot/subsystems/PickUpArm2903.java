package frc.robot.subsystems;

import frc.robot.RobotMap;

import edu.wpi.first.wpilibj.command.Subsystem;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.DigitalInput;



public class PickUpArm2903 extends Subsystem implements Runnable{
    double speed = 0.5;
    private TalonSRX leftIntakeMotor;
    private TalonSRX rightIntakeMotor;
    private TalonSRX wristMotor;
    private TalonSRX elbowMotor;
    public Solenoid leftPunch;
    public Solenoid rightPunch;
    public AnalogInput elbowPotentiometer;
    public DigitalInput topHall;
    public DigitalInput bottomHall;


    //THESE ARE NOT SET! REEEEEEE
    final int ELBOW_MAX = 10;
    final int ELBOW_MIN = 0;
    final int WRIST_MAX = 10;
    final int WRIST_MIN = 0;

    private int target = 0;

    @Override
    protected void initDefaultCommand() {
    }
    public PickUpArm2903(){
        Thread thread = new Thread(this);
        elbowPotentiometer = new AnalogInput(0);
        target = elbowPotentiometer.getValue();
        leftIntakeMotor = new TalonSRX(RobotMap.TBD);
        rightIntakeMotor = new TalonSRX(RobotMap.TBD);
        wristMotor = new TalonSRX(RobotMap.TBD);
        wristMotor.configSelectedFeedbackSensor(FeedbackDevice.QuadEncoder, 0, 10);
        elbowMotor = new TalonSRX(RobotMap.TBD);
        leftPunch = new Solenoid(RobotMap.TBD);
        rightPunch = new Solenoid(RobotMap.TBD);
        topHall = new DigitalInput(0);
        bottomHall = new DigitalInput(1);
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
    
    public void ElbowSet(){
        if(elbowPotentiometer.getValue() < target){
            elbowMotor.set(ControlMode.PercentOutput, speed);
        }
        else if(elbowPotentiometer.getValue() > target){
            elbowMotor.set(ControlMode.PercentOutput, -speed);
        }
        else{
            ElbowStop();
        }
    }

    public void SetElbowTarget(int t){
        if(t > ELBOW_MAX)
            target = ELBOW_MAX;
        else if(t < ELBOW_MIN)
            target = ELBOW_MIN;
        else target = t;
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
        target = elbowPotentiometer.getValue();
    }
    public void punch(){
        leftPunch.set(true);
        rightPunch.set(true);
        try {
            Thread.sleep(500);
        } catch (InterruptedException e) {
            e.printStackTrace();
        }
        leftPunch.set(false);
        rightPunch.set(false);
    }

    @Override
    public void run() {
        ElbowSet();
    }
    
}