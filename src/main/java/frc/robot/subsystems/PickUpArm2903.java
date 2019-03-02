package frc.robot.subsystems;

import frc.robot.Robot;
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
    double elbowError = 2;
    private TalonSRX leftIntakeMotor;
    private TalonSRX rightIntakeMotor;
    private TalonSRX wristMotor;
    private TalonSRX elbowMotor;
    public Solenoid panelRetract;
    public Solenoid panelEject;
    public AnalogInput elbowPotentiometer;
    public DigitalInput topHall;
    public DigitalInput bottomHall;

    ArmState currentArmState = ArmState.Confined;
    Thread currentTask = null;

    //THESE ARE NOT SET! REEEEEEE
    final int ELBOW_MAX = 999999999;
    final int ELBOW_MIN = -999999999;
    final int WRIST_MAX = 999999999;
    final int WRIST_MIN = -999999999;

    //Larger elbow = lower position
    //Larger wrist = folded up

    final int ELBOW_AWAY = 2424;  //CHANGE!!
    final int WRIST_AWAY = -417;  //CHANGE!!

    final int ELBOW_TOP = 2424;
    final int WRIST_TOP = -417;

    final int ELBOW_MIDDLE = 3245;
    final int WRIST_MIDDLE = 347;

    final int ELBOW_BOTTOM = 3797;
    final int WRIST_BOTTOM = 1120;

    final int ELBOW_FLOOR = 3878;
    final int WRIST_FLOOR = -890;

    final int ELBOW_CONFINED = 3878;  //CHANGE!!
    final int WRIST_CONFINED = -890;  //CHANGE!!

    @Override
    protected void initDefaultCommand() {
    }
    public PickUpArm2903(){
        Thread thread = new Thread(this);
        elbowPotentiometer = new AnalogInput(RobotMap.elbowPotentiometer);
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

    public enum ArmState {
        Confined, Floor, Bottom, Middle, Top, Away
    }

    public void waitASec() {
        try {
            Thread.sleep(1000);
        } catch (InterruptedException e) {
            e.printStackTrace();
        }
    }

    public void setArm(ArmState state) {
        try {
        if (currentTask != null)
            if (currentTask.isAlive()) return;

        currentTask = new Thread(() -> {
            ensureWristCanMove();
            if (state.equals(ArmState.Confined))
                goToConfined();
            else if (state.equals(ArmState.Floor))
                goToFloor();
            else if (state.equals(ArmState.Bottom))
                goToBottom();
            else if (state.equals(ArmState.Middle))
                goToMiddle();
            else if (state.equals(ArmState.Top))
                goToTop();
            else if (state.equals(ArmState.Away))
                goToAway();
        });
        currentTask.start();    //Running as thread so sleep functions don't block teleOp
    } catch (Exception ex) { }
    }

    public void goToConfined() {
        SetWristTarget(WRIST_CONFINED); //Move wrist all the way up
        waitASec(); //Give wrist time to move out of the way
        SetElbowTarget(ELBOW_CONFINED); //Move elbow all the way down
        currentArmState = ArmState.Confined;
    }

    public void goToFloor() {
        SetWristTarget(WRIST_FLOOR);
        waitASec();
        SetElbowTarget(ELBOW_FLOOR);
        currentArmState = ArmState.Floor;
    }

    public void goToBottom() {
        SetWristTarget(WRIST_BOTTOM);
        waitASec(); 
        SetElbowTarget(ELBOW_BOTTOM);
        currentArmState = ArmState.Bottom;
    }

    public void goToMiddle() {
        SetWristTarget(WRIST_MIDDLE);
        waitASec(); 
        SetElbowTarget(ELBOW_MIDDLE);
        currentArmState = ArmState.Middle;
    }

    public void goToTop() {
        if(wristNeedsUp(WRIST_MIDDLE-5)) {
        SetWristTarget(WRIST_MIDDLE);
        waitASec(); 
        }
        SetElbowTarget(ELBOW_TOP);
        waitASec(); 
        SetWristTarget(WRIST_TOP);
        currentArmState = ArmState.Top;
    }

    public void goToAway() {
        if(wristNeedsUp(WRIST_MIDDLE-5)) {
        SetWristTarget(WRIST_MIDDLE);
        waitASec(); 
        }
        SetElbowTarget(ELBOW_AWAY);
        waitASec(); 
        SetWristTarget(WRIST_AWAY);
        currentArmState = ArmState.Away;
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
    public void WristSet(){
        wristMotor.set(ControlMode.PercentOutput, -Robot.wristValue);
        SmartDashboard.putNumber("Wrist Encoder Value:", getWrist());
        SmartDashboard.putNumber("Wrist Speed:", Robot.wristValue);
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
    return elbowPotentiometer.getValue();
    }

    public void ensureWristCanMove() {
        if (getElbow() > ELBOW_BOTTOM) { //assuming wrist can move at bottom rocket position
            SetElbowTarget(ELBOW_BOTTOM);   //Arm must go up to allow wrist movement
            waitASec(); //make sure arm has time to move out of the way
            }
    }

    public boolean wristNeedsUp(double position) {
        return getWrist() < position;
    }

    public double getWrist() {
        return wristMotor.getSelectedSensorPosition();
    }
    
    public void ElbowSet(){
        if ((!topHall.get() && -Robot.dartValue > 0) ||
        (!bottomHall.get() && -Robot.dartValue < 0)) 
            elbowMotor.set(ControlMode.PercentOutput, 0);
        else
            elbowMotor.set(ControlMode.PercentOutput, -Robot.dartValue);

        SmartDashboard.putNumber("Elbow Encoder Value:", getElbow());
        SmartDashboard.putNumber("Elbow Speed:", -Robot.dartValue);
        SmartDashboard.putBoolean("Top Hall:", !topHall.get());
        SmartDashboard.putBoolean("Bottom Hall:", !bottomHall.get());
    }

    public void ElbowSetHa(double speed) {
        if ((!topHall.get() && speed > 0) ||
        (!bottomHall.get() && speed < 0)) 
            elbowMotor.set(ControlMode.PercentOutput, 0);
        else
        elbowMotor.set(ControlMode.PercentOutput, speed);

        SmartDashboard.putNumber("Elbow Encoder Value:", getElbow());
        SmartDashboard.putNumber("Elbow Speed:", Robot.dartValue);
        SmartDashboard.putBoolean("Top Hall:", !topHall.get());
        SmartDashboard.putBoolean("Bottom Hall:", !bottomHall.get());
    }

    public void SetElbowTarget(int t){
        if(t > ELBOW_MAX)
            Robot.dartController.setSetpoint(ELBOW_MAX);
        else if(t < ELBOW_MIN)
            Robot.dartController.setSetpoint(ELBOW_MIN);
        else 
            Robot.dartController.setSetpoint(t);
    }

    public void SetWristTarget(int t){
        if(t > WRIST_MAX)
            Robot.wristController.setSetpoint(WRIST_MAX);
        else if(t < WRIST_MIN)
            Robot.wristController.setSetpoint(WRIST_MIN);
        else 
            Robot.wristController.setSetpoint(t);
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
        while(true) {
        //ElbowSet();
        //WristSet();
    }
    }
    
}