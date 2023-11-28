
package frc.robot;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.VictorSPX;
import edu.wpi.first.wpilibj.SPI;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;


public class Robot extends TimedRobot {
  private final Timer c_timer = new Timer();
  private VictorSPX motorL = new VictorSPX(7);
  private VictorSPX motorL2 = new VictorSPX(2);
  private VictorSPX motorR = new VictorSPX(1);
  private VictorSPX motorR2 = new VictorSPX(4);

  private int kAdjustValue = 14;
  private double kSpd = 0.4;
  private boolean centralized = false;
  private boolean totalCentrilized = false;

  //Max dis = 75cm area% = 0.8
  //Min dis = 195cm area% = 0.1
  private double kMaxArea = 0.9;
  private double kMinArea = 0.6;
  

  @Override
  public void robotInit() {
    motorL2.follow(motorL); 
    motorR2.follow(motorR);
    motorL2.setInverted(false);
    motorL.setInverted(false);
    motorR.setInverted(true);
    motorR2.setInverted(true);
  }


  /** This function is called periodically during teleoperated mode. */
  @Override
  public void teleopPeriodic() {
    NetworkTable table = NetworkTableInstance.getDefault().getTable("limelight-hydra");
    NetworkTableEntry tx = table.getEntry("tx");
    NetworkTableEntry ty = table.getEntry("ty");
    NetworkTableEntry ta = table.getEntry("ta");
    NetworkTableEntry tv = table.getEntry("tv");
    NetworkTableEntry target_id = table.getEntry("tid");
    //read values periodically
    double x = tx.getDouble(0.0);
    double y = ty.getDouble(0.0);
    double area = ta.getDouble(0.0);
    double tId = target_id.getDouble(0.0);
    boolean targetB = tv.getBoolean(false);

    //post to smart dashboard periodically
    SmartDashboard.putNumber("LimelightX", x);
    SmartDashboard.putNumber("LimelightY", y);
    SmartDashboard.putNumber("LimelightArea", area);
    SmartDashboard.putNumber("Target ID", tId);
    
    targetCentralized(x);
    SmartDashboard.putBoolean("Centralizado?", centralized);
    SmartDashboard.putNumber("Centralized Timer", c_timer.get());
    SmartDashboard.putBoolean("Total Centralizado", totalCentrilized);

    stopBabe();

    if(!centralized){
      //targetFollowerL(x);
      //targetFollowerR(x);
      totalCentrilized = false;
      c_timer.stop();
      c_timer.reset();
    }
    if(centralized){
      
      if(c_timer.get() >= 0.65) {
        totalCentrilized = true;
        c_timer.stop();
      }

    }
    if(totalCentrilized && targetB){
      dogFollower(area);
    }
  }

  /** This function is called once each time the robot enters test mode. */
  @Override
  public void testInit() {}

  /** This function is called periodically during test mode. */
  @Override
  public void testPeriodic() {}

  private void turnLeft(double spd){
    motorL.set(ControlMode.PercentOutput, -spd);
    motorR.set(ControlMode.PercentOutput, spd);
  }

  private void turnRight(double spd){
    motorL.set(ControlMode.PercentOutput, spd);
    motorR.set(ControlMode.PercentOutput, -spd);
  }

  private void goFront(double spd){
    motorL.set(ControlMode.PercentOutput, spd);
    motorR.set(ControlMode.PercentOutput, spd);
  }

  private void goBack(double spd){
    motorL.set(ControlMode.PercentOutput, -spd);
    motorR.set(ControlMode.PercentOutput, -spd);
  }

  private void stopBabe(){
    motorL.set(ControlMode.PercentOutput, 0);
    motorR.set(ControlMode.PercentOutput, 0);
  }

  private void targetFollowerL(double xAxis){
    if(xAxis < -kAdjustValue) turnLeft(kSpd - 1);
    else stopBabe();
  }
  private void targetFollowerR(double xAxis){
    if(xAxis > kAdjustValue) turnRight(kSpd - 1);
  }

  private void targetCentralized(double xAxis){
    if(Math.abs(xAxis) <= kAdjustValue) {
      centralized = true;
      stopBabe();
    }
    else centralized = false;
  }

  private void dogFollower(double a){
    stopBabe();
      if(a > kMaxArea){
        goBack(kSpd);
      } 
      else if(a < kMinArea){
        goFront(kSpd);
      }
  }

  }