package frc.robot;

import com.ctre.phoenix.motorcontrol.VictorSPXControlMode;
import com.ctre.phoenix.motorcontrol.can.VictorSPX;

import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.Servo;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.Compressor;

public class Robot extends TimedRobot {

  // Controller Assignments //

  Joystick driverController = new Joystick(0);    // Logitech Extreme 3D Pro //
  GenericHID functionController = new GenericHID(1);    // Other USB Items //
  double servoPos = driverController.getThrottle();   // Assigns Throttle to Servo on PWM 0 //
  double autoStart = 0;

  // Motor Assignments //

  VictorSPX driveLeftA = new VictorSPX(4);   // VictorSPX LeftA //
  VictorSPX driveLeftB = new VictorSPX(3);   // VictorSPX LeftB //
  VictorSPX driveRightA = new VictorSPX(2);   // VictorSPX RightA //  
  VictorSPX driveRightB = new VictorSPX(1);   // VictorSPX RightB // 
  
  // Servo Assignments //

  Servo servo0 = new Servo(0);    // Servo on PWM 0 //
  Servo servo1 = new Servo(1);    // Servo on PWM 1 //
  Servo servo2 = new Servo(2);    // Servo on PWM 2 //

  // Compresser Component Assignments //

  Compressor compressor = new Compressor(PneumaticsModuleType.CTREPCM);    // CTRE Compressor //

  Solenoid solenoid0 = new Solenoid(PneumaticsModuleType.CTREPCM, 0);    // CTRE Solenoid 0 //
  Solenoid solenoid1 = new Solenoid(PneumaticsModuleType.CTREPCM, 1);    // CTRE Solenoid 1 //

  boolean enabled = compressor.enabled();   // Pressure Switch Enabled //
  boolean pressureSwitch = compressor.getPressureSwitchValue();   // Get Pressure Switch Value //
  double current = compressor.getCurrent();   // Gets Compressors Current Output Pressure //

  // Acceleration Limiter //

  SlewRateLimiter filter = new SlewRateLimiter(0.5);  // SlewRateLimiter //

  @Override
  public void robotInit() {  // initialize robot //
  }
  

  @Override
  public void autonomousInit() {

    // Get a time for auton start to do events based on time later. //

    autoStart = Timer.getFPGATimestamp();

  }

  // Autonomous moves forward for 1.5 seconds at 0.5 speed then stops. //

  @Override
  public void autonomousPeriodic() {

    double autoTimeElapsed = Timer.getFPGATimestamp() - autoStart;
      
      Double autoCode = 1.0;
     
    if(autoCode == 1) {
      if(autoTimeElapsed<1.5) {

        driveLeftA.set(VictorSPXControlMode.PercentOutput, 0.5);    // Set output speed 0.5 on LeftA. //
        driveLeftB.set(VictorSPXControlMode.PercentOutput, 0.5);    // Set output speed 0.5 on LeftB. //
        driveRightA.set(VictorSPXControlMode.PercentOutput, -0.5);    // Set output speed 0.5 on RightA. //
        driveRightB.set(VictorSPXControlMode.PercentOutput, -0.5);    // Set output speed 0.5 on RightB. //

      }

      else {

        driveLeftA.set(VictorSPXControlMode.PercentOutput, 0);    // Set output speed 0 on LeftA. //
        driveLeftB.set(VictorSPXControlMode.PercentOutput, 0);    // Set output speed 0 on LeftB. //
        driveRightA.set(VictorSPXControlMode.PercentOutput, 0);    // Set output speed 0 on RightA. //
        driveRightB.set(VictorSPXControlMode.PercentOutput, 0);    // Set output speed 0 on RightB. //

      }

    } 

  }


  // This function is called once when teleop is enabled. //
  @Override
  public void teleopInit() {

  }

  // This function is called periodically during operator control. //
  @Override
  public void teleopPeriodic() {

    // Servos //

    servo0.setAngle(servoPos);

    // Compresser & Solenoid //

    if (driverController.getRawButton(5)) {   // Enables Compresser ONLY when Button 5 is pressed //
      compressor.enableDigital();             // and disables when Pressure switch goes active.   //
    } else if (pressureSwitch = true) {
      compressor.disable();        
    }                       

    if (driverController.getRawButton(4)) {   // Open Solenoid ONLY if Trigger is pressed //
      solenoid0.set(true);
    } else {
      solenoid0.set(false);
    }

    if (driverController.getRawButton(3)) {   // Open Solenoid ONLY if Trigger is pressed //
      solenoid1.set(true);
    } else {
      solenoid1.set(false);
    }

    // Driving Controls //

    double slowturn = driverController.getRawAxis(2);   // Axis 2 Slowturn //
    double forward = driverController.getRawAxis(1);    // Axis 1 Forward //
    double turn = -driverController.getRawAxis(0);    // Axis 0 Turn //
    
    driveLeftA.set(VictorSPXControlMode.PercentOutput, -(forward+(turn*0.5+(slowturn*-0.35))));   // Forward and Turn Speed at -0.5 & slowturn speed at -0.35 //
    driveLeftB.set(VictorSPXControlMode.PercentOutput, -(forward+(turn*0.5+(slowturn*-0.35))));   // Forward and Turn Speed at -0.5 & slowturn speed at -0.35 //
    driveRightA.set(VictorSPXControlMode.PercentOutput, forward-(turn*0.5-(slowturn*0.35)));    // Forward and Turn Speed at 0.5 & slowturn speed at 0.35 //
    driveRightB.set(VictorSPXControlMode.PercentOutput, forward-(turn*0.5-(slowturn*0.35)));    // Forward and Turn Speed at 0.5 & slowturn speed at 0.35 //
    
    // SlewRateLimiter //

    filter.calculate(turn);   // Slows Acceleration on turn by 0.5 //
    
  }
  
  // Disable Motors //

  @Override
  public void disabledInit() {
    driveLeftA.set(VictorSPXControlMode.PercentOutput,0);   // Disable Motor LeftA //
    driveLeftB.set(VictorSPXControlMode.PercentOutput,0);   // Disable Motors LeftB //
    driveRightA.set(VictorSPXControlMode.PercentOutput,0);    // Disable Motors RightA //
    driveRightB.set(VictorSPXControlMode.PercentOutput,0);    // Disable Motors RightB //

    servo0.setAngle(90);

    compressor.disable();

    solenoid0.set(false);
    solenoid1.set(false);

  }

}  