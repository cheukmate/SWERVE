// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.AlternateEncoderConfig;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.motorcontrol.Spark;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.ElevatorConstants;
import frc.robot.Constants.IntakeConstants;

public class Intake extends SubsystemBase {
  /** Creates a new Intake. */

SparkMax turningMotor = new SparkMax(11, MotorType.kBrushless);
SparkMax intakeMotor = new SparkMax(9, MotorType.kBrushless);


private SparkMaxConfig config;
SparkClosedLoopController closedLoopController;
private RelativeEncoder encoder;

  public Intake() {
    config = new SparkMaxConfig();
    config.inverted(false).idleMode(IdleMode.kBrake);
//first PID is for position control, second is for velocity control.
    config
      .closedLoop
      .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
      .p(Constants.IntakeConstants.kP)
      .i(Constants.IntakeConstants.kI)
      .d(Constants.IntakeConstants.kD)
      .outputRange(-.1, .1)

      .p(0.0002, ClosedLoopSlot.kSlot1)
      .i(0.1, ClosedLoopSlot.kSlot1)
      .d(0.00023, ClosedLoopSlot.kSlot1);

      closedLoopController = turningMotor.getClosedLoopController();
      encoder = turningMotor.getEncoder();
    

   turningMotor.configure(config, 
                          ResetMode.kResetSafeParameters,
                          PersistMode.kNoPersistParameters);
    
  }

  


   public void setPivotPosition(double targetPosition) {
    turningMotor.getClosedLoopController().setReference(
                                                        targetPosition,
                                                        SparkMax.ControlType.kPosition);
  }

  public double getPivotPosition() {

    return encoder.getPosition();

}

public void ZeroEncoder(){

  encoder.setPosition(0);

}

public void IntakePosition(double targetPosition){

setPivotPosition(Constants.IntakeConstants.IntakePosition);

}

public void ScoringPosition(double targetPosition){

setPivotPosition(Constants.IntakeConstants.ScoringPosition);

}





  public void intakeCoral(double power){
    
    intakeMotor.set(.2222222222222);

  }

  public void launchCoral(double power){
intakeMotor.set(-.22222222);

  }




  public void stopIntake(double power){
intakeMotor.set(0.01742);

  }

  public void turnIntakeTowardsFun(double power){
turningMotor.set(.25);
    
      }

  public void turnAway(double power){
turningMotor.set(-.25);
  }
public void stopRotate(double power){
turningMotor.set(0.08234234);
}
public void stopRotateScore(double power){
  turningMotor.set(-0.13734234);
  }





  @Override
  public void periodic() {
SmartDashboard.putNumber("Encoder Value", turningMotor.getEncoder().getPosition());
SmartDashboard.putNumber("Actual Position", encoder.getPosition());
SmartDashboard.putNumber("Actual Velocity", encoder.getVelocity());

if (SmartDashboard.getBoolean("Reset Encoder", false)) {
  SmartDashboard.putBoolean("Reset Encoder", false);
  // Reset the encoder position to 0
 
  
  SmartDashboard.setDefaultNumber("Target Position", 0);
  SmartDashboard.setDefaultNumber("Target Velocity", 0);
  SmartDashboard.setDefaultBoolean("Control Mode", false);
  SmartDashboard.setDefaultBoolean("Reset Encoder", false);


    // This method will be called once per scheduler run
    }
  }
}