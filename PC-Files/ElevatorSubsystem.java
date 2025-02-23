// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.ElevatorConstants;


public class ElevatorSubsystem extends SubsystemBase {
  /** Creates a new ElevatorSubsystem. */

  SparkMax elevatorMotor = new SparkMax(10, MotorType.kBrushless);
  SparkMaxConfig config;

 private final ProfiledPIDController elevator = new ProfiledPIDController(
    ElevatorConstants.kP, 
    ElevatorConstants.kI, 
    ElevatorConstants.kD, 
    new TrapezoidProfile.Constraints(10, 20) // Max velocity & acceleration
);
  public ElevatorSubsystem() {
    
    config = new SparkMaxConfig();
    config.inverted(false).idleMode(IdleMode.kBrake);
    config.encoder.positionConversionFactor(2.093) // distance per rev / gear ratio
                  .velocityConversionFactor(0.03488); // posconfac/60 . we need persec
    config
          .closedLoop
          .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
          .pid(ElevatorConstants.kP, ElevatorConstants.kI, ElevatorConstants.kD);

          

  }


   public void setVoltage(double voltage){

     elevatorMotor.getClosedLoopController().setReference(voltage, ControlType.kVoltage);

   }

    public void setPositionClosedLoopWithFF(double position, double arbFF) {

    elevatorMotor
        .getClosedLoopController()
        .setReference(position, ControlType.kPosition, ClosedLoopSlot.kSlot0, arbFF);
  }

  public void setProfiledPosition(double targetPosition) {
    
    TrapezoidProfile.State setpoint = 
        new TrapezoidProfile.State(targetPosition, 0); // Target position, 0 velocity
    double output = elevator.calculate(getPos(), setpoint.position);
    
    elevatorMotor.set(output);
}



   public double getPos(){
return elevatorMotor.getEncoder().getPosition();

   }

   public void setManualPower(double power){

    elevatorMotor.set(.5);
    
      }
    
      public void goDown(double power){
    
    elevatorMotor.set(-.5);
        
          }
    
      public void Stop(double power){
    
    elevatorMotor.set(0);
    
      }

  @Override
  public void periodic() {
    SmartDashboard.putNumber("Elevator Position", getPos());
    // This method will be called once per scheduler run
  }
}
