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

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.ElevatorConstants;


public class ElevatorSubsystem extends SubsystemBase {
  /** Creates a new ElevatorSubsystem. */

  SparkMax elevatorMotor = new SparkMax(10, MotorType.kBrushless);
  SparkMaxConfig config;
  public ElevatorSubsystem() {
    
    config = new SparkMaxConfig();
    config.inverted(false).idleMode(IdleMode.kBrake);
    config.encoder.positionConversionFactor(2.093) //  distance per rev / gear ratio
                  .velocityConversionFactor(0.03488); // posconfac/60 . we need persec
    config
          .closedLoop
          .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
          .pid(ElevatorConstants.kP, ElevatorConstants.kI, ElevatorConstants.kD);





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


   public void setVoltage(double voltage){

     elevatorMotor.getClosedLoopController().setReference(voltage, ControlType.kVoltage);

   }

    public void setPositionClosedLoopWithFF(double position, double arbFF) {

    elevatorMotor
        .getClosedLoopController()
        .setReference(position, ControlType.kPosition, ClosedLoopSlot.kSlot0, arbFF);
  }




public void setPos(double position){

elevatorMotor.getClosedLoopController()
            .setReference(position, 
            ControlType.kPosition);
  }

public void setPosInches(double position){
    elevatorMotor.
                  getClosedLoopController()
                  .setReference(position, ControlType.kPosition);
  }


   public double getPos(){
return elevatorMotor.getEncoder().getPosition();

   }


  @Override
  public void periodic() {
    SmartDashboard.putNumber("Elevator Position", getPos());
    // This method will be called once per scheduler run
  }
}
