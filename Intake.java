// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;



import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import com.revrobotics.spark.SparkMax;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.ElevatorConstants;


public class Intake extends SubsystemBase {
  /** Creates a new Intake. */

 
  SparkMax rotateMotor = new SparkMax(11, MotorType.kBrushless);
  SparkMax wheelMotor = new SparkMax(9, MotorType.kBrushless);

    private final RelativeEncoder encoder;
    private final PIDController pidController;
    private double targetPosition; // Target position in degrees

    private TrapezoidProfile.Constraints constraints;
    private TrapezoidProfile profile;
    private TrapezoidProfile.State currentState;
    private TrapezoidProfile.State goalState;
  
  

    public enum IntakePosition {

            START(Constants.IntakeConstants.STARTING),
            SCORING(Constants.IntakeConstants.SCORING),
            OUT(Constants.IntakeConstants.OUTWARDS);
        
            public final double positionDegrees;

        
            IntakePosition(double positionDegrees) {
                this.positionDegrees = positionDegrees;
            }
  }


  public Intake() {

    encoder = rotateMotor.getEncoder();
    pidController = new PIDController

              ( Constants.IntakeConstants.kPR,
                Constants.IntakeConstants.kIR,
                Constants.IntakeConstants.kDR );
    
    pidController.setTolerance(1.0); // Tolerance in degrees

    constraints = new TrapezoidProfile.Constraints(

      Constants.IntakeConstants.maxVelocity,
      Constants.IntakeConstants.maxAcceleration

    );

 
      currentState = new TrapezoidProfile.State(0, 0);  // Starting at position 0 with velocity 0
      goalState = new TrapezoidProfile.State(0, 0);     // Goal state will be updated when set
      profile = new TrapezoidProfile(constraints);

}
      



  

  public void setMotorBrake(boolean brake){

    rotateMotor.set(0);

  }


  @Override
  public void periodic() {

 // This method will be called once per scheduler run

        double currentPosition = getCurrentPositionDegrees(); // Get current position in degrees

        double pidOutput = pidController.calculate(currentPosition, 
                                                    currentState.position);

        double outputPower = MathUtil.clamp(pidOutput, 
                                            -Constants.IntakeConstants.maxOutput,
                                            Constants.IntakeConstants.maxOutput);

        currentState = profile.calculate(0.020, currentState, goalState); // 20ms control loop

        

        rotateMotor.set(outputPower);



  }

  public void setPositionDegrees(double targetDegrees) {

    // Convert target degrees to the corresponding encoder position

    targetPosition = MathUtil.clamp

    (targetDegrees, 0, 360); // 0-360 degrees

    goalState = new TrapezoidProfile.State(targetPosition,
                                           0);

}

public void shootCoral(double speed){

wheelMotor.set
(.2);

}


public void intakeCoral(double speed){

wheelMotor.set
(-.2);

}


public void stopItIntake(double speed){

wheelMotor.set
(0);

}


private double getCurrentPositionDegrees() {

    // Convert encoder counts to degrees

    return encoder.getPosition() * 360.0 / 
    ElevatorConstants.countsPerInch;

}

public boolean isAtPosition() {

    return pidController.atSetpoint();

}


public void stopMotor() {

    rotateMotor.set(0);
    pidController.reset();

}

}
