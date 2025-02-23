// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.trajectory.ExponentialProfile.Constraints;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;


public class Intake extends SubsystemBase {
  /** Creates a new Intake. */

SparkMax turningMotor = new SparkMax(11, MotorType.kBrushless);
SparkMax intakeMotor = new SparkMax(9, MotorType.kBrushless);

 private final ProfiledPIDController pivotController = new ProfiledPIDController(
     Constants.IntakeConstants.kP, 
     Constants.IntakeConstants.kI, 
     Constants.IntakeConstants.kD,

    // Creates a new set of trapezoidal motion profile constraints
     // Max velocity of 5 meters per second
    // Max acceleration of 10 meters per second squared
  
    // Max velocity & acceleration definded belowwww
   new TrapezoidProfile.Constraints(5, 10)
                                    
 );


private SparkMaxConfig config;

private RelativeEncoder encoder;

  public Intake() {
// no more encoder identity crisis
    encoder = turningMotor.getEncoder();
// Make a new config
    config = new SparkMaxConfig();
    config.inverted(false).idleMode(IdleMode.kBrake);


    //Configure both motors.
    intakeMotor.configure(config, 
                            ResetMode.kResetSafeParameters,
                            PersistMode.kNoPersistParameters);


   turningMotor.configure(config, 
                          ResetMode.kResetSafeParameters,
                          PersistMode.kNoPersistParameters);
    
// smart dash board
                          SmartDashboard.setDefaultNumber("Target Position", 0);
                          SmartDashboard.setDefaultNumber("Target Velocity", 0);
                          SmartDashboard.setDefaultBoolean("Control Mode", false);
                          SmartDashboard.setDefaultBoolean("Reset Encoder", false);
  }

  // using WPI instead of sparkmax

  public void setPivotPosition(double targetPosition) {
    double output = pivotController.calculate(encoder.getPosition(), targetPosition);
    turningMotor.set(output);
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

public void Command(Intake intake, double targetPosition){



 

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
intakeMotor.set(0.00000000000);

  }

  
public void stopRotate(double power){
  setPivotPosition(getPivotPosition());
}
public void stopRotateScore(double power){
 setPivotPosition(getPivotPosition());
  }





  @Override
  public void periodic() {
SmartDashboard.putNumber("Encoder Value", turningMotor.getEncoder().getPosition());
SmartDashboard.putNumber("Actual Position", encoder.getPosition());
SmartDashboard.putNumber("Actual Velocity", encoder.getVelocity());

if (SmartDashboard.getBoolean("Reset Encoder", false)) {
  SmartDashboard.putBoolean("Reset Encoder", false);
  
    }
  }
}