// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.ElevatorConstants;

public class SuperStructure extends SubsystemBase {
  /** Creates a new SuperStructure. */
  Intake intake;
  ElevatorSubsystem elevator;
  public SuperStructure(Intake intake, ElevatorSubsystem elevator) {
    this.intake = intake;
    this.elevator = elevator;
  }


public Command ScoreL1(){
return run(
  () -> {

    elevator.setPositionClosedLoopWithFF(ElevatorConstants.L1, 0);
    intake.setPivotPosition(Constants.IntakeConstants.ScoringPosition);

  });

}

public Command ScoreL2(){
  return run(
  () -> {

    intake.setPivotPosition(Constants.IntakeConstants.ScoringPosition);
    elevator.setPositionClosedLoopWithFF(ElevatorConstants.L2, 0);

  });
 

}

public Command ScoreL3(){
  return run(
  () -> {
    intake.setPivotPosition(Constants.IntakeConstants.ScoringPosition);
    elevator.setPositionClosedLoopWithFF(ElevatorConstants.L3, 0);
    


  });

}


public Command ScoreL4(){
  return run(
  () -> {

    intake.setPivotPosition(Constants.IntakeConstants.ScoringPosition);
    elevator.setPositionClosedLoopWithFF(ElevatorConstants.L4, 0);
  

  });

}


public Command Intake(){
  return run(
  () -> {

    elevator.setPositionClosedLoopWithFF(ElevatorConstants.L1, 0);
    intake.setPivotPosition(Constants.IntakeConstants.IntakePosition);

  });

}

public Command stopMotors(){
return run(
() -> {

elevator.Stop(0);
intake.stopRotate(0);

});



}


  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
