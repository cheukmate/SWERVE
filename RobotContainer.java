// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.pathplanner.lib.auto.NamedCommands;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.OperatorConstants;
//import frc.robot.subsystems.Climber;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.Intake;
// import frc.robot.subsystems.Intake;
import frc.robot.subsystems.swervedrive.SwerveSubsystem;
import java.io.File;
import swervelib.SwerveInputStream;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a "declarative" paradigm, very
 * little robot logic should actually be handled in the {@link Robot} periodic methods (other than the scheduler calls).
 * Instead, the structure of the robot (including subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer
{

  // Replace with CommandPS4Controller or CommandJoystick if needed
  final         CommandXboxController operatorXbox = new CommandXboxController(1);
  //private Climber climber = new Climber();
  private ElevatorSubsystem m_elevator = new ElevatorSubsystem();
  private Intake intake = new Intake();
  // private Intake intake = new Intake();

  final         CommandXboxController driverXbox = new CommandXboxController(0);
  // The robot's subsystems and commands are defined here...
  private final SwerveSubsystem       drivebase  = new SwerveSubsystem(new File(Filesystem.getDeployDirectory(),
                                                                                "swerve/neo"));

  /**
   * Converts driver input into a field-relative ChassisSpeeds that is controlled by angular velocity.
   */
  SwerveInputStream driveAngularVelocity = SwerveInputStream.of(drivebase.getSwerveDrive(),
                                                                () -> driverXbox.getLeftY() * -1,
                                                                () -> driverXbox.getLeftX() * -1)
                                                            .withControllerRotationAxis(driverXbox::getRightX)
                                                            .deadband(OperatorConstants.DEADBAND)
                                                            .scaleTranslation(0.8)
                                                            .allianceRelativeControl(true);

  /**
   * Clone's the angular velocity input stream and converts it to a fieldRelative input stream.
   */
  SwerveInputStream driveDirectAngle = driveAngularVelocity.copy().withControllerHeadingAxis(driverXbox::getRightX,
                                                                                             driverXbox::getRightY)
                                                           .headingWhile(true);

  /**
   * Clone's the angular velocity input stream and converts it to a robotRelative input stream.
   */
  SwerveInputStream driveRobotOriented = driveAngularVelocity.copy().robotRelative(true)
                                                             .allianceRelativeControl(false);

  SwerveInputStream driveAngularVelocityKeyboard = SwerveInputStream.of(drivebase.getSwerveDrive(),
                                                                        () -> -driverXbox.getLeftY(),
                                                                        () -> -driverXbox.getLeftX())
                                                                    .withControllerRotationAxis(() -> driverXbox.getRawAxis(
                                                                        2))
                                                                    .deadband(OperatorConstants.DEADBAND)
                                                                    .scaleTranslation(0.8)
                                                                    .allianceRelativeControl(true);
  // Derive the heading axis with math!
  SwerveInputStream driveDirectAngleKeyboard     = driveAngularVelocityKeyboard.copy()
                                                                               .withControllerHeadingAxis(() ->
                                                                                                              Math.sin(
                                                                                                                  driverXbox.getRawAxis(
                                                                                                                      2) *
                                                                                                                  Math.PI) *
                                                                                                              (Math.PI *
                                                                                                               2),
                                                                                                          () ->
                                                                                                              Math.cos(
                                                                                                                  driverXbox.getRawAxis(
                                                                                                                      2) *
                                                                                                                  Math.PI) *
                                                                                                              (Math.PI *
                                                                                                               2))
                                                                               .headingWhile(true);

  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  public RobotContainer()
  {

    //climber.setDefaultCommand(
        //new RunCommand (() -> climber.Climb(0), climber));
    //  m_elevator.setDefaultCommand(
    //     new RunCommand(() -> m_elevator.setManualPower(0), m_elevator));

    //m_elevator.setDefaultCommand(m_elevator.setElevatorHeight(ElevatorConstants.bottomPos));
    
    // Configure the trigger bindings
    configureBindings();
    DriverStation.silenceJoystickConnectionWarning(true);
    NamedCommands.registerCommand("test", Commands.print("I EXIST"));
  }

  /**
   * Use this method to define your trigger->command mappings. Triggers can be created via the
   * {@link Trigger#Trigger(java.util.function.BooleanSupplier)} constructor with an arbitrary predicate, or via the
   * named factories in {@link edu.wpi.first.wpilibj2.command.button.CommandGenericHID}'s subclasses for
   * {@link CommandXboxController Xbox}/{@link edu.wpi.first.wpilibj2.command.button.CommandPS4Controller PS4}
   * controllers or {@link edu.wpi.first.wpilibj2.command.button.CommandJoystick Flight joysticks}.
   */
  private void configureBindings()
  {

    Command driveFieldOrientedDirectAngle      = drivebase.driveFieldOriented(driveDirectAngle);
    Command driveFieldOrientedAnglularVelocity = drivebase.driveFieldOriented(driveAngularVelocity);
    Command driveRobotOrientedAngularVelocity  = drivebase.driveFieldOriented(driveRobotOriented);
    Command driveSetpointGen = drivebase.driveWithSetpointGeneratorFieldRelative(
        driveDirectAngle);
    Command driveFieldOrientedDirectAngleKeyboard      = drivebase.driveFieldOriented(driveDirectAngleKeyboard);
    Command driveFieldOrientedAnglularVelocityKeyboard = drivebase.driveFieldOriented(driveAngularVelocityKeyboard);
    Command driveSetpointGenKeyboard = drivebase.driveWithSetpointGeneratorFieldRelative(
        driveDirectAngleKeyboard);





    if (RobotBase.isSimulation())
    {
      drivebase.setDefaultCommand(driveFieldOrientedDirectAngleKeyboard);
    } else
    {
      drivebase.setDefaultCommand(driveFieldOrientedAnglularVelocity);
    }

    if (Robot.isSimulation())
    {
      driverXbox.start().onTrue(Commands.runOnce(() -> drivebase.resetOdometry(new Pose2d(3, 3, new Rotation2d()))));
      driverXbox.button(1).whileTrue(drivebase.sysIdDriveMotorCommand());

    }
    if (DriverStation.isTest())
    {
      drivebase.setDefaultCommand(driveFieldOrientedAnglularVelocity); // Overrides drive command above!

      driverXbox.x().whileTrue(Commands.runOnce(drivebase::lock, drivebase).repeatedly());
      driverXbox.y().whileTrue(drivebase.driveToDistanceCommand(1.0, 0.2));
      driverXbox.start().onTrue((Commands.runOnce(drivebase::zeroGyro)));
      driverXbox.back().whileTrue(drivebase.centerModulesCommand());
      driverXbox.leftBumper().onTrue(Commands.none());
      driverXbox.rightBumper().onTrue(Commands.none());
      
    } else
    {
      driverXbox.a().onTrue((Commands.runOnce(drivebase::zeroGyro)));
      driverXbox.x().onTrue(Commands.runOnce(drivebase::addFakeVisionReading));
      driverXbox.b().whileTrue(
          drivebase.driveToPose(
              new Pose2d(new Translation2d(4, 4), Rotation2d.fromDegrees(0)))
                              );
      driverXbox.start().whileTrue(Commands.none());
      driverXbox.back().whileTrue(Commands.none());
      driverXbox.leftBumper().whileTrue(Commands.runOnce(drivebase::lock, drivebase).repeatedly());
      driverXbox.rightBumper().onTrue(Commands.none());

    //Operator commands


  
// operatorXbox.b().onTrue(new InstantCommand(() -> intake.shootCoral(.2)));
// operatorXbox.x().onTrue(new InstantCommand(() -> intake.intakeCoral(-.2)));

// operatorXbox.b().onFalse(new InstantCommand(() -> intake.stopItIntake(0)));
// operatorXbox.x().onFalse(new InstantCommand(() -> intake.stopItIntake(0)));

//  operatorXbox.a().whileTrue( new RunCommand(() 
//          -> m_elevator.setElevatorHeight
//             (ElevatorConstants.L2), m_elevator));


// operatorXbox.a().whileTrue( new RunCommand(() 
// -> m_elevator.setManualPower(.6)
//    , m_elevator));

//   operatorXbox.y().onTrue( new RunCommand(() 
//             -> m_elevator.setManualPower(-.6)
//                , m_elevator));

// operatorXbox.a().whileFalse( new RunCommand(() 
//             -> m_elevator.setManualPower(0)
//              ,m_elevator));

// operatorXbox.a().whileTrue(m_elevator.setElevatorHeight(ElevatorConstants.L2));

// TODO: bind the elevator commands.... when you make them

 operatorXbox.a().onTrue( new InstantCommand(() ->
              m_elevator.setManualPower(.5)));

operatorXbox.a().onFalse( new InstantCommand(() ->
              m_elevator.Stop(0)));

operatorXbox.b().onTrue( new InstantCommand(() ->
              m_elevator.goDown(-.5)));

operatorXbox.b().onFalse( new InstantCommand(() ->
              m_elevator.Stop(0)));


//coral manip

operatorXbox.x().onTrue( new InstantCommand(() ->
              intake.intakeCoral(.2)));

operatorXbox.y().onTrue( new InstantCommand(() ->
              intake.launchCoral(-.2)));


operatorXbox.rightBumper().onTrue( new InstantCommand(() ->
              intake.ScoringPosition(Constants.IntakeConstants.ScoringPosition)));

operatorXbox.leftBumper().onTrue( new InstantCommand(() -> 
                intake.IntakePosition(Constants.IntakeConstants.IntakePosition)));





operatorXbox.x().onFalse( new InstantCommand(() ->
              intake.stopIntake(0)));

operatorXbox.y().onFalse( new InstantCommand(() ->
              intake.stopIntake(0)));


operatorXbox.rightBumper().onFalse( new InstantCommand(() ->
              intake.stopRotateScore(-0.012342340)));

operatorXbox.leftBumper().onFalse( new InstantCommand(() ->
              intake.stopRotate(0.01234234)));
              
operatorXbox.pov(180).onTrue(new InstantCommand(() -> intake.ZeroEncoder()));




//     operatorXbox.b().whileTrue( new InstantCommand(() ->
//              climber.Climb(-.5)));

    
//   operatorXbox.b().whileFalse( new InstantCommand(() ->
//   climber.Climb(0)));



//   operatorXbox.b().whileFalse( new InstantCommand(() ->
//   climber.Climb(0)));



    }
  
  
  }
  
    


  

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand()
  {
    // An example command will be run in autonomous
    return drivebase.getAutonomousCommand("New Auto");
  }

  public void setMotorBrake(boolean brake)
  {
    drivebase.setMotorBrake(brake);
  }
}
