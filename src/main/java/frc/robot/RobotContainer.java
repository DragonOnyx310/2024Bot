// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.ctre.phoenix6.Utils;
import com.ctre.phoenix6.mechanisms.swerve.SwerveRequest;
import com.ctre.phoenix6.mechanisms.swerve.SwerveModule.DriveRequestType;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.PS4Controller.Button;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.POVButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.commands.Elevator.MoveElevator;
import frc.robot.commands.Intake.MoveIntake;
import frc.robot.commands.Intake.ReverseIntake;
import frc.robot.commands.Intake.StoreIntake;
import frc.robot.commands.Intake.intakeToggle;
import frc.robot.commands.Shooter.AimShooter;
import frc.robot.commands.Shooter.ShooterIntakeTilSight;
import frc.robot.commands.Shooter.ShooterToggle;
import frc.robot.commands.Shooter.StoreShooter;
import frc.robot.commands.limelight.AlignToTag;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.ClimberSubsystem;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.Limelight;
import frc.robot.subsystems.ShooterSubsystem;

public class RobotContainer {
  private static double MaxSpeed = 6; // 6 meters per second desired top speed
  private static double MaxAngularRate = 1.5 * Math.PI; // 3/4 of a rotation per second max angular velocity


  //private final SendableChooser<Command> autoChooser;

  /* Setting up bindings for necessary control of the swerve drive platform */
  private final CommandXboxController joystick = new CommandXboxController(0); // My joystick
  public final static CommandSwerveDrivetrain drivetrain = TunerConstants.DriveTrain; // My drivetrain
  public static final GenericHID driver = new GenericHID(0);

  public static JoystickButton A = new JoystickButton(driver, 1);
  public static JoystickButton B = new JoystickButton(driver, 2);
  public static JoystickButton X = new JoystickButton(driver, 3);
  public static JoystickButton Y = new JoystickButton(driver, 4);
  public static JoystickButton righButton = new JoystickButton(driver, 6);
  public static JoystickButton leftButton = new JoystickButton(driver, 5);
  public static JoystickButton Start = new JoystickButton(driver, 8);
  public static JoystickButton Menu = new JoystickButton(driver, 7);
  public static POVButton UpDpad = new POVButton(driver, 0);
  public static POVButton downDpad = new POVButton(driver, 180);


  public static final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
      .withDeadband(MaxSpeed * 0.1).withRotationalDeadband(MaxAngularRate * 0.1) // Add a 10% deadband
      .withDriveRequestType(DriveRequestType.OpenLoopVoltage); // I want field-centric
                                                               // driving in open loop
  private final SwerveRequest.SwerveDriveBrake brake = new SwerveRequest.SwerveDriveBrake();
  private final SwerveRequest.PointWheelsAt point = new SwerveRequest.PointWheelsAt();
  private final Telemetry logger = new Telemetry(MaxSpeed);


  public static final ShooterSubsystem mShooterSubsystem = new ShooterSubsystem();
  public static final IntakeSubsystem mIntakeSubsystem = new IntakeSubsystem();
  public static final ElevatorSubsystem mElevatorSubsystem = new ElevatorSubsystem();
  public static final ClimberSubsystem mClimberSubsystem = new ClimberSubsystem();
  public static final Limelight mLimelight = new Limelight();

  public static Trigger trigger(GenericHID controller, int axis){
    return new Trigger(()-> controller.getRawAxis(axis) >= 0.9);
  }


  private void configureBindings() {
    drivetrain.setDefaultCommand( // Drivetrain will execute this command periodically
        drivetrain.applyRequest(() -> drive.withVelocityX(-joystick.getLeftY() * MaxSpeed) // Drive forward with
                                                                                           // negative Y (forward)
            .withVelocityY(-joystick.getLeftX() * MaxSpeed) // Drive left with negative X (left)
            .withRotationalRate(-joystick.getRightX() * MaxAngularRate) // Drive counterclockwise with negative X (left)
        ));

        
    //joystick.a().whileTrue(drivetrain.applyRequest(() -> brake));
    //joystick.b().whileTrue(drivetrain
       // .applyRequest(() -> point.withModuleDirection(new Rotation2d(-joystick.getLeftY(), -joystick.getLeftX()))));

    // reset the field-centric heading on left bumper press
    UpDpad.onTrue(drivetrain.runOnce(() -> drivetrain.seedFieldRelative()));

    if (Utils.isSimulation()) {
      drivetrain.seedFieldRelative(new Pose2d(new Translation2d(), Rotation2d.fromDegrees(90)));
    }
    drivetrain.registerTelemetry(logger::telemeterize);


    //moves shooter to predetermined positions
    //mShooterSubsystem.setDefaultCommand(new AimShooter(mShooterSubsystem));
    
    //runs shooter at predetermined speeds after enabling with the button press
    //Start.toggleOnTrue(new ShooterToggle());
    A.toggleOnTrue(new MoveIntake());

    //lowers intake and starts running the intake. Will stop after collecting a note. Will retract after collecting a note
    //leftButton.toggleOnTrue(new intakeToggle());

    //runs the intake in a way that would spit back out a piece
    righButton.onTrue(new ReverseIntake());
    
    //moves the elevator up and positions the shooter to score in the amp
    X.onTrue(new MoveElevator(19.3, 23));//19.3, Note to future self: change this command to a sequential command group

    //moves the elevator down and positions the shooter to grab a piece
    Y.onTrue(new MoveElevator(0.1, 6.5));

    //Stores the shooter to a position where the robot can go under the stage
    B.onTrue(new StoreShooter());

    //Runs the Shooter
    downDpad.toggleOnTrue(new ShooterToggle());

    //Sets the intake in the store position
    Menu.onTrue(new StoreIntake());


    Start.toggleOnTrue(drivetrain.applyRequest(()-> drive.withVelocityY(-joystick.getLeftX() * MaxSpeed).withVelocityX(-joystick.getLeftY() * MaxSpeed).withRotationalRate(-Limelight.txSlowly())));
  }





  public RobotContainer() {
    NamedCommands.registerCommand("intakeToggle", new intakeToggle());
    NamedCommands.registerCommand("ShooterToggle", new ShooterToggle());
    configureBindings();
    
    
   // autoChooser = AutoBuilder.buildAutoChooser();
    //SmartDashboard.putData("Auto mode", autoChooser);
  }

  public Command getAutonomousCommand() {
    return Commands.print("No autonomous command configured");
  }
}
