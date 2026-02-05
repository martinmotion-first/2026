// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.Constants.OperatorConstants;
import frc.robot.controllers.DriverController;
import frc.robot.controllers.OperatorController;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.CommandSwerveDrivetrainOld;
import frc.robot.subsystems.ExampleSubsystem;
import frc.robot.subsystems.Feeder;
import frc.robot.subsystems.Floor;
import frc.robot.subsystems.Hanger;
import frc.robot.subsystems.Hood;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.LimelightSubsystem6237;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Swerve;

import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.RotationsPerSecond;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;
import com.pathplanner.lib.auto.AutoBuilder;

import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {
    // The robot's subsystems and commands are defined here...
    private final ExampleSubsystem m_exampleSubsystem = new ExampleSubsystem();

    private final Swerve swerve = new Swerve();
    private final Intake intake = new Intake();
    private final Floor floor = new Floor();
    private final Feeder feeder = new Feeder();
    private final Shooter shooter = new Shooter();
    private final Hood hood = new Hood();
    private final Hanger hanger = new Hanger();

    private final CommandXboxController driver = new CommandXboxController(Constants.OperatorConstants.kDriverControllerPort);
    // private final CommandXboxController operator = new CommandXboxController(Constants.OperatorConstants.kOperatorControllerPort);
    private final CommandXboxController operator =new CommandXboxController(Constants.OperatorConstants.kOperatorControllerPort);

    public final CommandSwerveDrivetrainOld drivetrain = TunerConstants.createDrivetrain();
      private final SendableChooser<Command> autoChooser;

    // Replace with CommandPS4Controller or CommandJoystick if needed
    private final CommandXboxController m_driverController =
        new CommandXboxController(OperatorConstants.kDriverControllerPort);

    private static final SwerveRequest.FieldCentric tempDrive = new SwerveRequest.FieldCentric()
      .withDeadband(TunerConstants.kSpeedAt12Volts.in(MetersPerSecond)).withRotationalDeadband(RotationsPerSecond.of(0.5).in(RadiansPerSecond))
      .withDriveRequestType(DriveRequestType.OpenLoopVoltage); // Use open-loop control for drive motors

    private final LimelightSubsystem6237 limelight = new LimelightSubsystem6237();

    /** The container for the robot. Contains subsystems, OI devices, and commands. */
    public RobotContainer() {
      // Configure the trigger bindings
      configureBindings();
      autoChooser = AutoBuilder.buildAutoChooser("DefaultPath");
      SmartDashboard.putData("Auto Mode", autoChooser);
    }


    /**
     * Use this method to define your trigger->command mappings. Triggers can be created via the
     * {@link Trigger#Trigger(java.util.function.BooleanSupplier)} constructor with an arbitrary
     * predicate, or via the named factories in {@link
     * edu.wpi.first.wpilibj2.command.button.CommandGenericHID}'s subclasses for {@link
     * CommandXboxController Xbox}/{@link edu.wpi.first.wpilibj2.command.button.CommandPS4Controller
     * PS4} controllers or {@link edu.wpi.first.wpilibj2.command.button.CommandJoystick Flight
     * joysticks}.
     */
    private void configureBindings() {
      // DriverMapping6237MR.mapXboxController(driver, drivetrain, NetworkTableInstance.getDefault().getTable("limelight"));
      DriverController.mapXboxController(driver, drivetrain, null);
      OperatorController.mapXboxController(operator, feeder, shooter, intake, hood, hanger, floor);
      // Schedule `exampleMethodCommand` when the Xbox controller's B button is pressed,
      // cancelling on release.

        // Command defaultDrivetrainCommand = drivetrain.applyRequest(() ->
        //     tempDrive.withVelocityX(m_driverController.getLeftY() * TunerConstants.kSpeedAt12Volts.in(MetersPerSecond)) 
        //         .withVelocityY(m_driverController.getLeftX() * TunerConstants.kSpeedAt12Volts.in(MetersPerSecond)) 
        //         .withRotationalRate(-1 * m_driverController.getRightX() * RotationsPerSecond.of(0.5).in(RadiansPerSecond))
        // ); 

        // drivetrain.setDefaultCommand(
        //     // Drivetrain will execute this command periodically
        //     defaultDrivetrainCommand
        // );
    }

    /**
     * Use this to pass the autonomous command to the main {@link Robot} class.
     *
     * @return the command to run in autonomous
     */
    public Command getAutonomousCommand() {
      return autoChooser.getSelected();
      // return new Command(){};
    }

    public void getSimPeriodic(Field2d field) {
      field.setRobotPose(drivetrain.getState().Pose);
    }
}
