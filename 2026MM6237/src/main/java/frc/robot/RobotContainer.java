// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.controllers.DriverController;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.LimelightSubsystem;

import com.pathplanner.lib.auto.AutoBuilder;

import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
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

    private final CommandXboxController driver = new CommandXboxController(Constants.OperatorConstants.kDriverControllerPort);
    // private final CommandXboxController operator = new CommandXboxController(Constants.OperatorConstants.kOperatorControllerPort);

    public final CommandSwerveDrivetrain drivetrain = TunerConstants.createDrivetrain();
    private final SendableChooser<Command> autoChooser;

    private final LimelightSubsystem limelight = new LimelightSubsystem();

    /** The container for the robot. Contains subsystems, OI devices, and commands. */
    public RobotContainer() {
      // Configure AutoBuilder before building the auto chooser
      drivetrain.configureAutoBuilder();
      
      // Set Limelight subsystem to always update (default command that does nothing but requires the subsystem)
      limelight.setDefaultCommand(
        Commands.runEnd(() -> {}, () -> {}, limelight)
          .withName("LimelightIdle")
      );
      
      // Configure the trigger bindings
      configureBindings();
      autoChooser = AutoBuilder.buildAutoChooser("DefaultAuto");
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
      DriverController.mapXboxController(driver, drivetrain, limelight);

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
