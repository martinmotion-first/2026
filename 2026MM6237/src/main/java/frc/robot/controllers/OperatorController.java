package frc.robot.controllers;

import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants;
import frc.robot.subsystems.Feeder;
import frc.robot.subsystems.Floor;
import frc.robot.subsystems.Hanger;
import frc.robot.subsystems.Hood;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Shooter;

/**
 * Operator Controller mapping for testing and exercising subsystems.
 * 
 * Provides simple voltage control for all subsystems with intuitive button mappings:
 * - Left stick for continuous control
 * - Triggers for directional control
 * - Bumpers and face buttons for specific subsystems
 * 
 * Button Mapping:
 * - LB: Feeder forward (positive voltage)
 * - LT: Feeder reverse (negative voltage)
 * - RB: Shooter forward (positive voltage)
 * - RT: Shooter reverse (negative voltage)
 * - Y: Intake forward (rollers)
 * - X: Intake reverse (rollers)
 * - B: Intake pivot up
 * - A: Intake pivot down
 * - Left Stick Up/Down: Hood position (analog, 0.0-1.0)
 * - DPad Up: Hanger extend (positive voltage)
 * - DPad Down: Hanger retract (negative voltage)
 * - DPad Left: Floor feed forward
 * - DPad Right: Floor feed reverse
 */
public class OperatorController {
    
    // Control percentages for safe voltage testing
    private static final double MOTOR_SPEED_PERCENT = 0.3;  // 30% voltage for testing
    
    /**
     * Maps Xbox controller inputs to subsystem commands.
     * 
     * @param operatorController The Xbox controller for the operator
     * @param feeder The Feeder subsystem
     * @param shooter The Shooter subsystem
     * @param intake The Intake subsystem
     * @param hood The Hood subsystem
     * @param hanger The Hanger subsystem
     * @param floor The Floor subsystem
     */
    public static void mapXboxController(
            CommandXboxController operatorController,
            Feeder feeder,
            Shooter shooter,
            Intake intake,
            Hood hood,
            Hanger hanger,
            Floor floor) {
        
        // ======================== FEEDER CONTROLS ========================
        // Left Bumper: Feeder forward
        operatorController.leftBumper()
            .whileTrue(feeder.runEnd(
                () -> feeder.setPercentOutput(MOTOR_SPEED_PERCENT),
                () -> feeder.setPercentOutput(0.0)
            ).withName("Feeder Forward"));
        
        // Left Trigger: Feeder reverse
        new Trigger(() -> operatorController.getLeftTriggerAxis() > Constants.OperatorConstants.kTriggerButtonThreshold)
            .whileTrue(feeder.runEnd(
                () -> feeder.setPercentOutput(-MOTOR_SPEED_PERCENT),
                () -> feeder.setPercentOutput(0.0)
            ).withName("Feeder Reverse"));
        
        // ======================== SHOOTER CONTROLS ========================
        // Right Bumper: Shooter forward
        operatorController.rightBumper()
            .whileTrue(shooter.runEnd(
                () -> shooter.setPercentOutput(MOTOR_SPEED_PERCENT),
                () -> shooter.setPercentOutput(0.0)
            ).withName("Shooter Forward"));
        
        // Right Trigger: Shooter reverse
        new Trigger(() -> operatorController.getRightTriggerAxis() > Constants.OperatorConstants.kTriggerButtonThreshold)
            .whileTrue(shooter.runEnd(
                () -> shooter.setPercentOutput(-MOTOR_SPEED_PERCENT),
                () -> shooter.setPercentOutput(0.0)
            ).withName("Shooter Reverse"));
        
        // ======================== INTAKE CONTROLS ========================
        // Y Button: Intake forward (rollers spin, pivot goes to INTAKE position)
        operatorController.y()
            .whileTrue(intake.runEnd(
                () -> {
                    intake.set(Intake.Speed.INTAKE);
                    intake.set(Intake.Position.INTAKE);
                },
                () -> intake.set(Intake.Speed.STOP)
            ).withName("Intake Forward"));
        
        // X Button: Intake reverse (rollers reverse, pivot goes to STOWED)
        operatorController.x()
            .whileTrue(intake.runEnd(
                () -> {
                    intake.set(Intake.Speed.STOP);
                    intake.set(Intake.Position.STOWED);
                },
                () -> intake.set(Intake.Speed.STOP)
            ).withName("Intake Stow"));
        
        // B Button: Intake agitate command
        operatorController.b()
            .onTrue(intake.agitateCommand().withName("Intake Agitate"));
        
        // A Button: Intake homing command  
        operatorController.a()
            .onTrue(intake.homingCommand().withName("Intake Homing"));
        
        // ======================== HOOD CONTROLS ========================
        // NOTE: Hood does NOT have a default command to prevent unwanted servo movement on robot enable.
        // When controller sticks are not centered, having a default command would cause the hood
        // to move immediately upon enable, potentially causing mechanical damage.
        // 
        // For future implementation: Add explicit button/trigger-based controls instead:
        // Example:
        // operatorController.leftStick()
        //     .whileTrue(hood.run(() -> hood.setPosition(newPosition)));
        //
        // IMPORTANT: Never use .setDefaultCommand() on subsystems that directly command motors!
        // Default commands run immediately on enable without any operator input validation.
        
        // ======================== HANGER CONTROLS ========================
        // DPad Up: Hanger extend (positive voltage)
        operatorController.pov(0)
            .whileTrue(hanger.runEnd(
                () -> hanger.setPercentOutput(MOTOR_SPEED_PERCENT),
                () -> hanger.setPercentOutput(0.0)
            ).withName("Hanger Extend"));
        
        // DPad Down: Hanger retract (negative voltage)
        operatorController.pov(180)
            .whileTrue(hanger.runEnd(
                () -> hanger.setPercentOutput(-MOTOR_SPEED_PERCENT),
                () -> hanger.setPercentOutput(0.0)
            ).withName("Hanger Retract"));
        
        // ======================== FLOOR CONTROLS ========================
        // DPad Left: Floor feed forward
        operatorController.pov(270)
            .whileTrue(floor.runEnd(
                () -> floor.set(Floor.Speed.FEED),
                () -> floor.set(Floor.Speed.STOP)
            ).withName("Floor Feed Forward"));
        
        // DPad Right: Floor manual control (reverse by directly controlling motor)
        // Since Floor doesn't have reverse in Speed enum, we'll stop and note this limitation
        operatorController.pov(90)
            .whileTrue(floor.runEnd(
                () -> floor.set(Floor.Speed.STOP),
                () -> floor.set(Floor.Speed.STOP)
            ).withName("Floor Stop"));
    }
}
