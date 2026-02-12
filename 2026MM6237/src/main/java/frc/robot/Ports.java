package frc.robot;

import com.ctre.phoenix6.CANBus;

public final class Ports {
    // CAN Buses
    public static final CANBus kRoboRioCANBus = new CANBus("rio");
    public static final CANBus kCANivoreCANBus = new CANBus("main");

    // Talon FX IDs
    public static final int kIntakePivot = 10;
    public static final int kIntakeRollers = 11;
    public static final int kFloor = 12;
    public static final int kFeeder = 13;
    public static final int kShooterLeft = 14;
    public static final int kShooterMiddle = 15;
    public static final int kShooterRight = 16;
    public static final int kHanger = 18;

    // ======================== SWERVE DRIVE MOTOR IDs ========================
    // Front Left Module
    public static final int kFrontLeftDriveMotor = 8;
    public static final int kFrontLeftSteerMotor = 7;
    public static final int kFrontLeftEncoder = 15;

    // Front Right Module
    public static final int kFrontRightDriveMotor = 10;
    public static final int kFrontRightSteerMotor = 12;
    public static final int kFrontRightEncoder = 17;

    // Back Left Module
    public static final int kBackLeftDriveMotor = 9;
    public static final int kBackLeftSteerMotor = 11;
    public static final int kBackLeftEncoder = 18;

    // Back Right Module
    public static final int kBackRightDriveMotor = 14;
    public static final int kBackRightSteerMotor = 13;
    public static final int kBackRightEncoder = 16;

    // Gyro/Pigeon 2
    public static final int kPigeon2Id = 19;

    // PWM Ports
    public static final int kHoodLeftServo = 3;
    public static final int kHoodRightServo = 4;
}
