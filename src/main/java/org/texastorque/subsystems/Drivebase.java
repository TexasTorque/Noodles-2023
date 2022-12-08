/**
 * Copyright 2023 Texas Torque.
 * 
 * This file is part of Swerve-2023, which is not licensed for distribution.
 * For more details, see ./license.txt or write <jus@justusl.com>.
 */
package org.texastorque.subsystems;

import java.util.Map;

import org.texastorque.Subsystems;
import org.texastorque.torquelib.base.TorqueMode;
import org.texastorque.torquelib.base.TorqueSubsystem;
import org.texastorque.torquelib.control.TorquePID;
import org.texastorque.torquelib.modules.TorqueSwerveModule2022;
import org.texastorque.torquelib.modules.TorqueSwerveModule2022.TorqueSwerveModuleConfiguration;
import org.texastorque.torquelib.sensors.TorqueNavXGyro;
import org.texastorque.torquelib.util.TorqueLog;
import org.texastorque.torquelib.util.TorqueUtil;

import edu.wpi.first.math.MatBuilder;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.Nat;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.Vector;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.numbers.N5;
import edu.wpi.first.math.numbers.N7;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.shuffleboard.WidgetType;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/**
 * The swerve drivebase subsystem.
 * 
 * The rotation of the drivebase is [0, 2π) radians counterclockwise, 
 * with 0 being straight ahead. 
 * 
 *     0 or 2π
 *        ↑
 *  π/2 ← * → 3π/2
 *        ↓
 *        π
 *
 * -- States --
 * FIELD_RELATIVE  Field relative mode, where the robot's heading is relative to the field.
 * ROBOT_RELATIVE  Robot relative mode, where the robot's heading is relative to itself.
 * ZERO            Sets all encoders back to zero position.
 * 
 * -- Fields --
 * ChassisSpeeds   inputSpeeds       The requested speeds (x, y, r).
 * boolean         isRotationLocked  Whether or not the robot's rotation is locked.
 * 
 */
public final class Drivebase extends TorqueSubsystem implements Subsystems {
    private static volatile Drivebase instance;

    public static final double
            WIDTH = Units.inchesToMeters(21.745), // m (swerve to swerve)
            LENGTH = Units.inchesToMeters(21.745), // m (swerve to swerve)

            MAX_VELOCITY = 4.0, // m/s
            MAX_ACCELERATION = 2.0, // m/s^2
            MAX_ANGULAR_VELOCITY = 2.0 * Math.PI, // rad/s
            MAX_ANGULAR_ACCELERATION = 2.0 * Math.PI, // rad/s^2
            WHEEL_DIAMETER = Units.inchesToMeters(4.0), // m

            MAGIC_NUMBER = 34;

    public static final Pose2d INITIAL_POS = new Pose2d(2.0, 2.0, new Rotation2d(0.0));

    private final Translation2d 
            LOC_FL = new Translation2d(-11.815, -12.059), // (+, +)
            LOC_FR = new Translation2d(-11.765, 12.057),  // (+, -)
            LOC_BL = new Translation2d(11.734, -12.025),  // (-, +)
            LOC_BR = new Translation2d(11.784, 12.027);   // (-, -)

    public static enum State {
        FIELD_RELATIVE, ROBOT_RELATIVE, ZERO;
    }

    // This is the kinematics object that calculates the desired wheel speeds
    private final SwerveDriveKinematics kinematics;

    // private final TorqueSwerveOdometry odometry;

    // PoseEstimator is a more advanced odometry system that uses a Kalman filter to estimate the robot's position
    // It also encorporates other measures like April tag positions
    private final SwerveDrivePoseEstimator poseEstimator;
    private final Field2d fieldMap = new Field2d();
    
    // Matrix constants for the pose estimator.
    private static final Matrix<N3, N1> STATE_STDS = new MatBuilder<>(Nat.N3(), Nat.N1()).fill(0.05, 0.05, 0.02);
    private static final Matrix<N1, N1> LOCAL_STDS = new MatBuilder<>(Nat.N1(), Nat.N1()).fill(0.01);
    private static final Matrix<N3, N1> VISION_STDS = new MatBuilder<>(Nat.N3(), Nat.N1()).fill(0.025, 0.025, 0.025);

    // Alternate matricies for the pose estimator.

    /**
     * Standard deviations of model states. Increase these numbers to trust your model's state estimates less. This
     * matrix is in the form [x, y, theta, s_0, ... s_n]ᵀ, with units in meters and radians, then meters.
     */
    // private static final Vector<N7> STATE_STDS = VecBuilder.fill(0.05, 0.05, Units.degreesToRadians(5), 0.05, 0.05, 0.05, 0.05);
    
    /**
     * Standard deviations of the encoder and gyro measurements. Increase these numbers to trust sensor readings from
     * encoders and gyros less. This matrix is in the form [theta, s_0, ... s_n], with units in radians followed by meters.
     */
    // private static final Vector<N5> LOCAL_STDS = VecBuilder.fill(Units.degreesToRadians(0.01), 0.01, 0.01, 0.01, 0.01);
    
    /**
     * Standard deviations of the vision measurements. Increase these numbers to trust global measurements from vision
     * less. This matrix is in the form [x, y, theta]ᵀ, with units in meters and radians.
     */
    // private static final Vector<N3> VISION_STDS = VecBuilder.fill(0.5, 0.5, Units.degreesToRadians(10));

    // The instances of the swerve modules.
    private final TorqueSwerveModule2022 fl, fr, bl, br;

    // The instance of the NavX gyro.
    private final TorqueNavXGyro gyro = TorqueNavXGyro.getInstance();

    // Internal state variables.
    private double lastRotationRadians;
    private final PIDController rotationalPID;
    private SwerveModuleState[] swerveStates;

    // Fields that store the state of the subsystem 
    public State state = State.FIELD_RELATIVE;
    public ChassisSpeeds inputSpeeds = new ChassisSpeeds(0, 0, 0);
    public boolean isRotationLocked = false;

    /**
     * Constructor called on initialization.
     */
    private Drivebase() {
        // Configure the subsystem ShuffleboardTab

        // Configure the rotational lock PID.
        rotationalPID = TorquePID.create(0.02 / 4.0 / 180 * Math.PI).addDerivative(.001).build();
        rotationalPID.enableContinuousInput(0, Math.PI);
        lastRotationRadians = gyro.getRotation2d().getRadians();

        final var config = TorqueSwerveModuleConfiguration.defaultConfig;

        // Configure the swerve modules based on the drivebase constants.
        config.maxVelocity = MAX_VELOCITY;
        config.maxAcceleration = MAX_ACCELERATION;
        config.maxAngularVelocity = MAX_ANGULAR_VELOCITY;
        config.maxAngularAcceleration = MAX_ANGULAR_ACCELERATION;

        // Configure all the swerve modules            Drive|Turn|Encoder|Offset
        fl = new TorqueSwerveModule2022("Front Left",  3,    4,   10, 0.0,  config);
        fr = new TorqueSwerveModule2022("Front Right", 5,    6,   11, 0.0, config);
        bl = new TorqueSwerveModule2022("Back Left",   1,    2,   9, 0.0,   config);
        br = new TorqueSwerveModule2022("Back Right",  7,    8,   12, 0.0, config);
        // The offsets need to be found experimentally.
        // With no power being set to the module position the wheel 100% straight ahead 
        // and the offset is the reading of the cancoder.
        // This is used when the module is in absolute mode so we dont ever have to line it up.

        // Configure the kinematics and poseEstimator objects.
        kinematics = new SwerveDriveKinematics(LOC_BL, LOC_BR, LOC_FL, LOC_FR);

        poseEstimator = new SwerveDrivePoseEstimator(gyro.getHeadingCCW(),
                INITIAL_POS, kinematics, STATE_STDS,
                LOCAL_STDS, VISION_STDS);

        SmartDashboard.putData("Est. Map", fieldMap);
    }

    /**
     * On change from auto to teleop and back.
     */
    @Override
    public final void initialize(final TorqueMode mode) {
        mode.onAuto(() -> state = State.ROBOT_RELATIVE);
        mode.onTeleop(() -> state = State.FIELD_RELATIVE);
    }

    /**
     * Updates the feedback systems like poseEstimator
     * and stuff and logs to SmartDashboard and Shuffleboard.
     */
    private void updateFeedback() { 
        SmartDashboard.putNumber("Gyro Angle", gyro.getHeadingCCW().getRadians());

        poseEstimator.update(gyro.getHeadingCCW(), fl.getState(), fr.getState(), bl.getState(), br.getState());

        fieldMap.setRobotPose(poseEstimator.getEstimatedPosition());

        final Translation2d estTranslation = poseEstimator.getEstimatedPosition().getTranslation();

        final String poseLog = TorqueUtil.group(2, 3, estTranslation.getX(), estTranslation.getY());
        log.log("Est. Pos", poseLog, 3, 1); // can infer text

        final String speedsLog = TorqueUtil.group(2, 3, inputSpeeds.vxMetersPerSecond, 
                inputSpeeds.vyMetersPerSecond, inputSpeeds.omegaRadiansPerSecond);
        log.log("Input. Speeds.", speedsLog, 3, 1); // can infer text

        log.log("Rot. Locked", isRotationLocked, 1, 1); // can infer bool

        log.log("Gyro Rad.", gyro.getHeadingCCW().getRadians(), 2, 1, TorqueLog.W_TEXT);

        log.log("Gyro Dial.", gyro.getHeadingCCW().getDegrees(), 2, 2, TorqueLog.W_GYRO); 
    }

    /**
     * Called every loop.
     * 
     * 1. Update feedback.
     * 2. Check state.
     * 3. Execute state base on input parameters.
     */
    @Override
    public final void update(final TorqueMode mode) {
        updateFeedback();

        SmartDashboard.putString("Input. Speeds.", TorqueUtil.group(2, 3, 
                inputSpeeds.vxMetersPerSecond, inputSpeeds.vyMetersPerSecond, inputSpeeds.omegaRadiansPerSecond));

        if (state == State.ZERO) {
            fl.zero();
            fr.zero();
            bl.zero();
            br.zero();
            return;
        }

        // Calculate the locked rotation with the PID.
        final double realRotationRadians = gyro.getRotation2d().getRadians();

        // if (isRotationLocked && Math.abs(inputSpeeds.omegaRadiansPerSecond) == 0)
        //     inputSpeeds.omegaRadiansPerSecond = rotationalPID.calculate(
        //             realRotationRadians, lastRotationRadians);
        // else lastRotationRadians = realRotationRadians;

        // // Calculate field relative vectors.
        // if (state == State.ROBOT_RELATIVE)
        //         inputSpeeds = ChassisSpeeds.fromFieldRelativeSpeeds(
        //                 inputSpeeds.vxMetersPerSecond,
        //                 inputSpeeds.vyMetersPerSecond,
        //                 inputSpeeds.omegaRadiansPerSecond,
        //                 gyro.getHeadingCCW());  
        //                 // Or just get counter clockwise LMAO
        
        // Convert robot vectors to module vectors.
        swerveStates = kinematics.toSwerveModuleStates(inputSpeeds);

        SwerveDriveKinematics.desaturateWheelSpeeds(swerveStates, MAX_VELOCITY);

        fl.setDesiredState(swerveStates[0]);
        fr.setDesiredState(swerveStates[1]);
        bl.setDesiredState(swerveStates[2]);
        br.setDesiredState(swerveStates[3]);
    }

    // Interfacing with the robot position estimator.

    public void resetPose(final Pose2d pose) {
        gyro.reset();
        gyro.setOffsetCCW(pose.getRotation());
        poseEstimator.resetPosition(pose, gyro.getHeadingCCW());
    }

    public void resetPose(final Rotation2d rotation) {
        resetPose(new Pose2d(getPose().getTranslation(), rotation));
    }

    public void resetPose(final Translation2d translation) {
        resetPose(new Pose2d(translation, gyro.getHeadingCCW()));
    }

    public Pose2d getPose() {
        updateFeedback();
        return poseEstimator.getEstimatedPosition();
    }

    public static synchronized final Drivebase getInstance() {
        return instance == null ? instance = new Drivebase() : instance;
    }
}
