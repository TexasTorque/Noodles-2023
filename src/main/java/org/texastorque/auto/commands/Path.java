/**
 * Copyright 2023 Texas Torque.
 * 
 * This file is part of Swerve-2023, which is not licensed for distribution.
 * For more details, see ./license.txt or write <jus@justusl.com>.
 */
package org.texastorque.auto.commands;

import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.PathPlannerTrajectory.PathPlannerState;
import edu.wpi.first.math.controller.HolonomicDriveController;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import org.texastorque.Subsystems;
import org.texastorque.auto.sequences.Drive;
import org.texastorque.subsystems.Drivebase;
import org.texastorque.torquelib.auto.TorqueCommand;
import org.texastorque.torquelib.control.TorquePID;

public final class Path extends TorqueCommand implements Subsystems {
    private final PIDController xController = TorquePID.create(3).build();
    private final PIDController yController = TorquePID.create(3).build();
    // private final PIDController xController = new PIDController(1, 0, 0);
    // private final PIDController yController = new PIDController(1, 0, 0);

    private final ProfiledPIDController thetaController;
    private final HolonomicDriveController controller;

    private final PathPlannerTrajectory trajectory;
    private final Timer timer = new Timer();
    private final boolean resetOdometry;

    public Path(final String name) {
        this(name, false);
    }

    public Path(final String name, final boolean reset) {
        this(name, reset, Drivebase.MAX_VELOCITY, Drivebase.MAX_ACCELERATION);
    }

    public Path(final String name, final boolean reset, final double maxSpeed, final double maxAcceleration) {
        trajectory = PathPlanner.loadPath(name, maxSpeed, maxAcceleration);
        this.resetOdometry = reset;

        thetaController = new ProfiledPIDController(Math.PI * 2, 0, 0, new TrapezoidProfile.Constraints(6 * Math.PI, 
                    6 * Math.PI));
        thetaController.enableContinuousInput(-Math.PI, Math.PI);
        controller = new HolonomicDriveController(xController, yController, thetaController);
    }

    @Override
    protected final void init() {
        timer.reset();
        timer.start();
        drivebase.isFieldOriented = false;
        if (!resetOdometry) return;
        drivebase.resetPose(extractInitialPose(trajectory));
    }

    @Override
    protected final void continuous() {
        final PathPlannerState current = (PathPlannerState)trajectory.sample(timer.get());
        ChassisSpeeds speeds = controller.calculate(drivebase.getPose(), current, current.holonomicRotation);
        speeds = new ChassisSpeeds(-speeds.vxMetersPerSecond, -speeds.vyMetersPerSecond, speeds.omegaRadiansPerSecond);
        SmartDashboard.putNumber("ARCS-X", speeds.vxMetersPerSecond);
        SmartDashboard.putNumber("ARCS-Y", speeds.vyMetersPerSecond);
        SmartDashboard.putNumber("ARCS-R", speeds.omegaRadiansPerSecond);

        drivebase.inputSpeeds = speeds;
    }

    @Override
    protected final boolean endCondition() {
        return timer.hasElapsed(trajectory.getTotalTimeSeconds());
    }

    @Override
    protected final void end() {
        timer.stop();
        drivebase.inputSpeeds = new ChassisSpeeds();
    }

    private static final Pose2d extractInitialPose(final PathPlannerTrajectory trajectory) {
        final PathPlannerState state = trajectory.getInitialState();
        return new Pose2d(state.poseMeters.getTranslation(), state.holonomicRotation); 
    }
}
