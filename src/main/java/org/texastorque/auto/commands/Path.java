/**
 * Copyright 2022 Texas Torque.
 *
 * This file is part of Clutch-2022, which is not licensed for distribution.
 * For more details, see ./license.txt or write <jus@gtsbr.org>.
 */
package org.texastorque.auto.commands;

import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.PathPlannerTrajectory.PathPlannerState;
import edu.wpi.first.math.controller.HolonomicDriveController;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.Timer;

import org.texastorque.Subsystems;
import org.texastorque.subsystems.Drivebase;
import org.texastorque.torquelib.auto.TorqueCommand;
import org.texastorque.torquelib.control.TorquePID;
import org.texastorque.torquelib.sensors.TorqueNavXGyro;

public final class Path extends TorqueCommand implements Subsystems {
    private final PIDController xController = TorquePID.create(1).build();
    private final PIDController yController = TorquePID.create(1).build();
    // private final PIDController xController = new PIDController(1, 0, 0);
    // private final PIDController yController = new PIDController(1, 0, 0);

    private final ProfiledPIDController thetaController =
            new ProfiledPIDController(4, 0, 0, new TrapezoidProfile.Constraints(Drivebase.MAX_ANGULAR_VELOCITY, 
                    Drivebase.MAX_ANGULAR_ACCELERATION));
    private final HolonomicDriveController controller =
            new HolonomicDriveController(xController, yController, thetaController);

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
        thetaController.enableContinuousInput(-Math.PI, Math.PI);
        trajectory = PathPlanner.loadPath(name, maxSpeed, maxAcceleration);
        this.resetOdometry = reset;
    }

    @Override
    protected final void init() {
        timer.reset();
        timer.start();
        if (!resetOdometry) return;
        drivebase.state = Drivebase.State.FIELD_RELATIVE;
        TorqueNavXGyro.getInstance().setAngleOffset(360 - trajectory.getInitialPose().getRotation().getDegrees());
        drivebase.getPoseEstimator().resetPosition(trajectory.getInitialState().poseMeters,
                                              trajectory.getInitialState().holonomicRotation);
    }

    @Override
    protected final void continuous() {
        final PathPlannerState current = (PathPlannerState)trajectory.sample(timer.get());
        ChassisSpeeds speeds =
                controller.calculate(drivebase.getPose(), current, current.holonomicRotation);
        System.out.println(speeds);
        speeds = new ChassisSpeeds(-speeds.vxMetersPerSecond, speeds.vyMetersPerSecond, speeds.omegaRadiansPerSecond);
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
}
