/**
 * Copyright 2023 Texas Torque.
 * 
 * This file is part of Swerve-2023, which is not licensed for distribution.
 * For more details, see ./license.txt or write <jus@justusl.com>.
 */
package org.texastorque.auto.commands;

import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.PathPlannerTrajectory.EventMarker;
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

import java.util.ArrayList;
import java.util.HashMap;
import java.util.List;
import java.util.Map;

import org.texastorque.Subsystems;
import org.texastorque.subsystems.Drivebase;
import org.texastorque.torquelib.auto.TorqueCommand;
import org.texastorque.torquelib.control.TorquePID;

public final class EventPath extends TorqueCommand implements Subsystems {
    private final PIDController xController = TorquePID.create(1).build();
    private final PIDController yController = TorquePID.create(1).build();

    private final ProfiledPIDController thetaController =
            new ProfiledPIDController(0.01, 0, 0, new TrapezoidProfile.Constraints(Math.PI, 
                    Math.PI));
    private final HolonomicDriveController controller =
            new HolonomicDriveController(xController, yController, thetaController);

    private final PathPlannerTrajectory trajectory;
    private final Timer timer = new Timer();
    private final boolean resetOdometry;

    private final List<EventMarker> unpassed, events;
    private final Map<String, TorqueCommand> commands;
    private final List<TorqueCommand> running;

    public EventPath(final String name, final boolean reset, final double maxSpeed, final double maxAcceleration) {
        this(name, new HashMap<String, TorqueCommand>(), reset, maxSpeed, maxAcceleration);
    }

    public EventPath(final String name, final Map<String, TorqueCommand> commands, final boolean reset, final double maxSpeed, final double maxAcceleration) {
        thetaController.enableContinuousInput(-Math.PI, Math.PI);
        trajectory = PathPlanner.loadPath(name, maxSpeed, maxAcceleration);
        events = trajectory.getMarkers();
        unpassed = new ArrayList<EventMarker>();
        this.commands = commands;
        this.resetOdometry = reset;
        running = new ArrayList<TorqueCommand>();
    }

    @Override
    protected final void init() {
        timer.reset();
        timer.start();
        if (!resetOdometry) return;
        drivebase.isFieldOriented = false;
        drivebase.resetPose(trajectory.getInitialPose());
        unpassed.clear();
        unpassed.addAll(events);
        running.clear();
    }

    @Override
    protected final void continuous() {
        final double elapsed = timer.get();
       
        final PathPlannerState current = (PathPlannerState)trajectory.sample(elapsed);

        ChassisSpeeds speeds = controller.calculate(drivebase.getPose(), current, current.holonomicRotation);
        speeds = new ChassisSpeeds(-speeds.vxMetersPerSecond, -speeds.vyMetersPerSecond, speeds.omegaRadiansPerSecond);

        SmartDashboard.putNumber("ARCS-X", speeds.vxMetersPerSecond);
        SmartDashboard.putNumber("ARCS-Y", speeds.vyMetersPerSecond);
        SmartDashboard.putNumber("ARCS-R", speeds.omegaRadiansPerSecond);

        drivebase.inputSpeeds = speeds; 

        if (unpassed.size() > 0 && elapsed >= unpassed.get(0).timeSeconds) {
            final EventMarker marker = unpassed.remove(0);
            for (final String name : marker.names) {
                final TorqueCommand command = commands.getOrDefault(name, null);
                if (command != null)
                    running.add(command);
            }
        }

        for (final TorqueCommand command : running)
            if (command.run())
                running.remove(command);
    }

    @Override
    protected final boolean endCondition() {
        return timer.hasElapsed(trajectory.getTotalTimeSeconds());
    }

    @Override
    protected final void end() {
        timer.stop();
        for (final TorqueCommand command : running)
            command.reset();
        drivebase.inputSpeeds = new ChassisSpeeds();
    }

    public void addEvent(final String name, final TorqueCommand command) {
        commands.put(name, command);
    }
}
