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
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import org.texastorque.Subsystems;
import org.texastorque.subsystems.Drivebase;
import org.texastorque.torquelib.auto.TorqueCommand;
import org.texastorque.torquelib.control.TorquePID;
import org.texastorque.torquelib.util.TorqueUtil;

public final class Zero extends TorqueCommand implements Subsystems {
    private double start = 0;

    @Override
    protected final void init() {
        drivebase.isZeroingModules = true;
        start = TorqueUtil.time();
    }

    @Override
    protected final void continuous() {
    }

    @Override
    protected final boolean endCondition() {
        return TorqueUtil.time() - start > 2;
    }

    @Override
    protected final void end() {
        drivebase.isZeroingModules = false;
    }
}
