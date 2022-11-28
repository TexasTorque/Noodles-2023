/**
 * Copyright 2022 Texas Torque.
 *
 * This file is part of Paddlefoot-2022, which is not licensed for distribution.
 * For more details, see ./license.txt or write <jus@gtsbr.org>.
 */
package org.texastorque;

import org.texastorque.torquelib.auto.TorqueAutoManager.TorqueBlankAutoManager;
import org.texastorque.torquelib.base.*;

public final class Robot extends TorqueRobotBase implements Subsystems {
    public Robot() {
        super(Input.getInstance(), new TorqueBlankAutoManager());

        addSubsystem(testbench);
    }
}
