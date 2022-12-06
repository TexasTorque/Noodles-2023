/**
 * Copyright 2023 Texas Torque.
 * 
 * This file is part of Swerve-2023, which is not licensed for distribution.
 * For more details, see ./license.txt or write <jus@justusl.com>.
 */
package org.texastorque.auto;

import org.texastorque.auto.sequences.Drive;
import org.texastorque.torquelib.auto.TorqueAutoManager;

public final class AutoManager extends TorqueAutoManager {
    public static volatile AutoManager instance; 

    private AutoManager() { super(); }

    @Override
    public void init() {
        addSequence("Drive Test", new Drive());

    }

    public static synchronized final AutoManager getInstance() {
        return instance == null ? instance = new AutoManager() : instance;
    }
}
