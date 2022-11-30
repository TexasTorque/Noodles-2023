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
