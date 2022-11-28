package org.texastorque.auto;

import org.texastorque.torquelib.auto.TorqueAutoManager;

public final class AutoManager extends TorqueAutoManager {
    public static volatile AutoManager instance; 

    private AutoManager() { super(); }

    @Override
    public void init() {

    }

    public static synchronized final AutoManager getInstance() {
        return instance == null ? instance = new AutoManager() : instance;
    }
}
