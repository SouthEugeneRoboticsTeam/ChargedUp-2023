package org.sert2521.chargedup2023

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard
import edu.wpi.first.wpilibj.DataLogManager
import edu.wpi.first.wpilibj.DriverStation
import java.io.File

object Output {
    private val values = mutableListOf<Pair<String, () -> Double>>()
    private val bools = mutableListOf<Pair<String, () -> Boolean>>()

    init {
        val storageDevices = File("/media").listFiles()!!
        if (storageDevices.isNotEmpty()) {
            DataLogManager.start(storageDevices[0].absolutePath)
            DriverStation.startDataLog(DataLogManager.getLog())
        }

        update()
    }

    fun update() {
        for (value in values) {
            SmartDashboard.putNumber("Output/${value.first}", value.second())
        }

        for (bool in bools) {
            SmartDashboard.putBoolean("Output/${bool.first}", bool.second())
        }
    }
}