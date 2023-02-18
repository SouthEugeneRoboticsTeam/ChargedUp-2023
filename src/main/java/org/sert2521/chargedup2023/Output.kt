package org.sert2521.chargedup2023

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard
import edu.wpi.first.wpilibj.DataLogManager
import edu.wpi.first.wpilibj.DriverStation
import edu.wpi.first.wpilibj.livewindow.LiveWindow
import org.sert2521.chargedup2023.subsystems.Elevator
import java.io.File

object Output {
    private val values = mutableListOf<Pair<String, () -> Double>>()
    private val bools = mutableListOf<Pair<String, () -> Boolean>>()

    init {
        LiveWindow.disableAllTelemetry()

        val storageDevices = File("/media").listFiles()!!
        if (storageDevices.isNotEmpty()) {
            DataLogManager.start(storageDevices[0].absolutePath)
            DriverStation.startDataLog(DataLogManager.getLog())
        }

        values.add(Pair("Elevator Extension") { Elevator.extensionMeasure() })
        values.add(Pair("Elevator Angle") { Elevator.angleMeasure() })

        bools.add(Pair("Elevator Extension At Top") { Elevator.extensionAtTop() })
        bools.add(Pair("Elevator Extension At Bottom") { Elevator.extensionAtBottom() })

        bools.add(Pair("Elevator Angle At Top") { Elevator.angleAtTop() })
        bools.add(Pair("Elevator Angle At Bottom") { Elevator.angleAtBottom() })

        bools.add(Pair("Elevator Extension Inited") { Elevator.extensionInited })
        bools.add(Pair("Elevator Extension Safe") { Elevator.extensionSafe() })

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