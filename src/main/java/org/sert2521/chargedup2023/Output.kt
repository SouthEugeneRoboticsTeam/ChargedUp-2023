package org.sert2521.chargedup2023

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard
import edu.wpi.first.wpilibj.DataLogManager
import edu.wpi.first.wpilibj.DriverStation
import edu.wpi.first.wpilibj.livewindow.LiveWindow
import edu.wpi.first.wpilibj.smartdashboard.Field2d
import org.sert2521.chargedup2023.subsystems.Drivetrain
import org.sert2521.chargedup2023.subsystems.Elevator
import java.io.File

object Output {
    private val values = mutableListOf<Pair<String, () -> Double>>()
    private val bools = mutableListOf<Pair<String, () -> Boolean>>()
    private val field = Field2d()

    init {
        LiveWindow.disableAllTelemetry()

        val storageDevices = File("/media").listFiles()!!
        if (storageDevices.isNotEmpty()) {
            DataLogManager.start(storageDevices[0].absolutePath)
            DriverStation.startDataLog(DataLogManager.getLog())
        }

        values.add(Pair("Elevator Extension") { Elevator.extensionMeasure() })
        values.add(Pair("Elevator Angle") { Elevator.angleMeasure() })

        values.add(Pair("Drivetrain Tilt") { Drivetrain.getTilt() })

        bools.add(Pair("Elevator Extension At Top") { Elevator.extensionAtTop() })
        bools.add(Pair("Elevator Extension At Bottom") { Elevator.extensionAtBottom() })

        bools.add(Pair("Elevator Angle At Top") { Elevator.angleAtTop() })
        bools.add(Pair("Elevator Angle At Bottom") { Elevator.angleAtBottom() })

        bools.add(Pair("Elevator Extension Inited") { Elevator.extensionInited })
        bools.add(Pair("Elevator Angle Inited") { Elevator.angleInited })
        bools.add(Pair("Elevator Extension Safe") { Elevator.extensionSafe() })

        SmartDashboard.putData("Output/Field", field)

        update()
    }

    fun update() {
        field.robotPose = Drivetrain.getPose()

        for (value in values) {
            SmartDashboard.putNumber("Output/${value.first}", value.second())
        }

        for (bool in bools) {
            SmartDashboard.putBoolean("Output/${bool.first}", bool.second())
        }
    }
}