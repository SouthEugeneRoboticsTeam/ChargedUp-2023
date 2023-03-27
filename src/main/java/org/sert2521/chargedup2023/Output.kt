package org.sert2521.chargedup2023

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard
import edu.wpi.first.wpilibj.DataLogManager
import edu.wpi.first.wpilibj.DriverStation
import edu.wpi.first.wpilibj.smartdashboard.Field2d
import edu.wpi.first.wpilibj2.command.button.Trigger
import org.sert2521.chargedup2023.commands.RumbleBlip
import org.sert2521.chargedup2023.subsystems.Claw
import org.sert2521.chargedup2023.subsystems.Drivetrain
import org.sert2521.chargedup2023.subsystems.Elevator
import java.io.File

object Output {
    private val values = mutableListOf<Pair<String, () -> Double>>()
    private val bools = mutableListOf<Pair<String, () -> Boolean>>()
    private val field = Field2d()
    private val visionField = Field2d()

    var visionHappy = false

    init {
        val storageDevices = File("/media").listFiles()
        if (storageDevices != null) {
            if (storageDevices.isNotEmpty()) {
                DataLogManager.start(storageDevices[0].absolutePath)
                DriverStation.startDataLog(DataLogManager.getLog())
            }
        }

        values.add(Pair("Elevator Extension") { Elevator.extensionMeasure() })
        values.add(Pair("Elevator Angle") { Elevator.angleMeasure() })
        // Maybe make this more encapsulationy
        values.add(Pair("Elevator Angle Power") { Elevator.angleMotor.appliedOutput })
        values.add(Pair("Elevator Angle Susness") { Elevator.angleSusness() })

        values.add(Pair("Drivetrain Tilt") { Drivetrain.getTilt() })
        values.add(Pair("Drivetrain Delta Pose X") { Drivetrain.deltaPose.x })
        values.add(Pair("Drivetrain Delta Pose Y") { Drivetrain.deltaPose.y })
        values.add(Pair("Drivetrain Delta Pose Rot") { Drivetrain.deltaPose.rotation.radians })

        bools.add(Pair("Elevator Extension At Top") { Elevator.extensionAtTop() })
        bools.add(Pair("Elevator Extension At Bottom") { Elevator.extensionAtBottom() })
        bools.add(Pair("Elevator Browned Out") { Elevator.brownedOut })

        bools.add(Pair("Elevator Angle At Top") { Elevator.angleAtTop() })
        bools.add(Pair("Elevator Angle At Bottom") { Elevator.angleAtBottom() })

        bools.add(Pair("Elevator Extension Inited") { Elevator.extensionInited })
        bools.add(Pair("Elevator Angle Inited") { Elevator.angleInited })
        bools.add(Pair("Elevator Extension Safe") { Elevator.extensionSafe() })

        bools.add(Pair("Slow Mode") { Input.slowMode })

        bools.add(Pair("Vision Happy") { visionHappy })

        bools.add(Pair("Claw Full") { Claw.clawFull })

        SmartDashboard.putData("Output/Field", field)
        SmartDashboard.putData("Output/VisionField", visionField)

        update()

        // Constants?
        Trigger { Claw.clawFull }.onTrue(RumbleBlip(0.7, 0.4))
    }

    fun update() {
        field.robotPose = Drivetrain.getPose()
        visionField.robotPose = Drivetrain.getVisionPose()

        for (value in values) {
            SmartDashboard.putNumber("Output/${value.first}", value.second())
        }

        for (bool in bools) {
            SmartDashboard.putBoolean("Output/${bool.first}", bool.second())
        }
    }
}