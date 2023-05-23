package org.sert2521.chargedup2023.commands

import edu.wpi.first.math.kinematics.ChassisSpeeds
import org.sert2521.chargedup2023.ConfigConstants
import org.sert2521.chargedup2023.DemoConstants
import org.sert2521.chargedup2023.subsystems.Drivetrain
import org.sert2521.chargedup2023.Input
import org.sert2521.chargedup2023.PhysicalConstants
import org.sert2521.chargedup2023.subsystems.Elevator

class JoystickDrive(private val fieldOrientated: Boolean) : JoystickCommand() {
    init {
        addRequirements(Drivetrain)
    }

    override fun execute() {
        val joystickData = readJoystick()
        if (joystickData.x == 0.0 && joystickData.y == 0.0 && joystickData.z == 0.0) {
            if (Input.getBrakePos()) {
                Drivetrain.enterBrakePos()
            } else {
                Drivetrain.stop()
            }
        } else if (!(Elevator.extensionMeasure()>DemoConstants.elevatorDemoTriggerExtension
                        || Elevator.angleMeasure()>DemoConstants.elevatorDemoTriggerAngle)){
            if (fieldOrientated) {
                Drivetrain.drive(ChassisSpeeds.fromFieldRelativeSpeeds(joystickData.x, joystickData.y, joystickData.z, Drivetrain.getPose().rotation))
            } else {
                Drivetrain.drive(ChassisSpeeds(joystickData.x, joystickData.y, joystickData.z))
            }
        } else {
            if (fieldOrientated) {
                Drivetrain.drive(ChassisSpeeds.fromFieldRelativeSpeeds(joystickData.x, joystickData.y, joystickData.z, Drivetrain.getPose().rotation))
            } else {
                Drivetrain.drive(ChassisSpeeds(joystickData.x, joystickData.y, joystickData.z))
            }
        }
    }

    override fun end(interrupted: Boolean) {
        Drivetrain.stop()
    }
}