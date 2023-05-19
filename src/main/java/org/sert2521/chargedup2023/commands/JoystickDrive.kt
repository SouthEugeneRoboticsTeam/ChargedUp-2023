package org.sert2521.chargedup2023.commands

import edu.wpi.first.math.kinematics.ChassisSpeeds
import org.sert2521.chargedup2023.ConfigConstants
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
        } else if (!(Elevator.extensionMeasure()>PhysicalConstants.elevatorDemoTriggerExtension
                        || Elevator.angleMeasure()>PhysicalConstants.elevatorDemoTriggerAngle)){
            if (fieldOrientated) {
                Drivetrain.drive(ChassisSpeeds.fromFieldRelativeSpeeds(joystickData.x*ConfigConstants.demoModeDriveMultiplier, joystickData.y*ConfigConstants.demoModeDriveMultiplier, joystickData.z*ConfigConstants.demoModeRotateMultiplier, Drivetrain.getPose().rotation))
            } else {
                Drivetrain.drive(ChassisSpeeds(joystickData.x*ConfigConstants.demoModeDriveMultiplier, joystickData.y*ConfigConstants.demoModeDriveMultiplier, joystickData.z*ConfigConstants.demoModeRotateMultiplier))
            }
        } else {
            if (fieldOrientated) {
                Drivetrain.drive(ChassisSpeeds.fromFieldRelativeSpeeds(joystickData.x*ConfigConstants.demoModeDriveArmMultiplier, joystickData.y*ConfigConstants.demoModeDriveArmMultiplier, joystickData.z*ConfigConstants.demoModeRotateArmMultiplier, Drivetrain.getPose().rotation))
            } else {
                Drivetrain.drive(ChassisSpeeds(joystickData.x*ConfigConstants.demoModeDriveArmMultiplier, joystickData.y*ConfigConstants.demoModeDriveArmMultiplier, joystickData.z*ConfigConstants.demoModeRotateArmMultiplier))
            }
        }
    }

    override fun end(interrupted: Boolean) {
        Drivetrain.stop()
    }
}