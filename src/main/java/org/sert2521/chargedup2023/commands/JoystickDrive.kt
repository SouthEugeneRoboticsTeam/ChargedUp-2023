package org.sert2521.chargedup2023.commands

import edu.wpi.first.math.kinematics.ChassisSpeeds
import edu.wpi.first.wpilibj.Timer
import edu.wpi.first.wpilibj2.command.CommandBase
import org.sert2521.chargedup2023.ConfigConstants
import org.sert2521.chargedup2023.subsystems.Drivetrain
import org.sert2521.chargedup2023.Input
import kotlin.math.abs
import kotlin.math.pow
import kotlin.math.sqrt

class JoystickDrive(private val fieldOrientated: Boolean) : CommandBase() {
    private var x = 0.0
    private var y = 0.0
    private var prevTime = 0.0

    init {
        addRequirements(Drivetrain)
    }

    override fun initialize() {
        x = 0.0
        y = 0.0

        prevTime = Timer.getFPGATimestamp()
    }

    override fun execute() {
        val currentTime = Timer.getFPGATimestamp()
        val diffTime = (currentTime - prevTime)
        prevTime = currentTime

        val slow = Input.getSlow()
        var currX = Input.getX()
        var currY = Input.getY()

        val sqrMagnitude = currX.pow(2) + currY.pow(2)
        if (sqrMagnitude <= ConfigConstants.joystickDeadband.pow(2)) {
            currX = 0.0
            currY = 0.0
        } else if (sqrMagnitude > 1) {
            val magnitude = sqrt(sqrMagnitude)
            currX /= magnitude
            currY /= magnitude
        }

        val trueSpeed = ConfigConstants.driveSpeed - (ConfigConstants.slowDriveSpeed * slow)
        currX *= trueSpeed
        currY *= trueSpeed

        val diffX = x - currX
        val diffY = y - currY
        val rateChangeSqr = diffX.pow(2) + diffY.pow(2)
        if (rateChangeSqr <= ConfigConstants.joystickChangeSpeed.pow(2)) {
            x = currX
            y = currY
        } else {
            val rateChange = sqrt(rateChangeSqr) / diffTime
            x -= diffX / diffTime * ConfigConstants.joystickChangeSpeed / rateChange
            y -= diffY / diffTime * ConfigConstants.joystickChangeSpeed / rateChange
        }

        var rot = Input.getRot()
        if (abs(rot) <= ConfigConstants.joystickDeadband) {
            rot = 0.0
        }

        rot *= ConfigConstants.rotSpeed - (ConfigConstants.slowRotSpeed * slow)

        if (x.pow(2) + y.pow(2) <= ConfigConstants.powerDeadband.pow(2) && abs(rot) <= ConfigConstants.rotDeadband) {
            if (Input.getBrakePos()) {
                Drivetrain.enterBrakePos()
            } else {
                Drivetrain.stop()
            }
        } else {
            if (fieldOrientated) {
                // Maybe use rotation not based on vision
                Drivetrain.drive(ChassisSpeeds.fromFieldRelativeSpeeds(x, y, rot, Drivetrain.getPose().rotation))
            } else {
                Drivetrain.drive(ChassisSpeeds(x, y, rot))
            }
        }
    }

    override fun end(interrupted: Boolean) {
        Drivetrain.stop()
    }
}