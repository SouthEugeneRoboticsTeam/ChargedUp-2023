package org.sert2521.chargedup2023.commands


import edu.wpi.first.math.geometry.Pose2d
import edu.wpi.first.wpilibj2.command.CommandBase
import org.sert2521.chargedup2023.ConfigConstants
import org.sert2521.chargedup2023.PhysicalConstants
import org.sert2521.chargedup2023.subsystems.Drivetrain
import org.sert2521.chargedup2023.subsystems.LEDs
import kotlin.math.min
import kotlin.math.pow

class LedIdle : CommandBase() {

    private var lastPose = Pose2d()

    private var driveSpeed = 0.0
    private var hueTimer = 0.0

    init {
        // each subsystem used by the command must be passed into the addRequirements() method
        addRequirements(LEDs)
    }

    override fun initialize() {
        LEDs.setAllLEDHSV(0, 0, 0)

        lastPose = Drivetrain.getPose()
    }


    override fun execute() {
        val currPose = Drivetrain.getPose()
        driveSpeed = 20*kotlin.math.sqrt((lastPose.x - currPose.x).pow(2) + (lastPose.y - currPose.y).pow(2))
        lastPose = Drivetrain.getPose()

        hueTimer += 10.0*driveSpeed+0.75


        for (i in 0 until PhysicalConstants.ledLength){
            LEDs.setLEDHSV(i, hueTimer.toInt()+i*4, 255,
                min((255.0*driveSpeed/ConfigConstants.driveSpeed).toInt(), 255).coerceAtLeast(15)
            )
        }
    }

    override fun isFinished(): Boolean {
        // TODO: Make this return true when this Command no longer needs to run execute()
        return false
    }

    override fun end(interrupted: Boolean) {
        LEDs.reset()
    }
}
