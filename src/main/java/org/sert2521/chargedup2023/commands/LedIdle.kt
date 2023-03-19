package org.sert2521.chargedup2023.commands


import edu.wpi.first.wpilibj2.command.CommandBase
import org.sert2521.chargedup2023.ConfigConstants
import org.sert2521.chargedup2023.PhysicalConstants
import org.sert2521.chargedup2023.TunedConstants
import org.sert2521.chargedup2023.TunedConstants.ledsRainbowA
import org.sert2521.chargedup2023.TunedConstants.ledsRainbowB
import org.sert2521.chargedup2023.TunedConstants.ledsRainbowC
import org.sert2521.chargedup2023.TunedConstants.ledsRainbowD
import org.sert2521.chargedup2023.TunedConstants.ledsRainbowE
import org.sert2521.chargedup2023.subsystems.Drivetrain
import org.sert2521.chargedup2023.subsystems.LEDs
import kotlin.math.min
import kotlin.math.pow

class LedIdle : CommandBase() {

    private var lastPose = Drivetrain.pose

    private var driveSpeed = 0.0
    private var hueTimer = 0.0

    init {
        // each subsystem used by the command must be passed into the addRequirements() method
        addRequirements(LEDs)
    }

    override fun initialize() {
        LEDs.setAllLEDHSV(0, 0, 0)
    }


    override fun execute() {

        driveSpeed =50 * kotlin.math.sqrt((lastPose.x - Drivetrain.pose.x).pow(2) + (lastPose.y - Drivetrain.pose.y).pow(2))
        lastPose = Drivetrain.pose

        hueTimer += (-ledsRainbowA.pow(ledsRainbowB/(driveSpeed/ConfigConstants.driveSpeed + ledsRainbowC))+ledsRainbowD)*ledsRainbowE



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
