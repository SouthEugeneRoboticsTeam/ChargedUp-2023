package org.sert2521.chargedup2023.commands

import edu.wpi.first.wpilibj2.command.CommandBase
import org.sert2521.chargedup2023.subsystems.Claw

//kai moment
class ClawIntake(private val direction: Boolean) : CommandBase() {


    override fun initialize() {}

    override fun execute() {

        //CLAW IS VERY SLOW PLEASE CHANGE
        if (direction){
            Claw.setMotor(0.01)
        }else{
            Claw.setMotor(-0.01)
        }

    }

    override fun isFinished(): Boolean {

        // TODO: Make this return true when this Command no longer needs to run execute()
        return false
    }

    override fun end(interrupted: Boolean) {
        Claw.stop()
    }
}