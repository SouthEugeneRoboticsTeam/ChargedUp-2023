package org.sert2521.chargedup2023.commands

import edu.wpi.first.math.filter.LinearFilter
import edu.wpi.first.math.geometry.Pose2d
import edu.wpi.first.math.geometry.Translation2d
import edu.wpi.first.math.kinematics.ChassisSpeeds
import edu.wpi.first.wpilibj2.command.CommandBase
import org.sert2521.chargedup2023.PhysicalConstants
import org.sert2521.chargedup2023.TunedConstants
import org.sert2521.chargedup2023.subsystems.Drivetrain

// This is unused
// Should be run with a timeout and is probably broken
class OverCable(private val driveVector: Translation2d) : CommandBase() {
    private val tiltFilter = LinearFilter.movingAverage(TunedConstants.overCableFilterTaps)
    private var startCablePose = Pose2d()
    private var onCable = false
    private var currentStep = 0

    init {
        addRequirements(Drivetrain)
    }

    override fun initialize() {
        tiltFilter.reset()
        onCable = false
        currentStep = 0
    }

    override fun execute() {
        Drivetrain.drive(ChassisSpeeds.fromFieldRelativeSpeeds(driveVector.x, driveVector.y, 0.0, Drivetrain.getPose().rotation))
    }

    override fun isFinished(): Boolean {
        val tilt = tiltFilter.calculate(Drivetrain.getTilt())
        if (onCable) {
            if (tilt < TunedConstants.cableTipLimit) {
                onCable = false
            }
        } else {
            if (tilt > TunedConstants.cableTipLimit) {
                onCable = true
                currentStep += 1

                if (currentStep == 1) {
                    startCablePose = Drivetrain.getPose()
                }
            }
        }

        if (currentStep == 2 && !onCable) {
            val pose = Pose2d(startCablePose.x + PhysicalConstants.cableDriveDistance, startCablePose.y, Drivetrain.getPose().rotation)
            Drivetrain.setNewPose(pose)
            Drivetrain.setNewVisionPose(pose)
            return true
        }

        return false
    }

    override fun end(interrupted: Boolean) {
        Drivetrain.stop()
    }
}