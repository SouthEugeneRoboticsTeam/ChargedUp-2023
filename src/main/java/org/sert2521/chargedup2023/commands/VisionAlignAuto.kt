package org.sert2521.chargedup2023.commands

import edu.wpi.first.math.controller.PIDController
import edu.wpi.first.math.kinematics.ChassisSpeeds
import edu.wpi.first.wpilibj.DriverStation
import edu.wpi.first.wpilibj2.command.CommandBase
import org.sert2521.chargedup2023.Input
import org.sert2521.chargedup2023.Output
import org.sert2521.chargedup2023.PhysicalConstants
import org.sert2521.chargedup2023.TunedConstants
import org.sert2521.chargedup2023.subsystems.Drivetrain
import java.lang.Math.PI

class VisionAlignAuto(private val inYTarget: Double, private val xTarget: Double) : CommandBase() {
    private val positionPIDY = PIDController(TunedConstants.swerveAlignDistanceP, TunedConstants.swerveAlignDistanceI, TunedConstants.swerveAlignDistanceD)
    private val positionPIDX = PIDController(TunedConstants.swerveAlignDistanceP, TunedConstants.swerveAlignDistanceI, TunedConstants.swerveAlignDistanceD)
    private val anglePID = PIDController(TunedConstants.swerveConeAlignAngleP, TunedConstants.swerveConeAlignAngleI, TunedConstants.swerveConeAlignAngleD)

    init {
        addRequirements(Drivetrain)

        anglePID.enableContinuousInput(0.0, 2 * PI)

        positionPIDY.setTolerance(TunedConstants.visionConePositionTolerance)
        positionPIDX.setTolerance(TunedConstants.visionConePositionTolerance)
        anglePID.setTolerance(TunedConstants.visionConeAngleTolerance)
    }

    override fun initialize() {
        super.initialize()

        Drivetrain.setVisionStandardDeviations()
        positionPIDY.reset()
        positionPIDX.reset()
        LedSolid(10, 255, 255).schedule()
    }

    override fun execute() {
        val pose = Drivetrain.getVisionPose()

        val yTarget = when (Input.getColor()) {
            DriverStation.Alliance.Blue -> inYTarget
            DriverStation.Alliance.Red -> PhysicalConstants.fieldWidth - inYTarget
            DriverStation.Alliance.Invalid -> null
        }

        if (Drivetrain.visionSeeingThings() && yTarget != null) {
            if (!positionPIDX.atSetpoint() || !positionPIDY.atSetpoint() || !anglePID.atSetpoint()) {
                Drivetrain.drive(ChassisSpeeds.fromFieldRelativeSpeeds(positionPIDX.calculate(pose.x, xTarget), positionPIDY.calculate(pose.y, yTarget), anglePID.calculate(pose.rotation.radians, PhysicalConstants.coneAngle), pose.rotation))
                Output.visionHappy = false
            } else {
                // To update stuff
                positionPIDY.calculate(pose.y, yTarget)
                positionPIDX.calculate(pose.x, xTarget)
                anglePID.calculate(pose.rotation.radians, PhysicalConstants.coneAngle)

                Drivetrain.stop()
                Output.visionHappy = true
            }
        } else {
            positionPIDY.reset()
            positionPIDX.reset()
            anglePID.reset()

            Drivetrain.stop()
            Output.visionHappy = false
        }

        if (Output.visionHappy) {
            LedSolid(60, 255, 255).schedule()
        }else{
            LedSolid(10, 255,255).schedule()
        }
    }

    override fun end(interrupted: Boolean) {
        Drivetrain.setVisionAlignDeviations()
        Output.visionHappy = false
        Drivetrain.stop()
        LedIdle().schedule()
    }
}