package org.sert2521.chargedup2023.commands

import edu.wpi.first.math.controller.PIDController
import edu.wpi.first.math.kinematics.ChassisSpeeds
import edu.wpi.first.wpilibj.DriverStation
import org.sert2521.chargedup2023.Input
import org.sert2521.chargedup2023.Output
import org.sert2521.chargedup2023.PhysicalConstants
import org.sert2521.chargedup2023.TunedConstants
import org.sert2521.chargedup2023.subsystems.Drivetrain
import java.lang.Math.PI
import kotlin.math.abs

class VisionAlignCone : JoystickCommand() {
    private val positionPID = PIDController(TunedConstants.swerveAlignDistanceP, TunedConstants.swerveAlignDistanceI, TunedConstants.swerveAlignDistanceD)
    private val anglePID = PIDController(TunedConstants.swerveConeAlignAngleP, TunedConstants.swerveConeAlignAngleI, TunedConstants.swerveConeAlignAngleD)

    init {
        addRequirements(Drivetrain)

        anglePID.enableContinuousInput(0.0, 2 * PI)

        positionPID.setTolerance(TunedConstants.visionConePositionTolerance)
        anglePID.setTolerance(TunedConstants.visionConeAngleTolerance)
    }

    override fun initialize() {
        super.initialize()

        Drivetrain.setVisionStandardDeviations()
        positionPID.reset()
        LedSolid(10, 255, 255).schedule()
    }

    override fun execute() {
        val pose = Drivetrain.getVisionPose()

        val color = Input.getColor()
        val conePoints = when (color) {
            DriverStation.Alliance.Blue -> PhysicalConstants.conePointsBlue
            DriverStation.Alliance.Red -> PhysicalConstants.conePointsRed
            DriverStation.Alliance.Invalid -> null
        }
        val sliderDirection = when (color) {
            DriverStation.Alliance.Blue -> 1
            DriverStation.Alliance.Red -> -1
            DriverStation.Alliance.Invalid -> 0
        }

        // Move 0.12 to constants (or get rid of it)
        val yTarget = conePoints?.minBy { abs(it - pose.y) }//?.plus(0.12 * Input.getSlider() * sliderDirection)

        if (Drivetrain.visionSeeingThings() && yTarget != null) {
            // Moving the y on the stick will affect the rate of change of the x
            if (!positionPID.atSetpoint() || !anglePID.atSetpoint()) {
                Drivetrain.drive(ChassisSpeeds.fromFieldRelativeSpeeds(readJoystick().x, positionPID.calculate(pose.y, yTarget), anglePID.calculate(pose.rotation.radians, PhysicalConstants.coneAngle), pose.rotation))
                Output.visionHappy = false
            } else {
                // To update stuff
                positionPID.calculate(pose.y, yTarget)
                anglePID.calculate(pose.rotation.radians, PhysicalConstants.coneAngle)

                Drivetrain.drive(ChassisSpeeds.fromFieldRelativeSpeeds(readJoystick().x, 0.0, 0.0, pose.rotation))
                Output.visionHappy = true
            }
        } else {
            positionPID.reset()
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