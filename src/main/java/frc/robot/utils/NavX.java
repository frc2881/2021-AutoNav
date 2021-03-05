package frc.robot.utils;

import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.smartdashboard.SendableBuilder;

/**
 * Extends the NavX Robotics Navigation Sensor to send more telemetry to Smart Dashboard / Shuffleboard.
 */
public class NavX extends AHRS {
    public NavX(SPI.Port spi_port_id) {
        super(spi_port_id);
    }

    /*@Override
    public void initSendable(SendableBuilder builder) {
        super.initSendable(builder);
        builder.addDoubleProperty("Yaw", this::getYaw, null);
        builder.addDoubleProperty("Pitch", this::getPitch, null);
        builder.addDoubleProperty("Roll", this::getRoll, null);
        builder.addDoubleProperty("Velocity", this::getVelocity, null);
        builder.addDoubleProperty("VelocityX", this::getVelocityX, null);
        builder.addDoubleProperty("VelocityY", this::getVelocityY, null);
    }

    private double getVelocity() {
        float x = getVelocityX();
        float y = getVelocityY();
        double v = Math.sqrt(x * x + y * y);
        return v / 3.28084;  // meters to feet
    }*/
}