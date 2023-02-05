package frc.robot.subsystems;

import com.techhounds.houndutil.houndlog.LogGroup;
import com.techhounds.houndutil.houndlog.LoggingManager;
import com.techhounds.houndutil.houndlog.enums.LogLevel;
import com.techhounds.houndutil.houndlog.logitems.StringLogItem;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

/**
 * The LEDs subsystem, which controls the state of the LEDs through an internal
 * state machine and continuously updates the LED's buffer.
 * 
 * @author dr
 */
public class LEDs extends SubsystemBase {
    /** The LEDs. */
    private AddressableLED led = new AddressableLED(0);

    /** The last state of the LEDs before it was in {@code state}. */
    private LEDState previousState = LEDState.Disabled;
    /** The current state of the LEDs. */
    private LEDState state = LEDState.Disabled;

    /**
     * Describes the states that the LEDs can be in.
     */
    public enum LEDState {
        /** Rainbow pattern that moves across the strip. */
        Disabled(LEDState::rainbow),
        /** Illuminated yellow to signify the cone pickup mode. */
        ConePickup(LEDState::conePickup),
        /** Illuminated purple to signify the cube pickup mode. */
        CubePickup(LEDState::cubePickup),
        /** Blinking red to signify an error state. */
        Error(LEDState::error),
        /** Blinking red to signify an error state. */
        Uninitialized(LEDState::uninitialized);

        /**
         * The hue of the first pixel in the strand, used to "move" the rainbow down the
         * strip.
         */
        private static int rainbowFirstPixelHue = 0;
        /** The common buffer object used between each buffer supplier. */
        private static AddressableLEDBuffer buffer = new AddressableLEDBuffer(Constants.LEDs.LENGTH);
        /** The current time in 20ms ticks, used for the error state. */
        private static int timeStep = 0;

        /** The supplier for the buffer for each LEDState. */
        private final Runnable bufferRunnable;

        /** The ctor for the LEDState, takes in a Supplier<AddressableLEDBuffer>. */
        private LEDState(Runnable bufferRunnable) {
            this.bufferRunnable = bufferRunnable;
        }

        /**
         * Get the buffer for this LEDState.
         * 
         * @return the buffer
         */
        public AddressableLEDBuffer getBuffer() {
            this.bufferRunnable.run();
            return buffer;
        }

        /**
         * Changes the contents of the AddressableLEDBuffer to the rainbow state.
         */
        private static void rainbow() {
            for (int i = 0; i < buffer.getLength(); i++) {
                // Calculate the hue - hue is easier for rainbows because the color
                // shape is a circle so only one value needs to precess
                final var hue = (rainbowFirstPixelHue + (i * 180 / buffer.getLength())) % 180;
                // Set the value
                buffer.setHSV(i, hue, 255, 128);

            }
            // Increase by 3 to make the rainbow "move"
            rainbowFirstPixelHue += 3;
            // Check bounds
            rainbowFirstPixelHue %= 180;
        }

        /**
         * Changes the contents of the AddressableLEDBuffer to the cone pickup state.
         */
        private static void conePickup() {
            for (int i = 0; i < buffer.getLength(); i++) {
                buffer.setHSV(i, 30, 255, 128);
            }
        }

        /**
         * Changes the contents of the AddressableLEDBuffer to the cube pickup state.
         */
        private static void cubePickup() {
            for (int i = 0; i < buffer.getLength(); i++) {
                buffer.setHSV(i, 150, 255, 128);
            }
        }

        /**
         * Changes the contents of the AddressableLEDBuffer to the error state.
         */
        private static void error() {
            timeStep++; // 50 timesteps is one second
            timeStep %= 50; // every 1 second it will roll over
            for (int i = 0; i < buffer.getLength(); i++) {
                // every 0.5 seconds it will switch from off to on
                if (timeStep / 25 == 1) {
                    buffer.setHSV(i, 0, 255, 128);
                } else {
                    buffer.setHSV(i, 0, 0, 0);
                }
            }
        }

        /**
         * Changes the contents of the AddressableLEDBuffer to the uninitialized state.
         */
        private static void uninitialized() {
            timeStep++; // 50 timesteps is one second
            timeStep %= 50; // every 1 second it will roll over
            for (int i = 0; i < buffer.getLength(); i++) {
                // every 0.2 seconds it will switch from off to on
                if (timeStep / 10 == 1) {
                    buffer.setHSV(i, 0, 255, 128);
                } else {
                    buffer.setHSV(i, 0, 0, 0);
                }
            }
        }
    }

    /**
     * Initializes the LEDs.
     */
    public LEDs() {
        led.setLength(50);
        led.setData(state.getBuffer());
        led.start();

        LoggingManager.getInstance().addGroup("LEDs",
                new LogGroup(
                        new StringLogItem("State", () -> this.state.toString(), LogLevel.MAIN)));
    }

    /**
     * Gets the current state of the LEDs.
     * 
     * @return the current LEDState.
     */
    public LEDState getLEDState() {
        return state;
    }

    /**
     * Sets the state of the LEDs, and sets the previousState to the old current
     * state.
     * 
     * @param state the new LEDState.
     */
    private void setLEDState(LEDState state) {
        this.previousState = this.state;
        this.state = state;
    }

    /**
     * Creates a command that changes the state of the LEDs.
     * 
     * @param state the new LEDState.
     * @return the command
     */
    public Command setLEDStateCommand(LEDState state) {
        return runOnce(() -> this.setLEDState(state));
    }

    /**
     * Creates a command that sets the LEDs to their previous setting.
     * 
     * @return
     */
    public Command setPreviousLEDStateCommand() {
        return runOnce(() -> this.setLEDState(previousState));
    }

    /**
     * Creates a command that will run the LEDs in an "error" mode for 2 seconds,
     * then switch back to their original setting (useful for indicating an error
     * after a button press, like if a certain value isn't set properly or a safety
     * has been triggered).
     * 
     * @return the command
     */
    public CommandBase errorCommand() {
        return Commands.sequence(
                setLEDStateCommand(LEDState.Error),
                Commands.waitSeconds(2),
                this.setPreviousLEDStateCommand());
    }

    /**
     * Sets the LEDs to the current state of the buffer.
     */
    @Override
    public void periodic() {
        super.periodic();
        led.setData(state.getBuffer());
    }
}
