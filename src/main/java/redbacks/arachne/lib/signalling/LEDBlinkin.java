package redbacks.arachne.lib.signalling;

import edu.wpi.first.wpilibj.Spark;
import redbacks.arachne.lib.actions.Action;
import redbacks.arachne.lib.checks.ChTrue;

/**
 * Class to control LEDs on robot.
 * @author Mitchell Barker, Darin Huang, Lucas Parker, Gabriela Ribeiro, Jon Reilly, Ben Schwarz, Sean Zammit
 */
public class LEDBlinkin
{
    private final Spark controller;
    private Pattern currentPattern = Pattern.BLACK;

    public LEDBlinkin(int channel) {
        this.controller = new Spark(channel);
    }

    public void set(Pattern pattern) {
        this.controller.set(pattern.value);
        this.currentPattern = pattern;
    }

    public Pattern get() {
        return currentPattern;
    }

    public Action acSet(Pattern pattern) {
        return new AcSet(pattern);
    }

    private class AcSet extends Action
    {
        private Pattern pattern;

        private AcSet(Pattern pattern) {
            super(new ChTrue());
            this.pattern = pattern;
        }

        protected void onFinish() {
            set(pattern);
        }
    }

    public static enum Pattern
    {
        YELLOW(0.57),// HOT_PINK(0.57),
        DARK_RED(0.59),
        RED(0.61),
        HOT_PINK(0.63),// RED_ORANGE(0.63),
        PINK(0.65),// ORANGE(0.65),
        PALE_VIOLET(0.67),// GOLD(0.67),
        VIOLET(0.69),// YELLOW(0.69),
        PURPLE(0.71),// LAWN_GREEN(0.71),
        LIGHT_PURPLE(0.73),// LIME(0.73),
        BLUE_PURPLE(0.75),// DARK_GREEN(0.75),
        DARK_BLUE(0.77),// GREEN(0.77),
        BLUE(0.79),// BLUE_GREEN(0.79),
        LIGHT_BLUE(0.81),// AQUA(0.81),
        PALE_BLUE(0.83),// SKY_BLUE(0.83),
        AQUA(0.85),// DARK_BLUE(0.85),
        TURQUOISE(0.87),// BLUE(0.87),
        GREEN(0.89),// BLUE_VIOLET(0.89),
        LIME(0.91),// VIOLET(0.91),
        WHITE(0.93),
        GRAY(0.95),
        DARK_GRAY(0.97),
        BLACK(0.99);

        public final double value;

        private Pattern(double value) {
            this.value = value;
        }
    }
}
