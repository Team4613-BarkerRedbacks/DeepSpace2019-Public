package redbacks.arachne.lib.solenoids;

/**
 * Temporary replacement for SolSingle that allows the instructions to be inverted for clarity.
 * Will likely be abstracted and integrated into Arachne for 2020 season.
 * 
 * @author Mitchell Barker, Darin Huang, Lucas Parker, Gabriela Ribeiro, Jon Reilly, Ben Schwarz, Sean Zammit
 */
public class SolSingle2 extends SolSingle
{
    protected boolean isInverted = false;

    public SolSingle2(int port) {
        super(port);
    }

    public void set(boolean on) {
        super.set(on != isInverted);
    }

    public void setInverted(boolean isInverted) {
        this.isInverted = isInverted;
    }

    public boolean getInverted() {
        return isInverted;
    }
}