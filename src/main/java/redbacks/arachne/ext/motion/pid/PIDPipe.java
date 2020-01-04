package redbacks.arachne.ext.motion.pid;

import java.util.function.DoubleUnaryOperator;

import edu.wpi.first.wpilibj.PIDOutput;

/**
 * To be integrated into Arachne in 2020.
 * Can be specified as a PIDOutput to perform an operation on the written value before passing it to other PIDOutputs.
 * 
 * @author Mitchell Barker, Darin Huang, Lucas Parker, Gabriela Ribeiro, Jon Reilly, Ben Schwarz, Sean Zammit
 */
public class PIDPipe implements PIDOutput
{
    private final DoubleUnaryOperator operation;
    private final PIDOutput[] outputs;

    public PIDPipe(DoubleUnaryOperator operation, PIDOutput... outputs) {
        this.operation = operation;
        this.outputs = outputs;
    }

    public void pidWrite(double output) {
        for(PIDOutput pidOutput : outputs) pidOutput.pidWrite(operation.applyAsDouble(output));
    }
}
