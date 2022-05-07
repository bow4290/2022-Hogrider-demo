package frc.robot.sensors;

import edu.wpi.first.wpilibj.DigitalInput;

public class ButtonWrapper {

    private int buttonPort;
    private enum ButtonType{ NO, NC };
    private ButtonType buttonType;

    private final DigitalInput button = new DigitalInput(buttonPort);
    
    public ButtonWrapper(ButtonType buttonType, int buttonPort) {
        this.buttonType = buttonType;
        this.buttonPort = buttonPort;
    }

    public boolean isPressed() { return (buttonType == ButtonType.NO) ? button.get() : !button.get(); }
}
