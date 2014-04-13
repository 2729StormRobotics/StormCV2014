/*
 * To change this template, choose Tools | Templates
 * and open the template in the editor.
 */
package Storm2014CV;

import edu.wpi.first.smartdashboard.camera.WPICameraExtension;
import edu.wpi.first.smartdashboard.camera.WPILaptopCameraExtension;
import edu.wpi.first.smartdashboard.properties.ColorProperty;
import edu.wpi.first.smartdashboard.properties.DoubleProperty;
import edu.wpi.first.smartdashboard.properties.IntegerProperty;
import edu.wpi.first.wpijavacv.WPIColor;
import edu.wpi.first.wpijavacv.WPIColorImage;
import edu.wpi.first.wpijavacv.WPIImage;
import java.awt.Color;

/**
 *
 * @author Tim
 */
public class SomethingCameras extends WPILaptopCameraExtension {
    public IntegerProperty xProp = new IntegerProperty(this, "X", 7);
    public IntegerProperty yProp = new IntegerProperty(this, "Y", 15);
    public IntegerProperty widthProp = new IntegerProperty(this, "Width", 85);
    public IntegerProperty heightProp = new IntegerProperty(this, "Height", 16);
    public IntegerProperty thicknessProp = new IntegerProperty(this, "Thickness", 8);
    public ColorProperty colorProp = new ColorProperty(this, "Color", Color.CYAN);
    

    @Override
    public WPIImage processImage(WPIColorImage rawImage) {
        WPIColor wpiColorProp = new WPIColor(colorProp.getValue());
        rawImage.drawRect(xProp.getValue(), yProp.getValue(), widthProp.getValue(), heightProp.getValue(), wpiColorProp, thicknessProp.getValue());
        return super.processImage(rawImage); //To change body of generated methods, choose Tools | Templates.
    }
    
}
