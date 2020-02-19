package frc.robot.subsystems;

import java.util.Map;
import java.util.Properties;
import java.util.logging.Logger;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.utils.PropertyFiles;

public abstract class PropertySubsystem extends SubsystemBase {
    protected Logger logger;
    protected final Properties properties;
    private final String className;

    public PropertySubsystem(String className) {
        this.logger = Logger.getLogger(className);
        this.className = className.toLowerCase();
        this.properties = PropertyFiles.loadProperties(this.className);

    }

    protected abstract Map<String, Object> getValues();

    public void saveProperties() {
        Map<String, Object> values = getValues();
        PropertyFiles.saveProperties(this.properties, values, this.className);
    }
}