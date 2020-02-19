package frc.robot.subsystems;

import java.util.Map;
import java.util.Properties;
import java.util.logging.Logger;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.utils.PropertyFiles;

public abstract class PropertySubsystem extends SubsystemBase {
    protected final Logger logger;
    protected final Properties properties;
    private final String className;

    public PropertySubsystem(String fullName) {
        this.logger = Logger.getLogger(fullName);
        String[] classNameArray = fullName.split("\\.");
        className = classNameArray[classNameArray.length - 1].toLowerCase();
        this.properties = PropertyFiles.loadProperties(this.className);

    }

    protected abstract Map<String, Object> getValues();

    public void saveProperties() {
        Map<String, Object> values = getValues();
        PropertyFiles.saveProperties(this.properties, values, this.className);
    }
}