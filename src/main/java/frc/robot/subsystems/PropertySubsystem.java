package frc.robot.subsystems;

import java.io.File;
import java.util.Map;
import java.util.Properties;

import edu.wpi.first.wpilibj.Filesystem;
import frc.robot.subsystems.interfaces.IPropertySubsystem;
import frc.robot.utils.PropertyFiles;

public abstract class PropertySubsystem extends LoggingSubsystem implements IPropertySubsystem {
    protected final String m_filename;
    protected final Properties m_properties;

    public PropertySubsystem() {
        m_filename = getClass().getSimpleName().toLowerCase() + ".properties";
        File defaultPropertiesFile = new File(Filesystem.getOperatingDirectory() + "/" + m_filename);
        File propertiesFile = new File(Filesystem.getDeployDirectory() + "/" + m_filename);
        m_properties = PropertyFiles.loadProperties(defaultPropertiesFile, propertiesFile);
    }

    public void saveProperties() {
        Map<String, Object> values = getValues();
        for (String key : values.keySet()) {
            m_properties.setProperty(key, values.get(key).toString());
        }
        
        File propertiesFile = new File(Filesystem.getOperatingDirectory() + "/" + m_filename);
        PropertyFiles.saveProperties(m_properties, propertiesFile);
    }
}