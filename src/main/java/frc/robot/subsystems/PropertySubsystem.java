package frc.robot.subsystems;

import java.io.File;
import java.util.Map;
import java.util.Properties;
import java.util.logging.Logger;

import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.interfaces.IPropertySubsystem;
import frc.robot.utils.PropertyFiles;

public abstract class PropertySubsystem extends SubsystemBase implements IPropertySubsystem {
    protected final Logger m_logger;
    protected final String m_filename;
    protected final Properties m_properties;

    public PropertySubsystem() {
        m_logger = Logger.getLogger(getClass().getName());
        m_filename = getClass().getName();
        File defaultPropertiesFile = new File(Filesystem.getOperatingDirectory() + "/" + m_filename);
        File propertiesFile = new File(Filesystem.getDeployDirectory() + "/" + m_filename);
        m_properties = PropertyFiles.loadProperties(defaultPropertiesFile, propertiesFile);
    }

    protected abstract Map<String, Object> getValues();

    public void saveProperties() {
        Map<String, Object> values = getValues();
        for (String key : values.keySet()) {
            m_properties.setProperty(key, values.get(key).toString());
        }
        
        File propertiesFile = new File(Filesystem.getOperatingDirectory() + "/" + m_filename);
        PropertyFiles.saveProperties(m_properties, propertiesFile);
    }
}