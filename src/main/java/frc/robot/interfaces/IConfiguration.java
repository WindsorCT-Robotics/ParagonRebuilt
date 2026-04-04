package frc.robot.interfaces;

public interface IConfiguration<Config> {
    public Config getDefaultConfiguration();
    public Config getCurrentConfiguration();
    public void configure(Config configuration);
}
