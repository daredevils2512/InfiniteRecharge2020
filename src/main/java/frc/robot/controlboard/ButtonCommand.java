package frc.robot.controlboard;

public enum ButtonCommand {
  INVERT_DRIVING,
  SHIFT_DRIVETRAIN,
  INTAKE_EXTENDER_MOTION_MAGIC,
  EXTEND_INTAKE,
  RETRACT_INTAKE,
  MANUAL_RUN_INTAKE,
  MANUAL_RUN_INTAKE_REVERSE,
  INTAKE,
  MANUAL_RUN_MAGAZINE,
  MANUAL_RUN_MAGAZINE_REVERSE,
  AUTO_REFILL_QUEUE,
  MANUAL_RUN_QUEUE,
  MANUAL_RUN_QUEUE_REVERSE,
  AUTO_FEED_SHOOTER,
  MOVE_POWER_CELLS, // Run everything (intake, magazine, queue)
  MOVE_POWER_CELLS_REVERSE, // Run everything in reverse (intake, magazine, queue)
  MANUAL_RUN_SHOOTER, // Spin flywheel manaully
  AUTO_RUN_SHOOTER, // Set flywheel to automatically determine a target velocity
  AUTO_SHOOT,
  TURRET_TESTING_MOTION_MAGIC,
  AUTO_AIM_TURRET,
  HOOD_UP,
  HOOD_DOWN, 
  TOGGLE_COMPRESSOR,
  STOP_MOTORS,
  AUTONOMOUS, SHOOT_BALL;
}
