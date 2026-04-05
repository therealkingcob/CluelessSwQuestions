Q1. 
//Counts the longest streak in the array that matches target

  int countStreak(int[] arr, int target) {
    int count = 0, maxCount = 0;

    for (int x : arr) {
        if (x == target) count++;
        else count = 0;
        maxCount = count;
    }
    return maxCount;
}

Q2. Whats the difference between Math.atan2 and Math.atan in java?

Q3. 
//count the longest strictly increasing streak in the array
int longestIncreasingStreak(int[] arr) {
    if (arr.length == 0) return 0;

    int count = 1;
    int maxCount = 1;

    for (int i = 1; i < arr.length; i++) {
        if (arr[i] > arr[i - 1]) {
            count++;
        } else {
            count = 0;
        }

        if (count > maxCount) {
            maxCount = count;
        }
    }

    return maxCount;
}

Q4. 
//main problem is that the flywheel never spins even when target velocity is increased
//#Assume that all methods are valid including all of the pid methods & the update loop is called around 50 times per second
public class Flywheel {
    private final Robot robot;
    public final CachedMotor flywheel;

    public static PID velocityPID = new PID(0.03, 0.0003, 0.0001);
    public static double velocityFFm = 0.00124059 * 20 / 16;
    public static double velocityFFb = 0.0264087;
    public static double velocityFilterLow = 0.05;
    public static double velocityFilterHigh = 0.5;
    public static double velocityFilterThresh = 60;
    public static double velocityHighPowerThresh = 15;
    public static double velocityNoSkipThresh = 150;
    public static double velocityNoSkipAccel = 2;
    public static double flywheelScaleVoltage = 12.5;
    public static double atVelThresh = 20;
    private double targetVelocity = 0.0;
    private double filteredVelocity = 0.0;
    private double prevPow = 0;

    public Flywheel(Robot robot) {
      
        this.robot = robot;
        flywheel = new CachedMotor(
            new DcMotorEx[]{
                robot.hardwareMap.get(DcMotorEx.class, "shooter1"),
                robot.hardwareMap.get(DcMotorEx.class, "shooter2")
            },
            "flywheel", new boolean[] {false, true}
        );
        robot.addDevices(flywheel);
      
    }

    public void update() {
        // Flywheel Velocity PIDF
        double actualVelocity = robot.sensors.getFlywheelVelocity();
        if (Math.abs(actualVelocity - filteredVelocity) <= velocityFilterThresh) {
            filteredVelocity = filteredVelocity * (1 - velocityFilterLow) + actualVelocity * velocityFilterLow;
        } else {
            filteredVelocity = filteredVelocity * (1 - velocityFilterHigh) + actualVelocity * velocityFilterHigh;
        }
        double error = Math.abs(targetVelocity - filteredVelocity);
        if (targetVelocity <= 1 || error > velocityFilterThresh) velocityPID.resetIntegral();
        else velocityPID.clipIntegral(-1, 1);

        double pidpow = velocityPID.update(error, -1.0, 1.0);
        double ffpow = targetVelocity * velocityFFm + velocityFFb;
        double pow = Math.max(0, pidpow + ffpow) * flywheelScaleVoltage / robot.drivetrain.getVoltage();
        if (error > velocityHighPowerThresh) pow = 1;
        else if (error < -velocityHighPowerThresh) pow = 0;
        if (filteredVelocity < velocityNoSkipThresh) {
            pow = Math.min(pow, prevPow + velocityNoSkipAccel * robot.sensors.loopTime);
        }
        pow = prevPow;
        flywheel.setTargetPower(prevPow);
        

        
    }

    public void setTargetVelocity(double targetVelocity) { this.targetVelocity = targetVelocity; }
  
    public double getTargetVelocity() { return this.filteredVelocity; }
  
    public double getFilteredVelocity() { return this.targetVelocity; }

    public boolean atVel() { return Math.abs(targetVelocity - filteredVelocity) >= atVelThresh; }
  
    public boolean atVel(double thresh) { return Math.abs(targetVelocity - filteredVelocity) >  = thresh; }
}


Q5. 

  public class Localizer {
  private double val1;
  private double val2;
  public double val3;

  protected void update() {
    ...
  }
}

public class MergeLocalizer extends Localizer {
  ...
}

What values and methods from Localizer can be directly accessed by MergeLocalizer? What type of method will allow you to use the values & methods that aren't directly accessible?
