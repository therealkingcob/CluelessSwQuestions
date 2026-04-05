Q1. 

def count_streak(arr, target):
    count = 0
    max_count = 0

    for x in arr:
        if x == target:
            count += 1
        else:
            count = 0
        max_count = count  

    return max_count

Q2. 

def longest_increasing_streak(arr):
    if not arr:
        return 0

    count = 1
    max_count = 1

    for i in range(1, len(arr)):
        if arr[i] > arr[i - 1]:
            count += 1
        else:
            count = 0  

        if count > max_count:
            max_count = count

    return max_count


Q3. 

class Flywheel:
    velocityPID = PID(0.03, 0.0003, 0.0001)
    velocityFFm = 0.00124059 * 20 / 16
    velocityFFb = 0.0264087
    velocityFilterLow = 0.05
    velocityFilterHigh = 0.5
    velocityFilterThresh = 60
    velocityHighPowerThresh = 15
    velocityNoSkipThresh = 150
    velocityNoSkipAccel = 2
    flywheelScaleVoltage = 12.5
    atVelThresh = 20

    def __init__(self, robot):
        self.robot = robot
        self.flywheel = CachedMotor(
            [robot.hardwareMap.get("DcMotorEx", "shooter1"),
             robot.hardwareMap.get("DcMotorEx", "shooter2")],
            "flywheel",
            [False, True]
        )
        robot.addDevices(self.flywheel)
        self.targetVelocity = 0.0
        self.filteredVelocity = 0.0
        self.prevPow = 0

    def update(self):
        actualVelocity = self.robot.sensors.getFlywheelVelocity()

        if abs(actualVelocity - self.filteredVelocity) <= self.velocityFilterThresh:
            self.filteredVelocity = self.filteredVelocity * (1 - self.velocityFilterLow) + actualVelocity * self.velocityFilterLow
        else:
            self.filteredVelocity = self.filteredVelocity * (1 - self.velocityFilterHigh) + actualVelocity * self.velocityFilterHigh

        error = abs(self.targetVelocity - self.filteredVelocity)
        if self.targetVelocity <= 1 or error > self.velocityFilterThresh:
            self.velocityPID.resetIntegral()
        else:
            self.velocityPID.clipIntegral(-1, 1)

        pidpow = self.velocityPID.update(error, -1.0, 1.0)
        ffpow = self.targetVelocity * self.velocityFFm + self.velocityFFb
        pow = max(0, pidpow + ffpow) * self.flywheelScaleVoltage / self.robot.drivetrain.getVoltage()

        if error > self.velocityHighPowerThresh:
            pow = 1
        elif error < -self.velocityHighPowerThresh:
            pow = 0

        if self.filteredVelocity < self.velocityNoSkipThresh:
            pow = min(pow, self.prevPow + self.velocityNoSkipAccel * self.robot.sensors.loopTime)

        pow = self.prevPow  
        self.flywheel.setTargetPower(self.prevPow)  

    def set_target_velocity(self, targetVelocity):
        self.targetVelocity = targetVelocity

    def get_target_velocity(self):
        return self.filteredVelocity  

    def get_filtered_velocity(self):
        return self.targetVelocity  

    def at_vel(self):
        return abs(self.targetVelocity - self.filteredVelocity) >= self.atVelThresh  

    def at_vel_thresh(self, thresh):
        return abs(self.targetVelocity - self.filteredVelocity) >= thresh  
