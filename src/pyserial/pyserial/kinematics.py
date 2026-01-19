import math
def p2rps(pulse,ppr,time):
    return pulse/ppr/time
class DiffKinematic:
    def __init__(self,lr_distance,wheel_diameter):
        self.lr_distance = lr_distance
        self.wheel_radius = wheel_diameter / 2.0
        self.wheel_circumference = math.pi * wheel_diameter
        self.total_wheels = 2.0
        self.max_rpm =  ((24 / 23) * 498) * 0.85

    def forward_kinematic(self,rpm1, rpm2):
        """
        Kinematics::velocities vel;

        float average_rpm_x = (float)(motor1 + motor2) / 2; // RPM
        //convert revolutions per minute to revolutions per second
        float average_rps_x = average_rpm_x / 60; // RPS
        vel.linear_x = (average_rps_x * circumference_); // m/s

        float average_rpm_a = (float)(motor2 - motor1) / 2;
        //convert revolutions per minute to revolutions per second
        float average_rps_a = average_rpm_a / 60;
        vel.angular_z =  (average_rps_a * circumference_) / (lr_wheels_dist_ / 2);

        return vel;
        """
        
        # convert average revolutions per minute to revolutions per second
        average_rps_x = (rpm1 + rpm2 ) / self.total_wheels
        vel_linear_x = average_rps_x * self.wheel_circumference; # m/s

        # convert average revolutions per minute in y axis to revolutions per second
        average_rps_y = 0.0
        
        # convert average revolutions per minute to revolutions per second
        average_rps_a = (-rpm1 + rpm2) / self.total_wheels
        vel_angular_z =  (average_rps_a * self.wheel_circumference) / (self.lr_distance / 2.0); # rad/s
        # v = 2 * math.pi * self.wheel_radius * ((rpmL/2) + (rpmR/2))
        # w = 2 * math.pi * self.wheel_radius * (-(rpmL/self.robot_radius) + (rpmR/self.robot_radius))
        return vel_linear_x,vel_angular_z

    def inverse_kinematic(self,linear_x,linear_y,angular_z):

        tangential_vel = angular_z * (self.lr_distance / 2.0)

        x_rpm = linear_x / self.wheel_circumference
        y_rpm = linear_y / self.wheel_circumference
        tan_rpm = tangential_vel / self.wheel_circumference

        a_x_rpm = abs(x_rpm)
        a_y_rpm = abs(y_rpm)
        a_tan_rpm = abs(tan_rpm)

        xy_sum = a_x_rpm + a_y_rpm
        xtan_sum = a_x_rpm + a_tan_rpm

        # calculate the scale value how much each target velocity
        # must be scaled down in such cases where the total required RPM
        # is more than the motor's max RPM
        # this is to ensure that the required motion is achieved just with slower speed
        if xy_sum >= self.max_rpm and angular_z == 0:
        
            vel_scaler = self.max_rpm / xy_sum

            x_rpm *= vel_scaler
            y_rpm *= vel_scaler
        
        elif xtan_sum >= self.max_rpm and linear_y == 0:
        
            vel_scaler = self.max_rpm / xtan_sum

            x_rpm *= vel_scaler
            tan_rpm *= vel_scaler
        
        rpm1 = x_rpm - y_rpm - tan_rpm # left
        rpm2 = x_rpm + y_rpm + tan_rpm # right

        # thetaL = 1 / (2 * math.pi * self.wheel_radius) * (v + -((self.robot_radius / 2) * w))
        # thetaR = 1 / (2 * math.pi * self.wheel_radius) * (v +  ((self.robot_radius / 2) * w))
        return rpm1,rpm2