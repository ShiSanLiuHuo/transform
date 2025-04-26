#include<iostream>
#include<cmath>

class Euler_angle;

class Quaternions {
public:
    Quaternions() = default;
    Quaternions(double w0, double x0, double y0, double z0)
        : w(w0), x(x0), y(y0), z(z0) {};

    Quaternions operator-(const Quaternions& other) const {
        return Quaternions(
            w - other.w,
            x - other.x,
            y - other.y,
            z - other.z
        );
    }

    double Quaternions_module() const {
        return std::sqrt(w*w + x*x + y*y + z*z);
    }

    Quaternions conjugate() const {
        return Quaternions(w, -x, -y, -z);
    }

    Quaternions inverse() const {
        double q_module = Quaternions_module();
        if (q_module == 0) {
            std::cerr << "module is zero" << std::endl;
            return Quaternions(0, 0, 0, 0);
        }
        Quaternions q_conj = conjugate();
        double scale = 1.0 / (q_module * q_module);
        return Quaternions(
            q_conj.w * scale,
            q_conj.x * scale,
            q_conj.y * scale,
            q_conj.z * scale
        );
    }

    Quaternions operator*(const Quaternions& other) const {
        double new_w = w*other.w - x*other.x - y*other.y - z*other.z;
        double new_x = w*other.x + x*other.w + y*other.z - z*other.y;
        double new_y = w*other.y - x*other.z + y*other.w + z*other.x;
        double new_z = w*other.z + x*other.y - y*other.x + z*other.w;
        return Quaternions(new_w, new_x, new_y, new_z);
    }

    Quaternions operator+(const Quaternions& other) const {
        return Quaternions(w + other.w, x + other.x, y + other.y, z + other.z);
    }

    friend Euler_angle Quaternions2Euler_angle(const Quaternions& q);
    friend std::ostream& operator<<(std::ostream& os, const Quaternions& q);

private:
    double w;
    double x;
    double y;
    double z;
};

class Euler_angle {
public:
    Euler_angle() = default;
    Euler_angle(double roll_0, double pitch_0, double yaw_0)
        : roll(roll_0), pitch(pitch_0), yaw(yaw_0) {};

    friend Quaternions Euler_angle2Quaternions(const Euler_angle& e);
    friend std::ostream& operator<<(std::ostream& os, const Euler_angle& e);

private:
    double roll;
    double pitch;
    double yaw;
};

Quaternions Euler_angle2Quaternions(const Euler_angle& e) {
    double cr = cos(e.roll / 2), sr = sin(e.roll / 2);
    double cp = cos(e.pitch / 2), sp = sin(e.pitch / 2);
    double cy = cos(e.yaw / 2), sy = sin(e.yaw / 2);

    double w = cr * cp * cy + sr * sp * sy;
    double x = sr * cp * cy - cr * sp * sy;
    double y = cr * sp * cy + sr * cp * sy;
    double z = cr * cp * sy - sr * sp * cy;

    return Quaternions(w, x, y, z);
}

Euler_angle Quaternions2Euler_angle(const Quaternions& q) {
    double mod = q.Quaternions_module();
    if (mod == 0) {
        std::cerr << "Quaternion module is zero!" << std::endl;
        return Euler_angle(0, 0, 0);
    }

    double w = q.w / mod, x = q.x / mod, y = q.y / mod, z = q.z / mod;

    double sinp = 2 * (w * y - z * x);
    if (fabs(sinp) >= 1.0) {
        std::cerr << "Euler_angle is 90!!!" << std::endl;
        return Euler_angle(
            atan2(2 * (w * x + y * z), 1 - 2 * (x * x + y * y)),
            copysign(M_PI / 2, sinp),
            0
        );
    }

    double yaw = atan2(2 * (w * z + x * y), 1 - 2 * (y * y + z * z));
    double pitch = asin(sinp);
    double roll = atan2(2 * (w * x + y * z), 1 - 2 * (x * x + y * y));
    
    return Euler_angle(roll, pitch, yaw);
}

std::ostream& operator<<(std::ostream& os, const Quaternions& q) {
    os << "(" << q.x << ", " << q.y<< ", " << q.z << ")";
    return os;
}

std::ostream& operator<<(std::ostream& os, const Euler_angle& e) {
    os << "(yaw:" << e.yaw << ", pitch:" << e.pitch << ", roll:" << e.roll << ")";
    return os;
}

Quaternions attitude_transform(const Quaternions& q1, const Quaternions& q2) {
    return q1 * q2;
}

Quaternions position_transform(const Quaternions& q, const Quaternions& t1, const Quaternions& t2) {
    Quaternions rotated = q * t1 * q.inverse();
    return rotated + t2; 
}

// 初始化参数
Quaternions Gimbal2Camera_position(0,2,0,0);
Quaternions Odom2Gimbal_position(0,0,0,0);
Euler_angle Gimbal2Camera_Euler_angle(0,0,0);
Euler_angle Odom2Gimbal_Euler_angle(-0.1,-0.1,-0.1);

Quaternions Gimbal2Camera_Quaternions = Euler_angle2Quaternions(Gimbal2Camera_Euler_angle);
Quaternions Odom2Gimbal_Quaternions = Euler_angle2Quaternions(Odom2Gimbal_Euler_angle);


Quaternions Camera2Gimbal_position = Gimbal2Camera_Quaternions.inverse() * Gimbal2Camera_position;
Quaternions Gimbal2Odom_position = Odom2Gimbal_Quaternions.inverse() * Odom2Gimbal_position;


int main() {
    double x0, y0, z0, roll_0, pitch_0, yaw_0;
    std::cin >> x0 >> y0 >> z0 >> roll_0 >> pitch_0 >> yaw_0;

    Quaternions Object2Camera_position(0, x0, y0, z0);
    Euler_angle Object2Camera_Euler_angle(roll_0, pitch_0, yaw_0);
    Quaternions Object2Camera_Quaternions = Euler_angle2Quaternions(Object2Camera_Euler_angle);

    std::string s;
    std::cin.ignore();
    std::getline(std::cin, s);


    std::cout << Odom2Gimbal_Quaternions << std::endl;
    
    if (s == "transform to /Gimbal") {
        

        Quaternions Object2Gimbal_position = position_transform(
            Gimbal2Camera_Quaternions.inverse(),
            Object2Camera_position,
            Camera2Gimbal_position
        );

        Quaternions Object2Gimbal_attitude = attitude_transform(
            Gimbal2Camera_Quaternions.inverse(),
            Object2Camera_Quaternions
        );

        Euler_angle Object2Gimbal_Euler = Quaternions2Euler_angle(Object2Gimbal_attitude);
        std::cout << Object2Gimbal_position << Object2Gimbal_Euler << std::endl;
    } 
    else if (s == "transform to /Odom") {
        Quaternions Object2Gimbal_position = position_transform(
            Gimbal2Camera_Quaternions.inverse(),
            Object2Camera_position,
            Camera2Gimbal_position  //inverse()????????????????
        );

        Quaternions Object2Gimbal_attitude = attitude_transform(
            Gimbal2Camera_Quaternions.inverse(),
            Object2Camera_Quaternions
        );

        std::cout << Object2Gimbal_attitude << std::endl;

        Quaternions Object2Odom_position = position_transform(
            Odom2Gimbal_Quaternions.inverse(),
            Object2Gimbal_position,
            Gimbal2Odom_position
        );

        Quaternions Object2Odom_attitude = attitude_transform(
            Odom2Gimbal_Quaternions.inverse(),
            Object2Gimbal_attitude
        );

        std::cout << Object2Odom_attitude <<std::endl;

        Euler_angle Object2Odom_attitude_Euler_angle = Quaternions2Euler_angle(Object2Odom_attitude);
        std::cout << Object2Odom_position << Object2Odom_attitude_Euler_angle << std::endl;
    }

    return 0;
}