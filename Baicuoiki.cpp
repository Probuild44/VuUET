#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <turtlesim/Pose.h>
#include <turtlesim/Spawn.h>
#include <iostream>
#include <algorithm>
#include <sstream>
using namespace std;

const float PI = 3.14159265;
float rate = 100;

geometry_msgs::Twist getMessage(double linear_x, double angular_z)
{
    geometry_msgs::Twist msg;
    msg.linear.x = linear_x;
    msg.angular.z = angular_z;
    return msg;
}

class PoseCallback {
public:
    int turtle_idx;
    ros::Subscriber sub;
    turtlesim::Pose current_pose;

    void callback(const turtlesim::Pose::ConstPtr& msg)
    {
        cout << "turtle " << turtle_idx+1 << " " << msg->x << " " << msg->y << endl;
        current_pose = *msg;
    }
};


int main(int argc, char** argv)
{
    int n_turtle = atoi(argv[1]);

    ros::init(argc, argv, "myturtle_control");
    ros::NodeHandle h;
    ros::Publisher pub[n_turtle];
    PoseCallback sub[n_turtle];

    for(int i = 1;i< n_turtle; i++)
    {
        ros::service::waitForService("spawn");
        ros::ServiceClient spawner = h.serviceClient<turtlesim::Spawn>("spawn");
        turtlesim::Spawn turtle;
        turtle.request.x = rand() % 12;// #include <turtlesim/Spawn.h>
        turtle.request.y = rand() % 12;
        turtle.request.theta =0;
        spawner.call(turtle);
    }
    cout << "n_turtle = " << n_turtle << endl;
    for (int i = 0; i < n_turtle; i++) {
        stringstream s;
        s << "turtle" << i+1;
        string name = s.str();

        pub[i] = h.advertise<geometry_msgs::Twist>(name + "/cmd_vel", 1000);
        sub[i].turtle_idx = i;
        sub[i].sub = h.subscribe(name+"/pose", 1000, &PoseCallback::callback, &sub[i]);
        cout << "subcribe turtle " << i << " to " << name << "/pose" << endl;
    }
    ros::Rate loopRate(rate);
    for (int i = 2; i < argc; i+=2) 
    {
        double x0 = atof(argv[i]), y0 = atof(argv[i+1]);
        const double tolerance = 1e-2;
            while (ros::ok())
            {
                loopRate.sleep();
                ros::spinOnce();
                bool exit = false;
                for (int idx = 0; idx < n_turtle; idx++)
                {
                    cout << sub[idx].current_pose.x << " " << sub[idx].current_pose.y << " " << sub[idx].current_pose.theta << endl;

                    double distance = sqrt( pow(x0-sub[idx].current_pose.x, 2) + pow(y0-sub[idx].current_pose.y, 2) );
                    if (distance < tolerance) {
                        pub[idx].publish(getMessage(0,0));
                        exit = true;
                        
                    }

                    //double alpha = atan2( y0-current_pose.y, x0-current_pose.x ),a_z;
                    double dx = x0 - sub[idx].current_pose.x, dy = y0 - sub[idx].current_pose.y, 
                        theta = sub[idx].current_pose.theta;
                    double dalpha = 0 ;
                    double scalar_product = cos(theta)*dx + sin(theta)*dy;
                    if(scalar_product >= 0)
                    {
                        if(distance < 1e-6)
                        {
                        dalpha = 0;
                        cout << dalpha << endl;
                        }
                        else
                        {
                        dalpha = asin ((cos(theta)*dy-sin(theta)*dx) / distance);
                        cout << dalpha << endl;
                        }
                        geometry_msgs::Twist msg = getMessage(
                        min(20*distance, 15.0),
                        20*dalpha
                        );
                        pub[idx].publish(msg);
                    }
                    else
                    {
                        if(distance < 1e-6)
                        {
                        dalpha = 0;
                        cout << dalpha << endl;
                        }
                        else
                        {
                        dalpha = asin ((cos(theta + PI)*dy-sin(theta + PI)*dx) / distance);
                        cout << dalpha << endl;
                        }
                        geometry_msgs::Twist msg = getMessage(
                        min(20*distance, 15.0)*(-1),
                        20*dalpha
                        );
                        pub[idx].publish(msg);

                    }
                    
                    
                }
                if(exit == true) {break;}
            }
    }
    
    return 0;
}
