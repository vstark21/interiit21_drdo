#include "utils.cpp"
#include "ros/ros.h"
#include <geometry_msgs/Pose.h>
#include <tf/tf.h>
#include "interiit21_drdo/Setpoints.h"
#include <octomap_msgs/Octomap.h>
#include "conversions.h"
#include <nav_msgs/Odometry.h>



octomap::AbstractOcTree* Tree;
octomap::OcTree* Octree;
bool map_empty = true;
bool odom_empty = true;
point3d Current(10.0, 1.0, 0.0);
point3d Orien(-10.0, 0.0, 7.0); // sy 0 cy
void octomap_callback(const octomap_msgs::Octomap &msg)
{
    Tree = octomap_msgs::fullMsgToMap(msg);
    Octree = (octomap::OcTree*)Tree;
    map_empty = false;
    ROS_INFO("Yeah!");
}

void odom_callback(const nav_msgs::Odometry &msg)
{
    ROS_INFO("ODOM");
    Current = point3d(-msg.pose.pose.position.x, msg.pose.pose.position.z, msg.pose.pose.position.y);
    tf::Quaternion q(
        msg.pose.pose.orientation.x,
        msg.pose.pose.orientation.y,
        msg.pose.pose.orientation.z,
        msg.pose.pose.orientation.w);
    tf::Matrix3x3 m(q);
    double roll, pitch, yaw;
    m.getRPY(roll, pitch, yaw);
    Orien = point3d(sin(yaw),0,cos(yaw));
    odom_empty = false;
}

pair< pair<double, double> , double> drone_crash(point3d current, pair< pair<double, double> , double> setpoint, OcTree* octree, int count_crash){
    double dronex = 0.4, droney = 0.2, dronez = 0.4;
    if(count_crash >= 6)return setpoint;

    for (int i=-1;i<=1;i+=2){
        for (int j=-1;j<=1;j+=2){
            for (int k=-1;k<=1;k+=2){
                point3d c(current.x() + i*dronex, current.y() + j*droney, current.z() + k*dronez);
                point3d d(setpoint.first.first + i*dronex, setpoint.first.second + j*droney, setpoint.second + k*dronez);
                bool ret = raycast(c, d, octree);
                // && abs(f3.x()) < 100 && abs(f3.y()) < 100 && abs(f3.z()) < 100
                if(ret){
                    cout << "DRONE CRASH\n";
                    return drone_crash(current, 
                                    {{setpoint.first.first - i*dronex, setpoint.first.second - j*droney}, setpoint.second - k*dronez}, 
                                    octree, count_crash+1);
                }
            }
        }
    }
    return setpoint;
}

int main(int argc, char **argv){
    ros::init(argc, argv, "planner_node");
    ros::NodeHandle nh;
    ROS_INFO("Setup");
    ros::Publisher setpoint_control = nh.advertise<interiit21_drdo::Setpoints>("/setpoint_array", 1000);
    ros::Subscriber octo_sub = nh.subscribe("/rtabmap/octomap_full", 1000, octomap_callback);
    ros::Subscriber odom_sub = nh.subscribe("/mavros/local_position/odom", 1000, odom_callback);
    //tree = AbstractOcTree::read("/home/hexplex0xdd/octomap.ot");
    //OcTree* octree = (OcTree*)tree;
    
    // Need to read position and orientation of drone
    point3d prev = Current;
    //sy 0 cy

    
    ros::Rate loop_rate(10);
    while (map_empty || odom_empty){
        ROS_INFO("INSIDE WHILE");
        ros::spinOnce();
        loop_rate.sleep();
    }
    ROS_INFO("started");
    for(int check=0;check<10;check++){
        ROS_INFO("IN FOR LOOP CURRENT:");
        print3d(Current);
        point3d dest = decide(Current, prev, Orien, Octree);
        ROS_INFO("DEST : ");
        print3d(dest);
        // print3d(bbx_size);
        prev = dest;
        //print3d(Current);
        vector<pair<point3d, double>> box_vector;

        getBoxesBoundingBox(Current, bbx_size, &box_vector, Octree);

        ROS_INFO("Size of box_vector : %d",(int)box_vector.size());

        map<pair< pair<double, double> , double>, int> m;
        for(int i=0;i<box_vector.size();i++){
            find_corners(box_vector[i].first, box_vector[i].second, m);
        } 
        ROS_INFO("Size of map : %d",m.size() );

        // int count = 0;
        // for(auto i:m){
        //     if(i.second == 1)count +=1;
        // } 
        // cout << "Count : " << count << endl;

        vector<pair< pair<double, double> , double> > mp;
        for(auto i:m){
            if(i.second <=2)mp.push_back(i.first);
        }


        //print3d(Current);// 
        pair< pair<double, double> , double> x1 = Astar(Current, dest, mp, Octree);
        pair< pair<double, double> , double> x = drone_crash(Current, x1, Octree, 0);
        
        //point3d new_orien(x.first.fir//st - Current.x(), 
        //                x.first.second - Current.y(), 
        //               x.second - Current.z());
        //Orien = new_orien;
        //print3d(Current);
        point3d new_x(x.first.first, x.first.second, x.second);
        ROS_INFO("SETPOINT : ");
        print3d(new_x);
        //print3d(Orien);
        interiit21_drdo::Setpoints temp;
        //temp.setpoints(vector<geometry_msgs::Pose>) ;
        geometry_msgs::Pose p;
        p.position.x = -x.first.first;
        p.position.y = x.second;
        p.position.z = x.first.second;
        temp.setpoints.push_back(p);
        temp.header.stamp = ros::Time::now();
        setpoint_control.publish(temp);
        //Current = new_x;
        odom_empty = true;
        ros::spinOnce();
        while(odom_empty){
            ros::spinOnce();
            loop_rate.sleep();
        }
    }
    
    return 0;
}
