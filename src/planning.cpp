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
point3d Current(-3.11, 1.2, 2.99);
point3d Orien(-0.5, 0.5, 0.0); // sy 0 cy


/* Callback function to recieve octomap generated and published by rtabmap.*/
void octomap_callback(const octomap_msgs::Octomap &msg)
{
    Tree = octomap_msgs::fullMsgToMap(msg);
    Octree = (octomap::OcTree*)Tree;
    map_empty = false;
    ROS_INFO("Yeah!");
}

/* Callback function to recieve odometry data published by mavros.*/
void odom_callback(const nav_msgs::Odometry &msg)
{
    ROS_INFO("ODOM");
    Current = point3d(msg.pose.pose.position.x, msg.pose.pose.position.y, msg.pose.pose.position.z);
    tf::Quaternion q(
        msg.pose.pose.orientation.x,
        msg.pose.pose.orientation.y,
        msg.pose.pose.orientation.z,
        msg.pose.pose.orientation.w);
    tf::Matrix3x3 m(q);
    tf::Vector3 dummy(1,0,0);
    tf::Vector3 final = m*dummy;
    Orien = point3d(final.x(), final.y(), final.z());
    odom_empty = false;
}

/* Returns a point around given setpoint which is safe for the quadcopter to navigate from current to that point. */
pair< pair<double, double> , double> drone_crash(point3d current, pair< pair<double, double> , double> setpoint, OcTree* octree, int count_crash, double fac=0.1){
    double fac_x=fac, fac_y=fac, fac_z=fac;
    double dronex = 0.35, droney = 0.35, dronez = 0.25;
    if(count_crash >= 15)return setpoint;

    for (int i=-1;i<=1;i+=2){
        for (int j=-1;j<=1;j+=2){
            for (int k=-1;k<=1;k+=2){
                point3d c(current.x() + i*dronex, current.y() + j*droney, current.z() + k*dronez);
                point3d d(setpoint.first.first + i*dronex, setpoint.first.second + j*droney, setpoint.second + k*dronez);
                bool ret = raycast(c, d, octree);
                // && abs(f3.x()) < 100 && abs(f3.y()) < 100 && abs(f3.z()) < 100
                if(ret){
                    ROS_ERROR("DRONE CRASH\n");
                    return drone_crash(current, 
                        {{setpoint.first.first - i*fac_x, setpoint.first.second - j*fac_y}, setpoint.second - k*fac_z} , 
                                    octree, count_crash+1, fac );
                }
            }
        }
    }
    return setpoint;
}

/* Takes in current position(current) and next setpoint(sp) and returns a point between current and sp. */
pair<pair<double, double> , double> step(point3d current, pair<pair<double, double> , double> sp){
    point3d te(sp.first.first, sp.first.second, sp.second);
    double val = l2_norm(te,current);
    if (val<=0.8)return sp;
    te-=current;
    te = te.normalize();
    te*= 0.8;
    te+=current;
    
    pair<pair<double, double> , double> setpoint;
    setpoint.first.first =te.x(); // (4*current.x() + sp.first.first) / 5.0;
    setpoint.first.second =te.y(); // (4*current.y() + sp.first.second) / 5.0;
    setpoint.second =te.z(); // (4*current.z() + sp.second) / 5.0;
    return setpoint;
}

int main(int argc, char **argv){
    /* Initializing node, subscribing to required topics and creating different publishers to publish data to required topics. */
    ros::init(argc, argv, "planner_node");
    ros::NodeHandle nh;
    ROS_INFO("Setup");
    ros::Publisher setpoint_control = nh.advertise<interiit21_drdo::Setpoints>("/setpoint_array", 1000);
    ros::Publisher dest_pub = nh.advertise<geometry_msgs::PoseStamped>("/dest_pose", 1000);
    ros::Subscriber octo_sub = nh.subscribe("/rtabmap/octomap_full", 1000, octomap_callback);
    ros::Subscriber odom_sub = nh.subscribe("/mavros/local_position/odom", 1000, odom_callback);

    // octomap::AbstractOcTree* tree = AbstractOcTree::read("/home/vishwas/Downloads/octomap_updated.ot");
    // OcTree* Octree = (OcTree*)tree;


    ros::Rate loop_rate(10);
    while (ros::ok() &&(map_empty || odom_empty)){
        ROS_INFO("INSIDE WHILE");
        ros::spinOnce();
        loop_rate.sleep();
    }
    point3d ref_current = Current;
    point3d prev = ref_current, ref_orien = Orien;
    point3d dest;
    octomap::OcTree* ref_octree = Octree;
    ROS_INFO("started");
    //loc = ref_current;
    for(int check=0;ros::ok()&&(check>-1);check++){
    // for(int check=0;check<1;check++){
        
        ROS_INFO("IN FOR LOOP CURRENT:");
        print3d(ref_current);
        if (check%1==0)
            dest = decide(ref_current, prev, ref_orien, ref_octree);
        ROS_INFO("DEST : ");
        print3d(dest);
        geometry_msgs::PoseStamped dest_tmp;
        dest_tmp.pose.position.x = dest.x();
        dest_tmp.pose.position.y = dest.y();
        dest_tmp.pose.position.z = dest.z();
        dest_tmp.header.stamp = ros::Time::now();
        dest_tmp.header.frame_id = "map";
        dest_tmp.pose.orientation.w = 1;
        dest_pub.publish(dest_tmp);
        // print3d(bbx_size);
        prev = dest;
        //print3d(Current);
        vector<pair<point3d, double>> box_vector;

        getBoxesBoundingBox(ref_current, bbx_size, &box_vector, ref_octree);

        ROS_INFO("Size of box_vector : %d",(int)box_vector.size());

        map<pair< pair<double, double> , double>, int> m;
        for(int alp=0;alp<box_vector.size();alp++){
            find_corners(box_vector[alp].first, box_vector[alp].second, m);
        } 
        ROS_INFO("Size of map : %d",(int)m.size() );

        // int count = 0;
        // for(auto i:m){
        //     if(i.second == 1)count +=1;
        // } 
        // cout << "Count : " << count << endl;

        vector<pair< pair<double, double> , double> > mp;
        for(auto i:m){
            if(i.second<=8.0 && !check_occupancy(point3d(i.first.first.first, i.first.first.second, i.first.second),ref_octree, 0.3))mp.push_back(i.first);//   
        }

        ROS_INFO("Size of mp : %d",(int)mp.size() );
        //return 0;
        //print3d(Current);// 
        pair< pair<double, double> , double> x1 = Astar(ref_current, dest, mp, ref_octree,ref_orien);
        x1 = step(ref_current, x1);
        pair< pair<double, double> , double> x = drone_crash(ref_current, x1, ref_octree, 0);
        
        //point3d new_orien(x.first.fir//st - Current.x(), 
        //                x.first.second - Current.y(), 
        //               x.second - Current.z());
        //Orien = new_orien;
        //print3d(Current);
        repel.push_back(x);
        point3d new_x(x.first.first, x.first.second, x.second);
        ROS_INFO("SETPOINT : ");
        print3d(new_x);
        //print3d(Orien);
        interiit21_drdo::Setpoints temp;
        //temp.setpoints(vector<geometry_msgs::Pose>) ;
        geometry_msgs::Pose p;
        p.position.x = x.first.first;
        p.position.y = x.first.second;
        p.position.z = x.second;
        octomath::Vector3 tem = new_x - ref_current ;
        tf::Vector3 D = tf::Vector3(tem.x(),tem.y(),tem.z()).normalize();
        tf::Vector3 S = tf::Vector3(0,0,1).cross(D);
        tf::Vector3 U = D.cross(S);
        /*tf::Matrix3x3 m_temp(D.x(),D.y(),D.z(),
                        S.x(),S.y(),S.z(),
                        U.x(),U.y(),U.z());
        double t_r,t_p,t_y;
        m_temp.getRPY(t_r, t_p, t_y);
        tf::Quaternion q();*/
        double angle = atan2(tem.y(), tem.x());
        double q_w = cos(angle/2);//sqrt(1.0 + D.x() + S.y() + U.z()) / 2.0;
	    double q_w4 = (4.0 * q_w);
	    double q_x = 0;//(U.y() - S.z()) / q_w4 ;
	    double q_y = 0;//(D.z() - U.x()) / q_w4 ;
	    double q_z = sin(angle/2);
        p.orientation.x = q_x;
        p.orientation.y = q_y;
        p.orientation.z = q_z;
        p.orientation.w = q_w;
        ROS_INFO("quat %f %f %f %f",q_x, q_y, q_z, q_w);
        print3d(octomath::Vector3(D.x(),D.y(),D.z()));
        print3d(octomath::Vector3(S.x(),S.y(),S.z()));
        print3d(octomath::Vector3(U.x(),U.y(),U.z()));
        temp.setpoints.push_back(p);
        temp.header.stamp = ros::Time::now();
        temp.header.frame_id = "map";
        setpoint_control.publish(temp);
        //Current = new_x;
        odom_empty = true;
        ros::spinOnce();
        while(ros::ok()&&(odom_empty || l2_norm(x,{{Current.x(), Current.y()}, Current.z()})>0.3)){
            ros::spinOnce();
            loop_rate.sleep();
        }
        //ros::Duration(2.0).sleep();
        loc = ref_current;
        ref_orien = Orien;
        ref_current = Current;
        ref_octree = Octree;
    }
    
    return 0;
}
