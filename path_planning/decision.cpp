#include "main.cpp"
// #include "ros/ros.h"
// #include "geometry_msgs/Pose.h"
// #include "interiit21_drdo/Setpoints.h"


point drone_crash(point3d current, point setpoint, OcTree* octree, int count_crash){
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
    // ros::init(argc, argv, "planner_node");
    // ros::NodeHandle nh;
    // ros::Publisher setpoint_control = nh.advertise<interiit21_drdo::String>("/setpoint_array", 1000);
    AbstractOcTree* tree = AbstractOcTree::read("/home/vishwas/Downloads/octomap.ot");
    OcTree* octree = (OcTree*)tree;

    // Need to read position and orientation of drone
    point3d current(10.0, 1.0, 0.0);
    point3d prev = current;
    point3d orien(-10.0, 0.0, 7.0);
    
    // temp = interiit21_drdo::Setpoints();
    // temp.setpoints = vector<geomtry_msgs::Pose>;
    for(int check=0;check<10;check++){
        point3d dest = decide(current, prev, orien, octree);
        cout << "DEST : ";
        print3d(dest);
        // print3d(bbx_size);
        prev = dest;

        vector<pair<point3d, double>> box_vector;

        getBoxesBoundingBox(current, bbx_size, &box_vector, octree);

        cout << "Size of box_vector : " << box_vector.size() << endl;

        map<point, int> m;
        for(int i=0;i<box_vector.size();i++){
            find_corners(box_vector[i].first, box_vector[i].second, m);
        } 
        cout << "Size of map : " << m.size() << endl;

        // int count = 0;
        // for(auto i:m){
        //     if(i.second == 1)count +=1;
        // } 
        // cout << "Count : " << count << endl;

        vector<point> mp;
        for(auto i:m){
            if(i.second <=2)mp.push_back(i.first);
        }



        point x1 = Astar(current, dest, mp, octree);
        point x = drone_crash(current, x1, octree, 0);
        
        point3d new_orien(x.first.first - current.x(), 
                        x.first.second - current.y(), 
                        x.second - current.z());
        orien = new_orien;
        point3d new_x(x.first.first, x.first.second, x.second);
        cout << "CURRENT : ";
        print3d(new_x);
        print3d(orien);
        
        // p = geometry_msgs::Pose();
        // p.position.x = x.first.first;
        // p.position.y = x.first.second;
        // p.position.z = x.second;
        // temp.setpoints.pusb_back(p);
        current = new_x;
    }
    // temp.header.stamp = ros::Time::now();
    // setpoint_control.publish(temp);
    return 0;
}