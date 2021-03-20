#include "main.cpp"
#include "ros/ros.h"
#include "geometry_msgs/Pose.h"
#include "interiit21_drdo/Setpoints.h"
point3d bbx_size(10.0, 2.0, 10.0);

point3d prec(point3d node){
    double factor = 100.0;
    int x = node.x() * factor; 
    int y = node.y() * factor; 
    int z = node.z() * factor; 
    point3d prec_node(x / factor, y / factor, z / factor);
    return prec_node;
}

bool check_occupancy(point3d g, OcTree* octree){
    point3d size(0.5, 0.5, 0.5);
    point3d bbx_min = g - size;
    point3d bbx_max = g + size;

    for (OcTree::leaf_bbx_iterator
           it = octree->begin_leafs_bbx(bbx_min, bbx_max),
           end = octree->end_leafs_bbx();
       it != end; ++it) {
           if(octree->isNodeOccupied(*it))return true;
    }
    return false;
}

point3d decide(point3d current, point3d prev, point3d orien, OcTree* octree){

    double length = 3.0;
    point3d pull;
    if(current.y() <= 3.0){
        point3d pull_(0.0, 0.1*pow(3.0 - current.y(), 2), 0.0);
        pull = pull_;
    }
    else{
        point3d pull_(0.0, -0.1 * pow(3.0 - current.y(), 2), 0.0);
        pull = pull_;
    }

    point3d f1(orien.x(), 0.0, orien.z());
    point3d f2(prev.x() - current.x(), 0.0, prev.z() - current.z());

    
    point3d orien_norm = f1.normalize();

    point3d prev_norm = f2.normalize();
    
    // if(prev_norm.y() + orien_norm.y() )
    point3d new_dir = prev_norm + orien_norm + pull;
    point3d new_dir_norm = new_dir.normalize();

    new_dir = prec(new_dir_norm * length);

    while(check_occupancy(new_dir + current, octree)){
        length += 0.5;
        new_dir = prec(new_dir_norm * length);
    }
    // point3d new_bbx(length, bbx_size.y(), length);
    // bbx_size = new_bbx;
    cout << length << endl;
    if(new_dir.y() + current.y() >= 0.5 && new_dir.y() + current.y() <= 4.5){
        return new_dir + current;
    }
    if(new_dir.y() + current.y() < 0.5){
        point3d ans(new_dir.x() + current.x(), 0.5, new_dir.z() + current.z());
        return ans;
    }
    point3d ans(new_dir.x() + current.x(), 4.5, new_dir.z() + current.z());

    return ans;
}



int main(int argc, char **argv){
    ros::init(argc, argv, "planner_node");
    ros::NodeHandle nh;
    ros::Publisher setpoint_control = nh.advertise<interiit21_drdo::String>("/setpoint_array", 1000);
    AbstractOcTree* tree = AbstractOcTree::read("/home/vishwas/Downloads/octomap.ot");
    OcTree* octree = (OcTree*)tree;

    point3d current(10.0, 1.0, 0.0);
    
    point3d prev = current;
    point3d orien(-10.0, 0.0, 7.0);
    temp = interiit21_drdo::Setpoints();
    temp.setpoints = vector<geomtry_msgs::Pose>;
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



        point x = Astar(current, dest, mp, octree);
        
        point3d new_orien(x.first.first - current.x(), 
                        x.first.second - current.y(), 
                        x.second - current.z());
        orien = new_orien;
        point3d new_x(x.first.first, x.first.second, x.second);
        cout << "CURRENT : ";
        print3d(new_x);
        print3d(orien);
        
        p = geometry_msgs::Pose();
        p.position.x = x.first.first;
        p.position.y = x.first.second;
        p.position.z = x.second;
        temp.setpoints.pusb_back(p);
        current = new_x;
    }
    temp.header.stamp = ros::Time::now();
    setpoint_control.publish(temp);
    return 0;
}