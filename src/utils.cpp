// #include <iostream>
// #include<windows.h>
// #include <time.h>
#include <unistd.h>
// #include <octomap_msgs/Octomap.h>
// #include "conversions.h" 
#include <octomap/AbstractOcTree.h>
#include <octomap/OcTree.h>
#include <octomap/OccupancyOcTreeBase.h>
#include <octomap/octomap.h>
#include <bits/stdc++.h>
#include <ros/console.h>

using namespace std;
using namespace octomap;

point3d bbx_size(10.0, 10.0, 2.0);


void getBoxesBoundingBox(
    const point3d& position,
    const point3d& bounding_box_size,
    vector<pair<point3d, double> >* box_vector,
    OcTree* octree_) {
  box_vector->clear();
  if (bounding_box_size.x() <= 0.0 || 
        bounding_box_size.y() <= 0.0 || 
        bounding_box_size.z() <= 0.0 
    || octree_->size() == 0) {
    return;
  }
  point3d max_boxes(2.0 * bounding_box_size.x() / octree_->getResolution(),
                            2.0 * bounding_box_size.y() / octree_->getResolution(),
                            2.0 * bounding_box_size.z() / octree_->getResolution());
  long int max_vector_size = ceil(max_boxes.x()) *
                              ceil(max_boxes.y()) *
                              ceil(max_boxes.z());
  box_vector->reserve(max_vector_size);

  const double epsilon = 0.001;  // Small offset to not hit boundary of nodes.
  point3d epsilon_3d(epsilon, epsilon, epsilon);
//   epsilon_3d.setConstant(epsilon);

  point3d bbx_min = position - bounding_box_size + epsilon_3d;
  point3d bbx_max = position + bounding_box_size - epsilon_3d;

//   octomap::point3d bbx_min = pointEigenToOctomap(bbx_min_eigen);
//   octomap::point3d bbx_max = pointEigenToOctomap(bbx_max_eigen);

  for (OcTree::leaf_bbx_iterator
           it = octree_->begin_leafs_bbx(bbx_min, bbx_max),
           end = octree_->end_leafs_bbx();
       it != end; ++it) {
    point3d cube_center(it.getX(), it.getY(), it.getZ());
    int depth_level = it.getDepth();
    double cube_size = octree_->getNodeSize(depth_level);

//     // Check if it is really inside bounding box, since leaf_bbx_iterator begins
//     // "too early"
//     Eigen::Vector3d cube_lower_bound =
//         cube_center - (cube_size / 2) * Eigen::Vector3d::Ones();
//     Eigen::Vector3d cube_upper_bound =
//         cube_center + (cube_size / 2) * Eigen::Vector3d::Ones();
   
    if (octree_->isNodeOccupied(*it)) {
      box_vector->emplace_back(cube_center, cube_size);
    } 
  }
}

void insert(pair< pair<double, double> , double> p, map< pair< pair<double, double> , double> , int>& m){
    m[p]++;
}

void find_corners(point3d p, double d, map< pair< pair<double, double> , double>, int>& m){
    // d*=0.9;
    double dronex = 0.47, droney = 0.47, dronez = 0.23;
    // for(int i=-1; i<=1; i+=2){
    //     for(int j=-1;j<=1;j+=2){
    //         for(int k=-1;k<=1;k+=2){
    //             insert({{(double)(p.x() + i*d/2)*1, (double)(p.y() + j*d/2)*1}, (double)(p.z() + k*d/2)*1}, m);
    //         }
    //     }
    // }
    double factor=2.0;
    int cx, cy, cz;
    double fx, fy, fz;
    for(int i=-1; i<=1; i+=1){
        for(int j=-1;j<=1;j+=1){
            for(int k=-1;k<=1;k+=1){
                if(i==0 && j==0 && k==0)continue;
                cx = (p.x() + i*(d+dronex)/2)*factor;
                fx = cx / factor;
                cy = (p.y() + j*(d+droney)/2)*factor;
                fy = cy / factor;
                cz = (p.z() + k*(d+dronez)/2)*factor;
                fz = cz / factor;
                
                insert({{fx, fy}, fz}, m);
            }
        }
    }
    // insert({{p.x(), p.y()}, p.z()}, m);
}

void print3d(point3d f3){
    ROS_INFO("%f %f %f",f3.x(), f3.y(), f3.z());
    // cout << f3.x() << " " << f3.y() << " " << f3.z() << endl;
}

bool raycast(point3d src, point3d dest, OcTree* octree, bool ignoreUnknownCells=true){
    // Returns true if hit, else false
    if(src.z() < 0.3)return true;
    if(dest.z() < 0.3)return true;
    if(src.z() > 4.7)return true;
    if(dest.z() > 4.7)return true;

    if(src == dest)return false;

    point3d f3;
    OccupancyOcTreeBase<OcTreeNode>* octree_ = (OccupancyOcTreeBase<OcTreeNode>*) octree;
    bool ret = octree_->castRay(src, dest-src, f3, ignoreUnknownCells);
    // double thresh=100.0;
    
    // if(!ret && abs(f3.x()) < thresh && abs(f3.y()) < thresh && abs(f3.z()) < thresh){
    //     return false;
    // }
    // cout << "f3 : ";
    // print3d(f3);
    return ret;
}

double l2_norm(pair< pair<double, double> , double> a, pair< pair<double, double> , double> b){
    // No need of sqrt
    return sqrt(pow(a.first.first - b.first.first, 2) + 
                    pow(a.first.second - b.first.second, 2) +
                    pow(a.second - b.second, 2));
}


pair< pair<double, double> , double> generate_path(int qidx, 
                    int start_idx, 
                    int target_idx, 
                    vector< pair< pair<double, double> , double> >& mp, 
                    int parent[],
                    int numVertices){
    int current = target_idx;
    pair< pair<double, double> , double> x = mp[target_idx];

    
    // for(int i=0;i<numVertices;i+=1){
    //     cout << parent[i] << " ";
    // }

    ROS_INFO("PATH STARTED :");
    // cout << "PATH STARTED :\n";
    
    while(current != start_idx){
        current = parent[current];
        if(current != start_idx){
            x = mp[current];
        }
        cout << mp[current].first.first << " " << mp[current].first.second << " " << mp[current].second << "\n";
    }

    cout << ": PATH ENDED\n";
    return x;

}

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

    double length = 2.0;
    point3d pull;
    if(current.z() <= 3.0){
        point3d pull_(0.0, 0.0, 0.1*pow(3.0 - current.z(), 2));
        pull = pull_;
    }
    else{
        point3d pull_(0.0, 0.0, -0.1*pow(3.0 - current.z(), 2));
        pull = pull_;
    }
    // prev = current;
    point3d f1(orien.x(), orien.y(), 0.0);
    point3d f2(prev.x() - current.x(), prev.y() - current.y(), 0.0);

    
    point3d orien_norm = f1.normalize();

    point3d prev_norm = f2.normalize();
    
    // if(prev_norm.y() + orien_norm.y() )
    point3d new_dir = prev_norm + orien_norm ; //+ pull;
    point3d new_dir_norm = new_dir.normalize();

    new_dir = prec(new_dir_norm * length);

    while(check_occupancy(new_dir + current, octree)){
        length += 0.5;
        new_dir = prec(new_dir_norm * length);
    }
    // point3d new_bbx(length, bbx_size.y(), length);
    // bbx_size = new_bbx;
    cout << length << endl;
    if(new_dir.z() + current.z() >= 0.5 && new_dir.z() + current.z() <= 4.5){
        return new_dir + current;
    }
    if(new_dir.z() + current.z() < 0.5){
        point3d ans(new_dir.x() + current.x(), new_dir.y() + current.y(), 0.5);
        return ans;
    }
    point3d ans(new_dir.x() + current.x(), new_dir.y() + current.y(), 4.5);

    return ans;
}

pair< pair<double, double> , double> Astar(point3d current, point3d dest, vector<pair< pair<double, double> , double> >& mp, OcTree* octree){

    // OccupancyOcTreeBase<OcTreeNode>* octree_ = (OccupancyOcTreeBase<OcTreeNode>*) octree;

    pair< pair<double, double> , double> start{{current.x(), current.y()}, current.z()}, target({{dest.x(), dest.y()}, dest.z()});

    mp.push_back(start);
    mp.push_back(target);
    
    int countTrue = 0;
    int numEdges = 0;
    int numVertices = mp.size();
    vector<int> adj[numVertices];
    
    if(!raycast(current, dest, octree, false)){
        cout << "DIRECT PATH FOUND!\n";
        return target;
    }

    // point3d f11(start.first.first, start.first.second, start.second);
    // point3d f22(target.first.first, target.first.second, target.second);

    // bool ret = raycast(f11, f22, octree);

    // if(!ret){
    //    cout << "ERROR!\n";

    // }


    for(int i=0;i<mp.size()-1;i+=1){
        for(int j=i+1;j<mp.size();j+=1){

            point3d f1(mp[i].first.first, mp[i].first.second, mp[i].second);
            point3d f2(mp[j].first.first, mp[j].first.second, mp[j].second);

            bool ret = raycast(f1, f2, octree, false);
            if(ret){
                countTrue += 1;
            }
            else {
                numEdges+=1;
                adj[i].push_back(j);
                adj[j].push_back(i);
            }

            // print3d(f1);
            // print3d(f2);
            // print3d(f2-f1);
            // print3d(f3);
            // cout << ret << endl;
        }
    }
    // for(auto i:adj[numVertices - 1]){
    //     cout << i << " ";
    // }

    cout << "Size of mp : " << mp.size() << endl;
    ROS_INFO("numEdges : %d",numEdges);
    // cout << "numEdges : " << numEdges << endl;

    double g[numVertices];
    double h[numVertices];
    double f[numVertices];

    for(int i=0;i<numVertices;i+=1){
        // h[i] = l2_norm(target, mp[i]);
        g[i] = 10000000.0;
    }
    
    int start_idx = numVertices - 2;
    int target_idx = numVertices - 1;

    g[start_idx] = 0.0;
    h[start_idx] = 10000000.0;
    f[start_idx] = g[start_idx] + h[start_idx];
    

    priority_queue<pair<double, int>, vector<pair<double, int>>, greater<pair<double, int>>> openpq;

    int openarray[numVertices];
    int closearray[numVertices];
    int parent[numVertices];

    memset(openarray, -1, sizeof(openarray));
    memset(closearray, -1, sizeof(closearray));
    memset(parent, -1, sizeof(parent));


    openpq.push({f[start_idx], start_idx});
    openarray[start_idx] = f[start_idx];

    while(!openpq.empty()){
        pair<double,int> q = openpq.top();
        openpq.pop();
        
        if(closearray[q.second] != -1)continue;

        if(q.second == target_idx){
            pair< pair<double, double> , double> x1 = generate_path(q.second, start_idx, target_idx, mp, parent, numVertices);
            return x1;
        }

        closearray[q.second] = f[q.second];
        openarray[q.second] = -1;

        for(auto el:adj[q.second]){
            
            if(closearray[el] != -1)continue;

            double norm_elqs = l2_norm(mp[el], mp[q.second]);
            if(g[q.second] + norm_elqs < g[el]){
                parent[el] = q.second;
                g[el] = g[q.second] + norm_elqs;
                h[el] = l2_norm(target, mp[el]) + pow(mp[el].second - 3.0, 2);
                f[el] = g[el] + h[el];
                openpq.push({f[el], el});
            }
                       
        }
    }
    ROS_ERROR("NO PATH FOUND");
    // cout << "NO PATH FOUND\n";
    return start;
}

