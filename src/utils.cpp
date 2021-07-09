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

point3d bbx_size(6.0, 6.0, 3.5);
point3d accum(0.0, 0.0, 0.0);
queue<double> qu;
int qusize=0;
double total_err=0.0;

/* Calculates error angle between two orientations. */
double calc_error(point3d cur_orien,point3d prev_orien){
	point3d cur_orien1(cur_orien.x(),cur_orien.y(),0.0);
	point3d prev_orien1(prev_orien.x(),prev_orien.y(),0.0);
	cur_orien1=cur_orien1.normalize();
	prev_orien1 = prev_orien1.normalize();
	double val=cur_orien.cross(prev_orien).z();
	return asin(val);
}

/* Returns a vector given an angle and the length of vector.*/
point3d vec(double angle, double val){
	return point3d(val*cos(angle),val*sin(angle),0.0);
}

/* Returns a vector of points which are occupied around a point(position)
 * with in a bounding box (with sizes bounding_box_size) */
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
                            5.0 / octree_->getResolution());
  long int max_vector_size = ceil(max_boxes.x()) *
                              ceil(max_boxes.y()) *
                              ceil(max_boxes.z());
  box_vector->reserve(max_vector_size);

  const double epsilon = 0.001;  // Small offset to not hit boundary of nodes.
  point3d epsilon_3d(epsilon, epsilon, epsilon);
//   epsilon_3d.setConstant(epsilon);
  point3d bbx_min = point3d(position.x()-bbx_size1.x(),position.y()-bbx_size1.y(),0.0)+epsilon_3d;
  //point3d bbx_min = position.x() - bounding_box_size.x() + epsilon_3d;
  point3d bbx_max = point3d(position.x()+bounding_box_size.x(),position.y()+bounding_box_size.y(),4.0) - epsilon_3d;
  //point3d bbx_max = position + bounding_box_size - epsilon_3d;

//   octomap::point3d bbx_min = pointEigenToOctomap(bbx_min_eigen);
//   octomap::point3d bbx_max = pointEigenToOctomap(bbx_max_eigen);

  for (OcTree::leaf_bbx_iterator
           it = octree_->begin_leafs_bbx(bbx_min, bbx_max),
           end = octree_->end_leafs_bbx();
       it != end; ++it) {
    point3d cube_center(it.getX(), it.getY(), it.getZ());
    int depth_level = it.getDepth();
    double cube_size = octree_->getNodeSize(depth_level);
    if (octree_->isNodeOccupied(*it)) {
      box_vector->emplace_back(cube_center, cube_size);
    } 
  }
}

/* Increases the count of a given point(p) in a given map(m).*/
void insert(pair< pair<double, double> , double> p, map< pair< pair<double, double> , double> , int>& m){
    m[p]++;
}

/* Finds and inserts all the corners of a given point(p) into a map(m).*/
void find_corners(point3d p, double d, map< pair< pair<double, double> , double>, int>& m){
    // d*=0.9;
    double dronex = 0.6, droney = 0.6, dronez = 0.5;
    // for(int i=-1; i<=1; i+=2){
    //     for(int j=-1;j<=1;j+=2){
    //         for(int k=-1;k<=1;k+=2){
    //             insert({{(double)(p.x() + i*d/2)*1, (double)(p.y() + j*d/2)*1}, (double)(p.z() + k*d/2)*1}, m);
    //         }
    //     }
    // }
    double factor=5;
    int cx, cy, cz;
    double fx, fy, fz;
    for(int i=-1; i<=1; i+=2){
        for(int j=-1;j<=1;j+=2){
            for(int k=-1;k<=1;k+=2){
                if(i==0 && j==0 && k==0)continue;
                cx = (p.x() + i*(d+dronex)/2)*factor;
                fx = cx / factor;
                cy = (p.y() + j*(d+droney)/2)*factor;
                fy = cy / factor;
                cz = (p.z() + k*(d+dronez)/2)*factor;
                fz = cz / factor;
                if(fz<=5.0 && fz>=0.5){
                    insert({{fx, fy}, fz}, m);
                }
            }
        }
    }
}

void print3d(point3d f3){
    ROS_INFO("%f %f %f",f3.x(), f3.y(), f3.z());
    // cout << f3.x() << " " << f3.y() << " " << f3.z() << endl;
}

/* Returns euclidean distance between given points (a, b).*/

double l2_norm(point3d a, point3d b = point3d(0.0, 0.0, 0.0)){
    // No need of sqrt
    return sqrt(pow(a.x()- b.x(), 2) + 
                    pow(a.y() - b.y(), 2) +
                    pow(a.z() - b.z(), 2));
}


double calc_ang(point3d a,point3d b){
    a=a.normalize();
    b=b.normalize();
    double val = a.dot(b);
    return acos(val);
}

/** Casts a ray from a point(src) to another point(dest) in a given octree, 
 * and returns true if hit, else returns false. */
bool raycast(point3d src, point3d dest, OcTree* octree, bool ignoreUnknownCells=true){
    // Returns true if hit, else false
    if(src.z() <= 0.3)return true;
    if(dest.z() <= 0.3)return true;
    if(src.z() >= 4.7)return true;
    if(dest.z() >= 4.7)return true;

    if(src == dest)return false;

    point3d f3;
    OccupancyOcTreeBase<OcTreeNode>* octree_ = (OccupancyOcTreeBase<OcTreeNode>*) octree;
    bool ret = octree_->castRay(src, dest-src, f3, ignoreUnknownCells, l2_norm(src,dest)+0.3);
    // double thresh=100.0;
    
    // if(!ret && abs(f3.x()) < thresh && abs(f3.y()) < thresh && abs(f3.z()) < thresh){
    //     return false;
    // }
    // cout << "f3 : ";
    // print3d(f3);
    return ret;
}

/* Returns euclidean distance between given points (a, b).*/
double l2_norm(pair< pair<double, double> , double> a, pair< pair<double, double> , double> b){
    // No need of sqrt
    return sqrt(pow(a.first.first - b.first.first, 2) + 
                    pow(a.first.second - b.first.second, 2) +
                    pow(a.second - b.second, 2));
}

/** Performs parent tracing from target to start using parent[] generated by A* algorithm and
 * return the point just after start in the path obtained.
*/
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

/* Takes in a point and returns a point which is rounded off up to certain decimal points.*/
point3d prec(point3d node){
    double factor = 100.0;
    int x = node.x() * factor; 
    int y = node.y() * factor; 
    int z = node.z() * factor; 
    point3d prec_node(x / factor, y / factor, z / factor);
    return prec_node;
}

/* Takes in a point(g) and returns whether the given point is occupied or not with respect to given octree. */
bool check_occupancy(point3d g, OcTree* octree, double s){
    point3d size(s, s, 0.2);
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

/* Decides next setpoint based on current position, previous setpoint and current orientation with respect to given octree. */
point3d decide(point3d current, point3d prev, point3d orien, OcTree* octree){
    
    double length = 5.0;
    point3d pull;
    if(current.z() <= 2.5){
        point3d pull_(0.0, 0.0, 1.5*pow(2.5 - current.z(), 2));
        pull = pull_;
    }
    else{
        point3d pull_(0.0, 0.0, -1.5*pow(2.5 - current.z(), 2));
        pull = pull_;
    }
    // prev = current;
    point3d f1(orien.x(), orien.y(), 0.0);
    point3d f2(prev.x() - current.x(), prev.y() - current.y(), 0.0);
    point3d f3(loc.x()-current.x(), loc.y() - current.y(), 0.0);
    
    point3d orien_norm = f1.normalize();
    if(l2_norm(f3)>0.7){
        prev_norm = f2; //.normalize();
        prev_norm *= 3.0; //point3d(1.1*prev_norm.x(),
    }
    // if(prev_norm.y() + orien_norm.y() )
    point3d new_dir =  prev_norm + orien_norm; //  + point3d(-1,0,0); // + accum.normalize();
    //point3d new_dir = orien_norm + pull - vec(total_err,0.8); 
    point3d new_dir_norm = new_dir.normalize();

    new_dir = prec(new_dir_norm * length);

    while(check_occupancy(new_dir + current, octree, 0.5)){
        length += 0.5;
        new_dir = prec(new_dir_norm * length);
    }
    // point3d new_bbx(length, bbx_size.y(), length);
    // bbx_size = new_bbx;

    cout << length << endl;
    if(new_dir.z() + current.z() >= 1.5 && new_dir.z() + current.z() <= 3.5){
        return new_dir + current;
    //    point3d ans(new_dir.x() + current.x(), new_dir.y() + current.y(), 3.0);
    //    return ans;

    }
    if(new_dir.z() + current.z() < 2.5){
        point3d ans(new_dir.x() + current.x(), new_dir.y() + current.y(), 2.0);
        return ans;
    }
    point3d ans(new_dir.x() + current.x(), new_dir.y() + current.y(), 3.0);
    
    return ans;
}

/** Creates adjacency matrix with the help of raycast function and performs path planning from 
 * current position to destination using A* algorithm. */
pair< pair<double, double> , double> Astar(point3d current, point3d dest, vector<pair< pair<double, double> , double> >& mp, OcTree* octree,point3d orien){

    // OccupancyOcTreeBase<OcTreeNode>* octree_ = (OccupancyOcTreeBase<OcTreeNode>*) octree;

    pair< pair<double, double> , double> start{{current.x(), current.y()}, current.z()}, target({{dest.x(), dest.y()}, dest.z()});

    mp.push_back(start);
    mp.push_back(target);
    
    int countTrue = 0;
    int numEdges = 0;
    int numVertices = mp.size();
    vector<int> adj[numVertices];
    
    if(!raycast(current, dest, octree)){
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
            if(i==(mp.size()-2) && j==(mp.size()-1))continue;
            point3d f1(mp[i].first.first, mp[i].first.second, mp[i].second);
            point3d f2(mp[j].first.first, mp[j].first.second, mp[j].second);
            bool ret;
            ret = raycast(f1, f2, octree);
           
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

     for(int i=0;i<mp.size();i+=1){
        if(!adj[i].empty()) continue;
        for(int j=0;j<mp.size();j+=1){
            if(i==j) continue;
            if(i==(mp.size()-2) && j==(mp.size()-1)) continue;
            if(j==mp.size()-2) continue;
            point3d f1(mp[i].first.first, mp[i].first.second, mp[i].second);
            point3d f2(mp[j].first.first, mp[j].first.second, mp[j].second);
            bool ret;
            ret = raycast(f1, f2, octree,false);
           
            if(ret){
                countTrue += 1;
            }
            else {
                numEdges+=1;
                adj[i].push_back(j);
                adj[j].push_back(i);
            }
        }
    }

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
                point3d var(mp[start_idx].first.first-mp[el].first.first,mp[start_idx].first.second-mp[el].first.second,mp[start_idx].second-mp[el].second);
                double hit = 15.0*pow(mp[el].second - 2.5, 2);
                if (mp[el].second>=1.5 || mp[el].second<=3.5) hit = hit/3.0; 
                long long int r=0;
                for(int it=0;it<(repel.size()-5);it++)
                {
                    int k=5;
                    int d= l2_norm(repel.at(it),mp[el]);
                    if(d==0) r+=1000000;
                    else r+= k/(d*d);
                }
                h[el] = 2.0*l2_norm(target, mp[el]) + hit + r + (mp[start_idx].first.second-mp[el].first.second)*0.5; // + (mp[el].first.first-mp[start_idx].first.first)*2.0;
                /*note change y back to 1.0*/
                f[el] = g[el] + h[el];
                //-20.0*calc_ang(var,orien)
                openpq.push({f[el], el});
            }
                       
        }
    }
    ROS_ERROR("NO PATH FOUND");
    // cout << "NO PATH FOUND\n";
    return target;
}

