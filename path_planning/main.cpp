// #include <iostream>

#include <octomap/AbstractOcTree.h>
#include <octomap/OcTree.h>
#include <octomap/OccupancyOcTreeBase.h>
#include <octomap/octomap.h>
#include <bits/stdc++.h>


using namespace std;
using namespace octomap;

#define point pair<pair<double, double>, double>

// void print_query_info(point3d query, OcTreeNode* node) {
//    if (node != NULL) {
//      cout << "occupancy probability at " << query << ":\t " << node->getOccupancy() << endl;
//    }
//    else 
//      cout << "occupancy probability at " << query << ":\t is unknown" << endl;    
//  }

void getBoxesBoundingBox(
    const point3d& position,
    const point3d& bounding_box_size,
    vector<pair<point3d, double>>* box_vector,
    OcTree* octree_) {
  box_vector->clear();
  if (bounding_box_size.x() <= 0.0 || 
        bounding_box_size.y() <= 0.0 || 
        bounding_box_size.z() <= 0.0 
    || octree_->size() == 0) {
    return;
  }
  const point3d max_boxes(2 * bounding_box_size.x() / octree_->getResolution(),
                            2 * bounding_box_size.y() / octree_->getResolution(),
                            2 * bounding_box_size.z() / octree_->getResolution());
  const int max_vector_size = ceil(max_boxes.x()) *
                              ceil(max_boxes.y()) *
                              ceil(max_boxes.z());
  box_vector->reserve(max_vector_size);

  const double epsilon = 0.001;  // Small offset to not hit boundary of nodes.
  point3d epsilon_3d;
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

void insert(point p, map<point, int>& m){
    m[p]++;
}

void find_corners(point3d p, double d, map<point, int>& m){
    // d*=0.9;
    double dronex = 0.47, droney = 0.23, dronez = 0.47;
    // for(int i=-1; i<=1; i+=2){
    //     for(int j=-1;j<=1;j+=2){
    //         for(int k=-1;k<=1;k+=2){
    //             insert({{(double)(p.x() + i*d/2)*1, (double)(p.y() + j*d/2)*1}, (double)(p.z() + k*d/2)*1}, m);
    //         }
    //     }
    // }
    double factor=1.0;
    int cx, cy, cz;
    double fx, fy, fz;
    for(int i=-1; i<=1; i+=1){
        for(int j=-1;j<=1;j+=1){
            for(int k=-1;k<=1;k+=1){
                if(i==0 && j==0 && k==0)continue;
                cx = (p.x() + i*(d+dronex)/2)*factor;
                fx = cx * factor;
                cy = (p.y() + j*(d+droney)/2)*factor;
                fy = cy * factor;
                cz = (p.z() + k*(d+dronez)/2)*factor;
                fz = cz * factor;
                
                insert({{fx, fy}, fz}, m);
            }
        }
    }
    // insert({{p.x(), p.y()}, p.z()}, m);
}

void print3d(point3d f3){
    cout << f3.x() << " " << f3.y() << " " << f3.z() << "\n";
}

double l2_norm(point a, point b){
    return sqrt(pow(a.first.first - b.first.first, 2) + 
                    pow(a.first.second - b.first.second, 2) +
                    pow(a.second - b.second, 2));
}


point generate_path(int qidx, 
                    int start_idx, 
                    int target_idx, 
                    vector<point>& mp, 
                    int parent[]){
    int current = target_idx;
    point x = mp[target_idx];
    
    while(current != start_idx){
        current = parent[current];
        if(current != start_idx){
            x = mp[current];
        }
        cout << mp[current].first.first << " " << mp[current].first.second << " " << mp[current].second << "\n";
    }
    return x;

}

void Astar(point3d current, point3d dest, vector<point>& mp, OcTree* octree){

    OccupancyOcTreeBase<OcTreeNode>* octree_ = (OccupancyOcTreeBase<OcTreeNode>*) octree;

    point start{{current.x(), current.y()}, current.z()}, target({{dest.x(), dest.y()}, dest.z()});

    mp.push_back(start);
    mp.push_back(target);
    
    int countTrue = 0;
    int numEdges = 0;
    int numVertices = mp.size();
    vector<int> adj[numVertices];
    


    for(int i=0;i<mp.size()-1;i+=1){
        for(int j=i+1;j<mp.size();j+=1){

            point3d f1(mp[i].first.first, mp[i].first.second, mp[i].second);
            point3d f2(mp[j].first.first, mp[j].first.second, mp[j].second);
            point3d f3;

            bool ret = octree_->castRay(f1, f2-f1, f3, true);
            if(ret)countTrue += 1;
            else {
                numEdges+=1;
                adj[i].push_back(j);
                adj[j].push_back(i);}

            // print3d(f1);
            // print3d(f2);
            // print3d(f2-f1);
            // print3d(f3);
            // cout << ret << endl;
        }
    }

    cout << "Size of mp : " << mp.size() << endl;
    cout << "CountTrue : " << countTrue << endl;

    double g[numVertices];
    double h[numVertices];
    double f[numVertices];

    for(int i=0;i<numVertices;i+=1){
        // h[i] = l2_norm(target, mp[i]);
        g[i] = 10000000;
    }
    
    int start_idx = numVertices - 2;
    int target_idx = numVertices - 1;

    g[start_idx] = 0;
    f[start_idx] = g[start_idx] + h[start_idx];

    priority_queue<pair<int, int>, vector<pair<int, int>>, greater<pair<int, int>>> openpq;
    // map<int, int> openmp;
    // map<int, int> closemp;
    // map<int, int> parent;
    int openarray[numVertices];
    int closearray[numVertices];
    int parent[numVertices];

    memset(openarray, -1, sizeof(openarray));
    memset(closearray, -1, sizeof(closearray));


    openpq.push({f[start_idx], start_idx});
    openarray[start_idx] = f[start_idx];

    while(!openpq.empty()){
        pair<int,int> q = openpq.top();
        openpq.pop();
        
        if(closearray[q.second] != -1)continue;

        if(q.second == target_idx){
            point x = generate_path(q.second, start_idx, target_idx, mp, parent);
            break;
        }

        closearray[q.second] = f[q.second];
        openarray[q.second] = -1;

        for(auto el:adj[q.second]){
            
            if(closearray[el] != -1)continue;

            double norm_elqs = l2_norm(mp[el], mp[q.second]);
            if(g[q.second] + norm_elqs < g[el]){
                parent[el] = q.second;
                g[el] = g[q.second] + norm_elqs;
                h[el] = l2_norm(target, mp[el]);
                f[el] = g[el] + h[el];
                openpq.push({f[el], el});
            }
                        

            // h[el] = l2_norm(target, mp[el]);
            // f[el] = g[el] + h[el];

            // if(openarray[el] < f[el])continue;

            // if(closearray[el] < f[el])continue;

            // parent[el] = q.second;
            // openpq.push({f[el], el});
            // openarray[el] = f[el];
        }
        // closearray[q.second] = f[q.second];
    }
}

int main() {
	AbstractOcTree* tree = AbstractOcTree::read("/home/hexplex0xdd/octomap.ot");
    cout << "Tree size : " << tree->size() << endl;
	// OcTree* octree = dynamic_cast<OcTree*>(tree);
    OcTree* octree = (OcTree*)tree;


    point3d current(10.0, 1.0, 0.0);
    point3d bbx_size(20.0, 1.0, 20.0);
    point3d dest(0.0, 1.0, 7.0);

    vector<pair<point3d, double>> box_vector;

    getBoxesBoundingBox(current, bbx_size, &box_vector, octree);

    cout << "Size of box_vector : " << box_vector.size() << endl;

    map<point, int> m;
    for(int i=0;i<box_vector.size();i++){
        find_corners(box_vector[i].first, box_vector[i].second, m);
    } 
    cout << "Size of map : " << m.size() << endl;

    int count = 0;
    for(auto i:m){
        if(i.second == 1)count +=1;
    } 
    cout << "Count : " << count << endl;

    vector<point> mp;
    for(auto i:m){
        // if(i.second == 1)
        mp.push_back(i.first);
    }



    Astar(current, dest, mp, octree);

	return 0;
}

    // OccupancyOcTreeBase<OcTreeNode>* octree_ = (OccupancyOcTreeBase<OcTreeNode>*) octree;


    // point3d f4;

    // point3d current1(0.0, 1.0, 0.0);
    // point3d dest1(0.0, 1.0, 7.0);
    // bool ans = octree_->castRay(current1, dest1-current1, f4, true);
    // print3d(f4);
    // cout << "dfgdfg : " << ans << endl;

    // point3d current2(0.0, 0.0, 1.0);
    // point3d dest2(0.0, 7.0, 1.0);
    // ans = octree_->castRay(current2, dest2-current2, f4, true);
    // print3d(f4);
    // cout << "dfgdfg : " << ans << endl;

    // point3d current3(1.0, 0.0, 0.0);
    // point3d dest3(1.0, 0.0, 7.0);
    // ans = octree_->castRay(current3, dest3-current3, f4, true);
    // print3d(f4);
    // cout << "dfgdfg : " << ans << endl;

    // point3d current4(1.0, 0.0, 0.0);
    // point3d dest4(1.0, 7.0, 0.0);
    // ans = octree_->castRay(current4, dest4-current4, f4, true);
    // print3d(f4);
    // cout << "dfgdfg : " << ans << endl;

    // point3d current5(0.0, 0.0, 1.0);
    // point3d dest5(7.0, 0.0, 1.0);
    // ans = octree_->castRay(current5, dest5-current5, f4, true);
    // print3d(f4);
    // cout << "dfgdfg : " << ans << endl;

    // point3d current6(0.0, 1.0, 0.0);
    // point3d dest6(7.0, 1.0, 0.0);
    // ans = octree_->castRay(current6, dest6-current6, f4, true);
    // print3d(f4);
    // cout << "dfgdfg : " << ans << endl;




    
    // point3d f1(mp[0].first.first, mp[0].first.second, mp[0].second);
    // point3d f2(mp[1].first.first, mp[1].first.second, mp[1].second);
    // point3d f3;

    // point3d f4;
    // for(double i=-2.0;i<=5.0;i+=1.0){
    //     for(double j=-2.0;j<=5.0;j+=1.0){
    //         point3d current1(i, i, 5.0);
    //         point3d dest1(j, j, 5.0);
    //         bool ans = octree_->castRay(current1, dest1-current1, f4, false);
    //         cout << "dfgdfg : " << ans << endl;
    //     }
    // }

    // cout << octree_->castRay(f1, f2-f1, f3, true) << endl;

    // print3d(f1);
    // print3d(f2);
    // print3d(f2-f1);
    // print3d(f3);

    



    // if(octree == NULL)cout << "GREAT!\n";

    // point3d q (0., 0., 0.);
    // int i=0;
    // for(auto& el:*octree){
    //     if(i>100)break;
    //     cout << el.getValue();
    //     i+=1;
    // }
    // OcTreeNode* result = octree->

    // OcTree* root = (OcTree*)(octree->getRoot());
    

    // print_query_info(q, result);
    // cout << result->getValue();


    // for(int i=0;i<10;i+=1){
        // OcTreeNode* temp;
        // octree->getNodeChild(temp, 0);
        // octree = (OcTree*) temp;

    // }
    // cout << c->getValue();

    // for(OcTree::tree_iterator it=octree->begin_tree(), 
    //         end=octree->end_tree(); it!=end; ++it){
    //         cout << "Node center: " << it.getCoordinate() << endl;
    //         cout << "Node size: " << it.getSize() << endl;
    //         cout << "Node value: " << it->getValue() << endl;
    //         }

    // cout << "Working\n";