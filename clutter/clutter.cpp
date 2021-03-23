// point3d calc_dest(point3d current){
//     point3d dest(current.x()+5.0, current.y(), current.z());
//     return dest;
// }

// int main() {

//     while(1){
        
//         octomap_msgs/Octomap msg;
//         //AbstractOcTree* tree= msgToMap( msg);
//         AbstractOcTree* tree= fullMsgToMap( msg);


//         // AbstractOcTree* tree = AbstractOcTree::read("/home/vishwas/Downloads/octomap.ot");
//         // cout << "Tree size : " << tree->size() << endl;
//         // // OcTree* octree = dynamic_cast<OcTree*>(tree);
//         OcTree* octree = (OcTree*)tree;


//         point3d current(10.0, 1.0, 0.0);
//         point3d bbx_size(5.0, 1.5, 5.0);
//         point3d dest = calc_dest(current);

//         vector<pair<point3d, double>> box_vector;

//         getBoxesBoundingBox(current, bbx_size, &box_vector, octree);

//         cout << "Size of box_vector : " << box_vector.size() << endl;

//         map<point, int> m;
//         for(int i=0;i<box_vector.size();i++){
//             find_corners(box_vector[i].first, box_vector[i].second, m);
//         } 
//         cout << "Size of map : " << m.size() << endl;

//         int count = 0;
//         for(auto i:m){
//             if(i.second == 1)count +=1;
//         } 
//         cout << "Count : " << count << endl;

//         vector<point> mp;
//         for(auto i:m){
//             // if(i.second == 1)
//             mp.push_back(i.first);
//         }



//         Astar(current, dest, mp, octree);

//         usleep(1000);
//     }
// 	return 0;
// }

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

// void print_query_info(point3d query, OcTreeNode* node) {
//    if (node != NULL) {
//      cout << "occupancy probability at " << query << ":\t " << node->getOccupancy() << endl;
//    }
//    else 
//      cout << "occupancy probability at " << query << ":\t is unknown" << endl;    
//  }