#include "mercator.h"
#include <cmath> // Include cmath for mathematical constants and functions

// Remove namespace comment, ensure namespace usage if required in your project setup.

void coreImportTest() {
    std::cout << "scancontext lib is successfully imported." << std::endl;
} // coreImportTest

inline float rad2deg(float radians) {
    return radians * (180.0f / M_PI);
}

inline float deg2rad(float degrees) {
    return degrees * (M_PI / 180.0f);
}

// float xy2theta(const float& x, const float& y) {
//     return std::atan2(y, x) * (180.0f / M_PI); // Simplified using atan2
// }

float xy2theta( const float & _x, const float & _y )
{
    if ( _x >= 0 & _y >= 0) 
        return (180/M_PI) * atan(_y / _x);

    if ( _x < 0 & _y >= 0) 
        return 180 - ( (180/M_PI) * atan(_y / (-_x)) );

    if ( _x < 0 & _y < 0) 
        return 180 + ( (180/M_PI) * atan(_y / _x) );

    if ( _x >= 0 & _y < 0)
        return 360 - ( (180/M_PI) * atan((-_y) / _x) );
} // xy2theta    rz2theta


MatrixXd circshift(const MatrixXd& mat, int numShift) {
    if (numShift == 0) return mat;

    MatrixXd shiftedMat = MatrixXd::Zero(mat.rows(), mat.cols());
    int cols = mat.cols();
    for (int i = 0; i < cols; ++i) {
        shiftedMat.col((i + numShift) % cols) = mat.col(i);
    }
    return shiftedMat;
}

std::vector<float> eig2stdvec( MatrixXd& eigmat) {
    return {eigmat.data(), eigmat.data() + eigmat.size()};
}

double MCManager::distDirectSC(const MatrixXd& sc1, const MatrixXd& sc2) {
    double sumSim = 0.0;
    int count = 0;
    for (int i = 0; i < sc1.cols(); ++i) {
        auto norm1 = sc1.col(i).norm(), norm2 = sc2.col(i).norm();
        if (norm1 == 0 || norm2 == 0) continue;
        sumSim += sc1.col(i).dot(sc2.col(i)) / (norm1 * norm2);
        ++count;
    }
    return count == 0 ? 0.0 : 1.0 - (sumSim / count);
}

int MCManager::fastAlignUsingVkey(const MatrixXd& vkey1, const MatrixXd& vkey2) {
    double minNorm = std::numeric_limits<double>::max();
    int argminShift = 0;
    for (int i = 0; i < vkey1.cols(); ++i) {
        auto shifted = circshift(vkey2, i);
        double norm = (vkey1 - shifted).norm();
        if (norm < minNorm) {
            minNorm = norm;
            argminShift = i;
        }
    }
    return argminShift;
}

std::pair<double, int> MCManager::distanceBtnScanContext(const MatrixXd& sc1, const MatrixXd& sc2) {
    auto vkey1 = makeSectorkeyFromScancontext(sc1);
    auto vkey2 = makeSectorkeyFromScancontext(sc2);
    int bestShift = fastAlignUsingVkey(vkey1, vkey2);
    std::vector<int> searchSpace = {bestShift};
    const int searchRadius = std::round(0.5 * SEARCH_RATIO * sc1.cols());
    for (int i = 1; i <= searchRadius; ++i) {
        searchSpace.push_back((bestShift + i+sc1.cols()) % sc1.cols());
        searchSpace.push_back((bestShift - i + sc1.cols()) % sc1.cols());
    }
    std::sort(searchSpace.begin(), searchSpace.end());

    double minDist = std::numeric_limits<double>::max();
    int optimalShift = 0;
    for (int shift : searchSpace) {
        MatrixXd shiftedSc2 = circshift(sc2, shift);
        double currentDist = distDirectSC(sc1, shiftedSc2);
        if (currentDist < minDist) {
            minDist = currentDist;
            optimalShift = shift;
        }
    }
    return {minDist, optimalShift};
}

// MatrixXd MCManager::makeScancontext(pcl::PointCloud<SCPointType>& scan) {
//     const int NO_POINT = -1000;
//     MatrixXd desc = MatrixXd::Constant(PC_NUM_RING, PC_NUM_SECTOR, NO_POINT);
//     for (const auto& pt : scan) {
//         float range = std::hypot(pt.x, pt.y);
//         if (range > PC_MAX_RADIUS || range < 2) continue;

//         // azim_range = sqrt(pt.x * pt.x + pt.y * pt.y);
//         // axiy_angle = xy2theta(pt.x, pt.y);
//         float angle = xy2theta(pt.x, pt.y);
//         float angle_z=xy2theta(range,(pt.z+LIDAR_HEIGHT));

//         int sectorIndex = std::clamp(int((angle / 360.0) * PC_NUM_SECTOR), 0, PC_NUM_SECTOR - 1);
//         int ringIndex = std::clamp(int((angle_z / 45.0) * PC_NUM_RING), 0, PC_NUM_RING - 1);
//         // std::max( std::min( PC_NUM_RING, int(ceil( (ariz_angle / 45.0) * PC_NUM_RING )) ), 1 );

//         if (pt.z > desc(ringIndex, sectorIndex)) {
//             desc(ringIndex, sectorIndex) = pt.z;
//         }
//     }
//     desc = (desc.array() == NO_POINT).select(0, desc);
//     return desc;
// }



// //wzb   ·莫卡托描述子  best
// MatrixXd MCManager::makeScancontext( pcl::PointCloud<SCPointType> & _scan_down )
// {
//     TicToc t_making_desc;

//     int num_pts_scan_down = _scan_down.points.size();

//     // main
//     const int NO_POINT = -1000;
//     MatrixXd desc = NO_POINT * MatrixXd::Ones(PC_NUM_RING, PC_NUM_SECTOR);

//     SCPointType pt;
//     float axiy_angle,ariz_angle,azim_range; // wihtin 2d plane
//     int ring_idx, sctor_idx,higtor_idx;
//     for (int pt_idx = 0; pt_idx < num_pts_scan_down; pt_idx++)
//     {
//         pt.x = _scan_down.points[pt_idx].x; 
//         pt.y = _scan_down.points[pt_idx].y;
//         pt.z = _scan_down.points[pt_idx].z + LIDAR_HEIGHT; // naive adding is ok (all points should be > 0).
//         pt.intensity=_scan_down.points[pt_idx].intensity;
//         // xyz to ring, sector
//         azim_range = sqrt(pt.x * pt.x + pt.y * pt.y);
//         axiy_angle = xy2theta(pt.x, pt.y);
//         ariz_angle=xy2theta(azim_range,pt.z);

//         // if range is out of roi, pass
//         if( azim_range > PC_MAX_RADIUS )
//             continue;

//         if( azim_range < 2 )
//             continue;

//         // ring_idx = std::max( std::min( PC_NUM_RING, int(ceil( (azim_range / PC_MAX_RADIUS) * PC_NUM_RING )) ), 1 );

//         sctor_idx = std::max( std::min( PC_NUM_SECTOR, int(ceil( (axiy_angle / 360.0) * PC_NUM_SECTOR )) ), 1 );
//         higtor_idx =std::max( std::min( PC_NUM_RING, int(ceil( (ariz_angle / 45.0) * PC_NUM_RING )) ), 1 );
//         // taking maximum z 
//         if ( desc(higtor_idx-1, sctor_idx-1) < pt.z ) // -1 means cpp starts from 0
//             desc(higtor_idx-1, sctor_idx-1) = pt.z; // update for taking maximum value at that bin
//     }
    

//     // reset no points to zero (for cosine dist later)
//     for ( int row_idx = 0; row_idx < desc.rows(); row_idx++ )
//         for ( int col_idx = 0; col_idx < desc.cols(); col_idx++ )
//             if( desc(row_idx, col_idx) == NO_POINT )
//                 desc(row_idx, col_idx) = 0;

//     t_making_desc.toc("PolarContext making");

//     return desc;
// } // SCManager::makeScancontext    pt.y


MatrixXd MCManager::makeScancontext(pcl::PointCloud<SCPointType>& scan) {
    // 使用两个矩阵，一个用于累积深度值，另一个用于计数
    MatrixXd descSum = MatrixXd::Zero(PC_NUM_RING, PC_NUM_SECTOR);  // 存储深度总和
    MatrixXi count = MatrixXi::Zero(PC_NUM_RING, PC_NUM_SECTOR);    // 存储每个区域的点数

    for (const auto& pt : scan) {
        float range = std::hypot(pt.x, pt.y);
        if (range > PC_MAX_RADIUS || range < 2) continue;

        float angle = xy2theta(pt.x, pt.y);
        float angle_z = xy2theta(range, (pt.z+LIDAR_HEIGHT));

        int sectorIndex = std::clamp(int((angle / 360.0) * PC_NUM_SECTOR), 0, PC_NUM_SECTOR - 1);
        int ringIndex = std::clamp(int((angle_z / 45.0) * PC_NUM_RING), 0, PC_NUM_RING - 1);

        // 累加当前点的深度值到对应区域，并增加计数
        descSum(ringIndex, sectorIndex) += pt.z;
        count(ringIndex, sectorIndex) += 1;
    }

    // 计算平均深度值
    MatrixXd desc = MatrixXd::Zero(PC_NUM_RING, PC_NUM_SECTOR);
    for (int i = 0; i < PC_NUM_RING; ++i) {
        for (int j = 0; j < PC_NUM_SECTOR; ++j) {
            if (count(i, j) > 0) {
                desc(i, j) = descSum(i, j) / count(i, j);
            } else {
                desc(i, j) = 0; // 如果没有点落在某个区域，可以设为NO_POINT或者其他表示无效的值
            }
        }
    }

    return desc;
}


MatrixXd MCManager::makeRingkeyFromScancontext(const MatrixXd& desc) {
    return desc.colwise().mean();
}

MatrixXd MCManager::makeSectorkeyFromScancontext(const MatrixXd& desc) {
    return desc.rowwise().mean();
}

void MCManager::makeAndSaveScancontextAndKeys(pcl::PointCloud<SCPointType>& scan) {
    MatrixXd sc = makeScancontext(scan);
    MatrixXd ringKey = makeRingkeyFromScancontext(sc);
    MatrixXd sectorKey = makeSectorkeyFromScancontext(sc);
    std::vector<float> invKeyVec = eig2stdvec(ringKey);

    polarcontexts_.push_back(sc);
    polarcontext_invkeys_.push_back(ringKey);
    polarcontext_vkeys_.push_back(sectorKey);
    polarcontext_invkeys_mat_.push_back(invKeyVec);
}
//row
// std::pair<int, float> MCManager::detectLoopClosureID() {
//     if (polarcontext_invkeys_mat_.size() <= NUM_EXCLUDE_RECENT) {
//         return {-1, 0.0};  // No loop found
//     }

//     auto& currKey = polarcontext_invkeys_mat_.back();
//     auto& currDesc = polarcontexts_.back();
//     double minDist = std::numeric_limits<double>::max();
//     int bestMatch = -1, bestShift = 0;

//     // Tree search simulation for the sake of example
//     for (size_t i = 0; i < polarcontext_invkeys_mat_.size() - NUM_EXCLUDE_RECENT; ++i) {
//         double dist = distDirectSC(currDesc, polarcontexts_[i]);
//         if (dist < minDist) {
//             minDist = dist;
//             bestMatch = i;
//         }
//     }

//     if (minDist < MC_DIST_THRES) {
//         bestShift = fastAlignUsingVkey(currDesc, polarcontexts_[bestMatch]);
//         return {bestMatch, bestShift};
//     }
//     return {-1, 0.0};  // No loop found
// }


std::pair<int, float> MCManager::detectLoopClosureID() {
    int loop_id = -1;
    float min_dist = std::numeric_limits<float>::max();
    int nn_align = 0;

    if (polarcontext_invkeys_mat_.size() <= NUM_EXCLUDE_RECENT) {
        return {-1, 0.0};  // No loop found
    }

    auto& currKey = polarcontext_invkeys_mat_.back();
    auto& currDesc = polarcontexts_.back();
    double minDist = std::numeric_limits<double>::max();
    int bestMatch = -1, bestShift = 0;

    // Tree search simulation for the sake of example
    for (size_t i = 0; i < polarcontext_invkeys_mat_.size() - NUM_EXCLUDE_RECENT; ++i) {
        double dist = distDirectSC(currDesc, polarcontexts_[i]);
        // float dist = distDirectSC(sc, polarcontexts[i]);
        if (dist < min_dist) {
            min_dist = dist;
            loop_id = i;
            nn_align = fastAlignUsingVkey(currDesc, polarcontexts_[i]);

            // 早期退出机制：如果找到足够接近的回环，提前退出
            if (min_dist < MC_DIST_THRES) {
                break;
            }
        }
    }

    // 输出日志信息，方便调试
    if (min_dist < MC_DIST_THRES) {
        std::cout << "[Loop found] Nearest distance: " << min_dist << " between current frame and frame " << loop_id << "." << std::endl;
        std::cout << "[Loop found] yaw difference: " << nn_align * PC_UNIT_SECTORANGLE << " degrees." << std::endl;
        return std::make_pair(loop_id, 1 - min_dist);

    } else {
        std::cout.precision(3);
        std::cout << "[Not loop] Nearest distance: " << min_dist << " between current frame and frame " << loop_id << "." << std::endl;
        std::cout << "[Not loop] yaw difference: " << nn_align * PC_UNIT_SECTORANGLE << " degrees." << std::endl;
        return {-1, 0.0};  // No loop found
    
    }

    // 返回回环帧ID和相似度（相似度为1-距离）
    // return std::make_pair(loop_id, 1 - min_dist);
}




// //wangzika
// std::pair<int, float> MCManager::detectLoopClosureID1 ( std::ofstream &file,const std::string &lidar_path,size_t cloudInd)
// {
//     int loop_id { -1 }; // init with -1, -1 means no loop (== LeGO-LOAM's variable "closestHistoryFrameID")

//     auto curr_key = polarcontext_invkeys_mat_.back(); // current observation (query)
//     auto curr_desc = polarcontexts_.back(); // current observation (query)

//     /* 
//      * step 1: candidates from ringkey tree_
//      */
//     if( polarcontext_invkeys_mat_.size() < NUM_EXCLUDE_RECENT + 1)
//     {
//         std::pair<int, float> result {loop_id, 0.0};
//         return result; // Early return 
//     }

//     // tree_ reconstruction (not mandatory to make everytime)
//     if( tree_making_period_conter % TREE_MAKING_PERIOD_ == 0) // to save computation cost
//     {
//         TicToc t_tree_construction;

//         polarcontext_invkeys_to_search_.clear();
//         polarcontext_invkeys_to_search_.assign( polarcontext_invkeys_mat_.begin(), polarcontext_invkeys_mat_.end() - NUM_EXCLUDE_RECENT ) ;

//         polarcontext_tree_.reset(); 
//         polarcontext_tree_ = std::make_unique<InvKeyTree>(PC_NUM_RING /* dim */, polarcontext_invkeys_to_search_, 10 /* max leaf */ );
//         // tree_ptr_->index->buildIndex(); // inernally called in the constructor of InvKeyTree (for detail, refer the nanoflann and KDtreeVectorOfVectorsAdaptor)
//         t_tree_construction.toc("Tree construction");
//     }
//     tree_making_period_conter = tree_making_period_conter + 1;
        
//     double min_dist = 10000000; // init with somthing large
//     int nn_align = 0;
//     int nn_idx = 0;

//     // knn search
//     std::vector<size_t> candidate_indexes( NUM_CANDIDATES_FROM_TREE ); 
//     std::vector<float> out_dists_sqr( NUM_CANDIDATES_FROM_TREE );

//     TicToc t_tree_search;
//     nanoflann::KNNResultSet<float> knnsearch_result( NUM_CANDIDATES_FROM_TREE );
//     knnsearch_result.init( &candidate_indexes[0], &out_dists_sqr[0] );
//     polarcontext_tree_->index->findNeighbors( knnsearch_result, &curr_key[0] /* query */, nanoflann::SearchParams(10) ); 
//     t_tree_search.toc("Tree search");

//     /* 
//      *  step 2: pairwise distance (find optimal columnwise best-fit using cosine distance)
//      */
//     TicToc t_calc_dist;   
//     for ( int candidate_iter_idx = 0; candidate_iter_idx < NUM_CANDIDATES_FROM_TREE; candidate_iter_idx++ )
//     {
//         MatrixXd polarcontext_candidate = polarcontexts_[ candidate_indexes[candidate_iter_idx] ];
//         std::pair<double, int> sc_dist_result = distanceBtnScanContext( curr_desc, polarcontext_candidate ); 
        
//         double candidate_dist = sc_dist_result.first;
//         int candidate_align = sc_dist_result.second;

//         if( candidate_dist < min_dist )
//         {
//             min_dist = candidate_dist;
//             nn_align = candidate_align;

//             nn_idx = candidate_indexes[candidate_iter_idx];
//         }
//     }
//     t_calc_dist.toc("Distance calc");

//     /* 
//      * loop threshold check
//      */
//     if( min_dist < MC_DIST_THRES )
//     {
//         loop_id = nn_idx; 
    
//         // std::cout.precision(3); 
//         cout << "[Loop found] Nearest distance: " << min_dist << " btn " << polarcontexts_.size()-1 << " and " << nn_idx << "." << endl;
//         cout << "[Loop found] yaw diff: " << nn_align * PC_UNIT_SECTORANGLE << " deg." << endl;
//     }
//     else
//     {
//         std::cout.precision(3); 
//         cout << "[Not loop] Nearest distance: " << min_dist << " btn " << polarcontexts_.size()-1 << " and " << nn_idx << "." << endl;
//         cout << "[Not loop] yaw diff: " << nn_align * PC_UNIT_SECTORANGLE << " deg." << endl;
//     }

//     // To do: return also nn_align (i.e., yaw diff)
//     float yaw_diff_rad = deg2rad(nn_align * PC_UNIT_SECTORANGLE);
//     std::pair<int, float> result {loop_id, 1-min_dist};

//     if (min_dist < MC_DIST_THRES) {
//     std::cout << "[Loop Detection] Ringhash loop: " << polarcontexts_.size()-1 << "--"
//                 << loop_id << ", similarity:" << 1-min_dist
//                 << std::endl;

//     std::stringstream lidar_data_path;
//     lidar_data_path << lidar_path << std::setfill('0') << std::setw(6)
//                     << cloudInd<< ".bin";
//     std::stringstream lidar_data_path1;
//     lidar_data_path1 << lidar_path << std::setfill('0') << std::setw(6) << (nn_idx+1) << ".bin";
//     file << "2\t" << (cloudInd) << "-" << (nn_idx+1)  << "\t" << 1-min_dist<< "\t0\t0\t0\t"<< lidar_data_path.str() << "\t" <<lidar_data_path1.str()<< std::endl;
    
//     }
//     else{
//     std::stringstream lidar_data_path;
//     lidar_data_path << lidar_path << std::setfill('0') << std::setw(6)
//                     << cloudInd << ".bin";
//     std::stringstream lidar_data_path1;
//     lidar_data_path1 << lidar_path << std::setfill('0') << std::setw(6) << (nn_idx+1) << ".bin";
//     file << "2\t" << cloudInd << "-" << (nn_idx+1) << "\t" << 1-min_dist<< "\t0\t0\t0\t"<< lidar_data_path.str() << "\t" <<lidar_data_path1.str()<< std::endl;
    
//     }
    
//     return result;

// } // SCManager::detectLoopClosureID