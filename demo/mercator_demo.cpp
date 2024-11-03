#include "include/merDesc.h"
#include "include/mercator.h"
#include <nav_msgs/Odometry.h>
#include <pcl_conversions/pcl_conversions.h>



// // loop detector 
MCManager mcManager;


// Read KITTI data
std::vector<float> read_lidar_data(const std::string lidar_data_path) {
  std::ifstream lidar_data_file;
  lidar_data_file.open(lidar_data_path,
                       std::ifstream::in | std::ifstream::binary);
  if (!lidar_data_file) {
    std::cout << "Read End..." << std::endl;
    std::vector<float> nan_data;
    return nan_data;
    // exit(-1);
  }
  lidar_data_file.seekg(0, std::ios::end);
  const size_t num_elements = lidar_data_file.tellg() / sizeof(float);
  lidar_data_file.seekg(0, std::ios::beg);

  std::vector<float> lidar_data_buffer(num_elements);
  lidar_data_file.read(reinterpret_cast<char *>(&lidar_data_buffer[0]),
                       num_elements * sizeof(float));
  return lidar_data_buffer;
}


int main(int argc, char **argv) {
  ros::init(argc, argv, "scancontext_demo");
  ros::NodeHandle nh;
  std::string lidar_path = "";
  std::string pose_path = "";
  std::string output_timepath = "";
  std::string output_posepath = "";

  nh.param<std::string>("lidar_path", lidar_path, "");
  nh.param<std::string>("pose_path", pose_path, "");
  nh.param<std::string>("output_timepath", output_timepath, "");
  nh.param<std::string>("output_posepath", output_posepath, "");


  ConfigSetting config_setting;
  read_parameters(nh, config_setting);
///写入数据
  std::ofstream outFile(output_posepath); 
  if (!outFile.is_open()) {
      std::cerr << "Failed to open file for writing." << std::endl;
      return 1;
  }


  ros::Publisher pubOdomAftMapped =
      nh.advertise<nav_msgs::Odometry>("/aft_mapped_to_init", 10);

  ros::Publisher pubCureentCloud =
      nh.advertise<sensor_msgs::PointCloud2>("/cloud_current", 100);
 
  ros::Publisher pubMatchedCloud =
      nh.advertise<sensor_msgs::PointCloud2>("/cloud_matched", 100);



  ros::Rate loop(500);
  ros::Rate slow_loop(10);

  std::vector<std::pair<Eigen::Vector3d, Eigen::Matrix3d>> poses_vec;
  std::vector<double> times_vec;
  load_pose_with_time(pose_path, poses_vec, times_vec);
  std::cout<<"pose_path:"<<pose_path<<endl;
  std::cout << "Sucessfully load pose with number: " << poses_vec.size()
            << std::endl;


  size_t cloudInd = 0;
  size_t keyCloudInd = 0;
  pcl::PointCloud<pcl::PointXYZI>::Ptr temp_cloud(
      new pcl::PointCloud<pcl::PointXYZI>());

  std::vector<double> descriptor_time;
  std::vector<double> querying_time;
  std::vector<double> update_time;
  int triggle_loop_num = 0;
  while (ros::ok()) {
    std::stringstream lidar_data_path;
    lidar_data_path << lidar_path << std::setfill('0') << std::setw(6)
                    << cloudInd << ".bin";
    std::vector<float> lidar_data = read_lidar_data(lidar_data_path.str());
    if (lidar_data.size() == 0) {
      break;
    }
    pcl::PointCloud<pcl::PointXYZI>::Ptr current_cloud(
        new pcl::PointCloud<pcl::PointXYZI>());
    pcl::PointCloud<pcl::PointXYZI>::Ptr row_cloud(
        new pcl::PointCloud<pcl::PointXYZI>());
    // std::cout<<"skdjahjdas"<<endl;

    Eigen::Vector3d translation = poses_vec[cloudInd].first;
    // std::cout<<"skdjahjdas"<<endl;
    Eigen::Matrix3d rotation = poses_vec[cloudInd].second;
    for (std::size_t i = 0; i < lidar_data.size(); i += 4) {
      pcl::PointXYZI point;
      point.x = lidar_data[i];
      point.y = lidar_data[i + 1];
      point.z = lidar_data[i + 2];
      point.intensity = lidar_data[i + 3];
      Eigen::Vector3d pv = point2vec(point);
      pv = rotation * pv + translation;
      point =point = vec2point(pv);
      current_cloud->push_back(point);
    }
    for (std::size_t i = 0; i < lidar_data.size(); i += 4) {
      pcl::PointXYZI point;
      point.x = lidar_data[i];
      point.y = lidar_data[i + 1];
      point.z = lidar_data[i + 2];
      point.intensity = lidar_data[i + 3];

      row_cloud->push_back(point);
    }
    down_sampling_voxel(*current_cloud, config_setting.ds_size_);
    down_sampling_voxel(*row_cloud, config_setting.ds_size_);
    
    for (auto pv : current_cloud->points) {
      temp_cloud->points.push_back(pv);
    }

    // check if keyframe
    if (cloudInd % config_setting.sub_frame_num_ == 0 && cloudInd != 0) {
      std::cout << "Key Frame id:" << keyCloudInd
                << ", cloud size: " << config_setting.sub_frame_num_ << std::endl;
    //   step1. Descriptor Extraction
      auto t_descriptor_begin = std::chrono::high_resolution_clock::now();
      mcManager.makeAndSaveScancontextAndKeys(*row_cloud);
      

      auto t_descriptor_end = std::chrono::high_resolution_clock::now();
      descriptor_time.push_back(time_inc(t_descriptor_end, t_descriptor_begin));
      // step2. Searching Loop
      auto t_query_begin = std::chrono::high_resolution_clock::now();
      std::pair<int, double> detectResult(-1, 0);
      if(keyCloudInd > config_setting.skip_near_num_) {
        // detectResult = mcManager.detectLoopClosureID1(outFile,lidar_path,cloudInd);
        detectResult = mcManager.detectLoopClosureID();

      }

      auto t_query_end = std::chrono::high_resolution_clock::now();
      querying_time.push_back(time_inc(t_query_end, t_query_begin));



      // step3. Add descriptors to the database
      auto t_map_update_begin = std::chrono::high_resolution_clock::now();

      mcManager.makeScancontext(*row_cloud);

      auto t_map_update_end = std::chrono::high_resolution_clock::now();
      update_time.push_back(time_inc(t_map_update_end, t_map_update_begin));


      pcl::PointCloud<pcl::PointXYZI> save_key_cloud;
      save_key_cloud = *temp_cloud;

      mcManager.key_cloud_vec_.push_back(save_key_cloud.makeShared());

      // publish

      sensor_msgs::PointCloud2 pub_cloud;
      pcl::toROSMsg(*temp_cloud, pub_cloud);
      pub_cloud.header.frame_id = "camera_init";
      pubCureentCloud.publish(pub_cloud);

      //wangzika：发布匹配到回环的帧
      if (detectResult.first > 0) {
        triggle_loop_num++;
        pcl::toROSMsg(*mcManager.key_cloud_vec_[detectResult.first],
                      pub_cloud);
        pub_cloud.header.frame_id = "camera_init";
        pubMatchedCloud.publish(pub_cloud);
        slow_loop.sleep();

      }
      if(keyCloudInd%5==0&&keyCloudInd>=5){
        temp_cloud->clear();
        // std::cout<<"clear"<<std::endl;
      }
      keyCloudInd++;
      loop.sleep();
    }
    // wangzika:发布里程计的地方
    nav_msgs::Odometry odom;
    odom.header.frame_id = "camera_init";
    odom.pose.pose.position.x = translation[0];
    odom.pose.pose.position.y = translation[1];
    odom.pose.pose.position.z = translation[2];
    Eigen::Quaterniond q(rotation);
    odom.pose.pose.orientation.w = q.w();
    odom.pose.pose.orientation.x = q.x();
    odom.pose.pose.orientation.y = q.y();
    odom.pose.pose.orientation.z = q.z();
    pubOdomAftMapped.publish(odom);
    loop.sleep();
    cloudInd++;
  }
//wzb
  std::ofstream outFiletime(output_timepath);
  if (outFiletime.is_open()) {
      for (size_t i = 0; i < descriptor_time.size(); ++i) {
          // Check if the other vectors have enough elements
          if (i < querying_time.size() && i < update_time.size()) {
              outFiletime << descriptor_time[i] << " " 
                      << querying_time[i] << " " 
                      << update_time[i] << std::endl;
          }
      }
      outFiletime.close(); // Close the file when done
  } else {
      std::cerr << "Unable to open file for writing." << std::endl;
      // return 1;
  }


  double mean_descriptor_time =
      std::accumulate(descriptor_time.begin(), descriptor_time.end(), 0) * 1.0 /
      descriptor_time.size();
  double mean_query_time =
      std::accumulate(querying_time.begin(), querying_time.end(), 0) * 1.0 /
      querying_time.size();
  double mean_update_time =
      std::accumulate(update_time.begin(), update_time.end(), 0) * 1.0 /
      update_time.size();
  std::cout << "Total key frame number:" << keyCloudInd
            << ", loop number:" << triggle_loop_num << std::endl;
  std::cout << "Mean time for descriptor extraction: " << mean_descriptor_time
            << "ms, query: " << mean_query_time
            << "ms, update: " << mean_update_time << "ms, total: "
            << mean_descriptor_time + mean_query_time + mean_update_time << "ms"
            << std::endl;
  outFile.close();
  return 0;
}