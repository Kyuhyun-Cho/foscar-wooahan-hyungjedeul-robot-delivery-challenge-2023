#include "header.h"
#include "dbscan.h"

// pcl point type
typedef pcl::PointXYZ PointT;
// cluster point type
typedef pcl::PointXYZI clusterPointT;

// ROI parameter
double xMinROI, xMaxROI, yMinROI, yMaxROI, zMinROI, zMaxROI;
double xMinBoundingBox, xMaxBoundingBox, yMinBoundingBox, yMaxBoundingBox, zMinBoundingBox, zMaxBoundingBox;
// DBScan parameter
int minPoints;
double epsilon, minClusterSize, maxClusterSize;
// VoxelGrid parameter
float leafSize;

// publisher
ros::Publisher pubROI;
ros::Publisher pubVoxelDownSampled;
ros::Publisher pubCluster;
ros::Publisher pubObjectInfo;
ros::Publisher pubObjectShapeArray;

//MSG
lidar_object_detection::ObjectInfo objectInfoMsg;

void cfgCallback(lidar_object_detection::hyper_parameterConfig &config, int32_t level) {
    xMinROI = config.xMinROI;
    xMaxROI = config.xMaxROI;
    yMinROI = config.yMinROI;
    yMaxROI = config.yMaxROI;
    zMinROI = config.zMinROI;
    zMaxROI = config.zMaxROI;

    minPoints = config.minPoints;
    epsilon = config.epsilon;
    minClusterSize = config.minClusterSize;
    maxClusterSize = config.maxClusterSize;

    xMinBoundingBox = config.xMinBoundingBox;
    xMaxBoundingBox = config.xMaxBoundingBox;
    yMinBoundingBox = config.yMinBoundingBox;
    yMaxBoundingBox = config.yMaxBoundingBox;
    zMinBoundingBox = config.zMinBoundingBox;
    zMaxBoundingBox = config.zMaxBoundingBox;

    leafSize  = config.leafSize;
}

pcl::PointCloud<PointT>::Ptr ROI (const sensor_msgs::PointCloud2ConstPtr& input) {
    // ... do data processing
    pcl::PointCloud<PointT>::Ptr cloud(new pcl::PointCloud<PointT>);

    pcl::fromROSMsg(*input, *cloud); // sensor_msgs -> PointCloud 형변환

    pcl::PointCloud<PointT>::Ptr cloud_filtered(new pcl::PointCloud<PointT>);
    // pcl::PointCloud<PointT>::Ptr *retPtr = &cloud_filtered;

    // 오브젝트 생성 
    // Z축 ROI
    pcl::PassThrough<PointT> filter;
    filter.setInputCloud(cloud);                //입력 
    filter.setFilterFieldName("z");             //적용할 좌표 축 (eg. Z축)
    filter.setFilterLimits(zMinROI, zMaxROI);          //적용할 값 (최소, 최대 값)
    //filter.setFilterLimitsNegative (true);     //적용할 값 외 
    filter.filter(*cloud_filtered);             //필터 적용 

    // X축 ROI
    // pcl::PassThrough<PointT> filter;
    filter.setInputCloud(cloud_filtered);                //입력 
    filter.setFilterFieldName("x");             //적용할 좌표 축 (eg. X축)
    filter.setFilterLimits(xMinROI, xMaxROI);          //적용할 값 (최소, 최대 값)
    //filter.setFilterLimitsNegative (true);     //적용할 값 외 
    filter.filter(*cloud_filtered);             //필터 적용 

    // Y축 ROI
    // pcl::PassThrough<PointT> filter;
    filter.setInputCloud(cloud_filtered);                //입력 
    filter.setFilterFieldName("y");             //적용할 좌표 축 (eg. Y축)
    filter.setFilterLimits(yMinROI, yMaxROI);          //적용할 값 (최소, 최대 값)
    //filter.setFilterLimitsNegative (true);     //적용할 값 외 
    filter.filter(*cloud_filtered);             //필터 적용 

    // 포인트수 출력

    sensor_msgs::PointCloud2 roi_raw;
    pcl::toROSMsg(*cloud_filtered, roi_raw);
    
    pubROI.publish(roi_raw);

    return cloud_filtered;
}

pcl::PointCloud<PointT>::Ptr voxelGrid(pcl::PointCloud<PointT>::Ptr input) {
    //Voxel Grid를 이용한 DownSampling
    pcl::VoxelGrid<PointT> vg;    // VoxelGrid 선언
    pcl::PointCloud<PointT>::Ptr cloud_filtered(new pcl::PointCloud<PointT>); //Filtering 된 Data를 담을 PointCloud 선언
    vg.setInputCloud(input);             // Raw Data 입력
    vg.setLeafSize(leafSize, leafSize, leafSize); // 사이즈를 너무 작게 하면 샘플링 에러 발생
    vg.filter(*cloud_filtered);          // Filtering 된 Data를 cloud PointCloud에 삽입
    
    std::cout << "After Voxel Filtered :" << cloud_filtered->width * cloud_filtered->height  << '\n'; 

    pcl::PCLPointCloud2 pcl_pc2;
    pcl::toPCLPointCloud2(*cloud_filtered, pcl_pc2);

    sensor_msgs::PointCloud2 voxel_downsampled;
    pcl_conversions::fromPCL(pcl_pc2, voxel_downsampled);

    pubVoxelDownSampled.publish(voxel_downsampled);

    return cloud_filtered;
}

void cluster(pcl::PointCloud<PointT>::Ptr input) {
    if (input->empty())
        return;

    //KD-Tree
    pcl::search::KdTree<PointT>::Ptr tree(new pcl::search::KdTree<PointT>);
    pcl::PointCloud<clusterPointT>::Ptr clusterPtr(new pcl::PointCloud<clusterPointT>);
    tree->setInputCloud(input);

    //Segmentation
    std::vector<pcl::PointIndices> cluster_indices;

    //DBSCAN with Kdtree for accelerating
    DBSCANKdtreeCluster<PointT> dc;
    dc.setCorePointMinPts(minPoints);   //Set minimum number of neighbor points
    dc.setClusterTolerance(epsilon); //Set Epsilon 
    dc.setMinClusterSize(minClusterSize);
    dc.setMaxClusterSize(maxClusterSize);
    dc.setSearchMethod(tree);
    dc.setInputCloud(input);
    dc.extract(cluster_indices);

    pcl::PointCloud<clusterPointT> totalcloud_clustered;
    int cluster_id = 0;

    //각 Cluster 접근
    for (std::vector<pcl::PointIndices>::const_iterator it = cluster_indices.begin(); it != cluster_indices.end(); it++, cluster_id++) {
        pcl::PointCloud<clusterPointT> eachcloud_clustered;
        float cluster_counts = cluster_indices.size();

        //각 Cluster내 각 Point 접근
        for(std::vector<int>::const_iterator pit = it->indices.begin(); pit != it->indices.end(); ++pit) {

            clusterPointT tmp;
            tmp.x = input->points[*pit].x; 
            tmp.y = input->points[*pit].y;
            tmp.z = input->points[*pit].z;
            tmp.intensity = cluster_id % 100; // 상수 : 예상 가능한 cluster 총 개수
            eachcloud_clustered.push_back(tmp);
            totalcloud_clustered.push_back(tmp);
        }

        //minPoint와 maxPoint 받아오기
        clusterPointT minPoint, maxPoint;
        pcl::getMinMax3D(eachcloud_clustered, minPoint, maxPoint);

        objectInfoMsg.lengthX[cluster_id] = maxPoint.x - minPoint.x; // 
        objectInfoMsg.lengthY[cluster_id] = maxPoint.y - minPoint.y; // 
        objectInfoMsg.lengthZ[cluster_id] = maxPoint.z - minPoint.z; // 
        objectInfoMsg.centerX[cluster_id] = (minPoint.x + maxPoint.x)/2; //직육면체 중심 x 좌표
        objectInfoMsg.centerY[cluster_id] = (minPoint.y + maxPoint.y)/2; //직육면체 중심 y 좌표
        objectInfoMsg.centerZ[cluster_id] = (minPoint.z + maxPoint.z)/2; //직육면체 중심 z 좌표
        
    }
    objectInfoMsg.objectCounts = cluster_id;
    pubObjectInfo.publish(objectInfoMsg);

    sensor_msgs::PointCloud2 cluster_point;
    pcl::toROSMsg(totalcloud_clustered, cluster_point);
    cluster_point.header.frame_id = "velodyne";
    pubCluster.publish(cluster_point);
}

void visualizeObject() {
    visualization_msgs::MarkerArray objectShapeArray;
    visualization_msgs::Marker objectShape;
    uint32_t shape = visualization_msgs::Marker::CYLINDER; // Set our initial shape type to be a cube
    // Set the frame ID and timestamp.  See the TF tutorials for information on these.
    objectShape.header.frame_id = "velodyne"; 
    objectShape.ns = "object_shape";
    // Set the marker type.  Initially this is CUBE, and cycles between that and SPHERE, ARROW, and CYLINDER
    objectShape.type = shape;
    // Set the marker action.  Options are ADD and DELETE
    objectShape.action = visualization_msgs::Marker::ADD;
    for (int i = 0; i < objectInfoMsg.objectCounts; i++) {
        if (xMinBoundingBox <= objectInfoMsg.lengthX[i] && objectInfoMsg.lengthX[i] <= xMaxBoundingBox &&
            yMinBoundingBox <= objectInfoMsg.lengthY[i] && objectInfoMsg.lengthY[i] <= yMaxBoundingBox &&
            zMinBoundingBox <= objectInfoMsg.lengthZ[i] && objectInfoMsg.lengthZ[i] <= zMaxBoundingBox) {

            // Set the namespace and id for this marker.  This serves to create a unique ID
            // Any marker sent with the same namespace and id will overwrite the old one
            objectShape.header.stamp = ros::Time::now();
            objectShape.id = 100+i; // 

            // Set the pose of the marker.  This is a full 6DOF pose relative to the frame/time specified in the header
            objectShape.pose.position.x = objectInfoMsg.centerX[i];
            objectShape.pose.position.y = objectInfoMsg.centerY[i];
            objectShape.pose.position.z = objectInfoMsg.centerZ[i];

            // Set the scale of the marker -- 1x1x1 here means 1m on a side
            // objectShape.scale.x = objectInfoMsg.lengthX[i];
            // objectShape.scale.y = objectInfoMsg.lengthY[i];
            // objectShape.scale.z = objectInfoMsg.lengthZ[i];
            objectShape.scale.x = 4.5;
            objectShape.scale.y = 4.5;
            objectShape.scale.z = objectInfoMsg.lengthZ[i];

            // Set the color -- be sure to set alpha to something non-zero!
            objectShape.color.r = 1.0;
            objectShape.color.g = 1.0;
            objectShape.color.b = 1.0;
            objectShape.color.a = 0.5;

            objectShape.lifetime = ros::Duration(0.1);
            objectShapeArray.markers.emplace_back(objectShape);
        }
    }

    // Publish the marker
    pubObjectShapeArray.publish(objectShapeArray);
}

void mainCallback(const sensor_msgs::PointCloud2ConstPtr& input) {
    pcl::PointCloud<PointT>::Ptr cloudPtr;

    // main process method
    cloudPtr = ROI(input);
    cloudPtr = voxelGrid(cloudPtr);
    cluster(cloudPtr);

    // visualize method
    visualizeObject();
}

int main (int argc, char** argv) {
    // Initialize ROS
    ros::init (argc, argv, "lidar_object_detection");
    ros::NodeHandle nh;
    
    dynamic_reconfigure::Server<lidar_object_detection::hyper_parameterConfig> server;
    dynamic_reconfigure::Server<lidar_object_detection::hyper_parameterConfig>::CallbackType f;

    f = boost::bind(&cfgCallback, _1, _2);
    server.setCallback(f);
    // Create a ROS subscriber for the input point cloud
    ros::Subscriber sub = nh.subscribe ("/velodyne_points", 1, mainCallback);

    // Create a ROS publisher for the output point cloud
    pubROI = nh.advertise<sensor_msgs::PointCloud2> ("/roi_raw", 1);
    pubVoxelDownSampled = nh.advertise<sensor_msgs::PointCloud2> ("/voxel_downsampling", 1);
    pubCluster = nh.advertise<sensor_msgs::PointCloud2>("/cluster", 1);
    pubObjectInfo = nh.advertise<lidar_object_detection::ObjectInfo>("/object_info", 1);
    pubObjectShapeArray = nh.advertise<visualization_msgs::MarkerArray>("/marker", 1);

    // Spin
    ros::spin();
}