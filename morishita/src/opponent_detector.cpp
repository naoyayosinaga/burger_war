#include <ros/ros.h>

//amcl
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <tf/transform_broadcaster.h>

//ball recognition
#include <geometry_msgs/Pose2D.h>

//Obstacle_detector
#include <obstacle_detector/Obstacles.h>

#include <visualization_msgs/Marker.h>

struct obstacles_position{
    double x;
    double y;
    double size;
};

class opponent_detector{
    public:
    opponent_detector();
    private:
    //トピックのコールバック関数
    void cb_raw_obstacles(const obstacle_detector::Obstacles::ConstPtr &msg);
    void cb_cam_pose(const geometry_msgs::Pose2D::ConstPtr &msg);
    void cb_amcl_pose(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr &msg);
    void cb_time_control(const ros::TimerEvent&);

    //ノードハンドラ作成
	ros::NodeHandle nh;

    //時間の関数作成
    ros::Timer timer;

    //使用するpub/sebの定義
    ros::Subscriber sub_raw_obstacles;
    ros::Subscriber sub_cam_ball_relative_pose;
    ros::Publisher pub_opponent_position;
    ros::Publisher pub_marker;

    //自己位置推定の結果
    geometry_msgs::Pose2D amcl_position;

    //カメラ得られた地図座標における相手の位置情報
    geometry_msgs::Pose2D cam_opponent_pose;

    //カメラによって10秒以内に相手の位置が取得できているかのフラグ
    int camera_flag = -1;
    //値の意味
    //1:相手の位置が取得出来ている
    //-1:相手の位置は取得できていない

    //カメラで相手の位置を取得できた時間
    ros::Time camera_time;

    //固定の障害物情報
    int fix_obstacles_number = 5;
    obstacles_position fix_obstacles_position[5]={
        {   0.0,     0.0, 0.400},
        { 0.530,   0.530, 0.250},
        { -0.530,  0.530, 0.250},
        {  0.530, -0.530, 0.250},
        { -0.530, -0.530, 0.250},
    };

    //デバック用のマーカ
    visualization_msgs::Marker marker_control ;
};

//コンストラクタ
opponent_detector::opponent_detector(){
    sub_raw_obstacles = nh.subscribe("/raw_obstacles", 5, &opponent_detector::cb_raw_obstacles, this);
    sub_cam_ball_relative_pose = nh.subscribe("/cam_ball_relative_pose", 5, &opponent_detector::cb_cam_pose, this);
    pub_opponent_position = nh.advertise<geometry_msgs::Pose2D>("opponent_position", 1);
    pub_marker= nh.advertise<visualization_msgs::Marker>("visualization_marker", 1);
};

//コールバック関数
//obstacle_detectorから障害物情報を取得
void opponent_detector::cb_raw_obstacles(const obstacle_detector::Obstacles::ConstPtr &msg){
    std::vector<obstacle_detector::CircleObstacle, std::allocator<obstacle_detector::CircleObstacle>> circle_obstacle;
    std::vector<obstacle_detector::CircleObstacle, std::allocator<obstacle_detector::CircleObstacle>> filter_fix_obstacle;
    std::vector<obstacle_detector::CircleObstacle, std::allocator<obstacle_detector::CircleObstacle>> filter_wall;
    //円の障害物だけ取得
    //obstacle_detector::CircleObstacle circle_obstacle = *msg.get()->circles.data();
    circle_obstacle = msg.get()->circles;
    //ROS_INFO("before %d",circle_obstacle.size());

    //データの中から障害物を排除
    for(int i=0; i<circle_obstacle.size(); i++){
        int filter_flag = -1;
        for(int j=0; j<5; j++){
            //ROS_INFO("%f %f",circle_obstacle[i].center.x, circle_obstacle[i].center.y);
            //if((0 < circle_obstacle[i].center.x) && (0 < circle_obstacle[i].center.y)){
            if(((fix_obstacles_position[j].x - fix_obstacles_position[j].size) < circle_obstacle[i].center.x) && (circle_obstacle[i].center.x < (fix_obstacles_position[j].x + fix_obstacles_position[j].size)) && ((fix_obstacles_position[j].y - fix_obstacles_position[j].size) < circle_obstacle[i].center.y) && (circle_obstacle[i].center.y < (fix_obstacles_position[j].y + fix_obstacles_position[j].size))){
                //障害物の範囲内ならfilter_flagを立てる
                filter_flag = 1;
            }
        }
        //filter_flagが立ってなかったらデータを取得する
        if(filter_flag == -1){
            filter_fix_obstacle.push_back(circle_obstacle[i]);
            ROS_INFO("%f %f", filter_fix_obstacle[0].center.x, filter_fix_obstacle[0].center.y);
        }
    }
    //ROS_INFO("filter obstacle %d",filter_fix_obstacle.size());
    //データの中から壁の外側のデータを排除
    if(filter_fix_obstacle.size() != 0){
        for(int i=0; i<filter_fix_obstacle.size(); i++){
            //地図上の右上と右下にデータがある場合
            if(filter_fix_obstacle[i].center.x > 0){
                //xの地点における最大、最小のyの範囲を計算
                double wall_limit_up_y = -filter_fix_obstacle[i].center.x + 1.6;
                double wall_limit_down_y = filter_fix_obstacle[i].center.x -1.6;

                //範囲内なら壁や障害物ではなく相手のロボットと判断
                if((wall_limit_down_y < filter_fix_obstacle[i].center.y) && (filter_fix_obstacle[i].center.y < wall_limit_up_y)){
                    filter_wall.push_back(filter_fix_obstacle[i]);
                }

            }
            //地図上の左上と左下にデータがある場合
            else{
                //xの地点における最大、最小のyの範囲を計算
                double wall_limit_up_y = filter_fix_obstacle[i].center.x + 1.6;
                double wall_limit_down_y = -filter_fix_obstacle[i].center.x -1.6;

                //範囲内なら壁や障害物ではなく相手のロボットと判断
                if((wall_limit_down_y < filter_fix_obstacle[i].center.y) && (filter_fix_obstacle[i].center.y < wall_limit_up_y)){
                    filter_wall.push_back(filter_fix_obstacle[i]);
                    ROS_INFO("%f %f", filter_fix_obstacle[i].center.x, filter_fix_obstacle[i].center.y);
                }
            }
        }
    }
    //デバック
    /*
    if(filter_wall.size() != 0){
        for(int i=0; filter_wall.size(); i++){
            ROS_INFO("%f %f", filter_wall[i].center.x, filter_wall[i].center.y);
        }
    }
    */
    if(filter_wall.size() > 0){
        for(int i=0; filter_wall.size(); i++){
            marker_control.header.frame_id = "/map";
            marker_control.header.stamp = ros::Time::now();
            marker_control.id = 0;
            marker_control.type = visualization_msgs::Marker::SPHERE;
            marker_control.action = visualization_msgs::Marker::ADD;
            marker_control.pose.position.x = filter_wall[0].center.x;
            marker_control.pose.position.y = filter_wall[0].center.y;
            marker_control.pose.position.z = 0;
            marker_control.pose.orientation.x = 0.0;
            marker_control.pose.orientation.y = 0.0;
            marker_control.pose.orientation.z = 0.0;
            marker_control.pose.orientation.w = 1.0;

            marker_control.scale.x = 0.1;
            marker_control.scale.y = 0.1;
            marker_control.scale.z = 0.1;

            marker_control.color.r = 1.0f;
            marker_control.color.g = 0.0f;
            marker_control.color.b = 0.0f;
            marker_control.color.a = 0.8;

            marker_control.lifetime = ros::Duration();

            pub_marker.publish(marker_control);
        }
   }
    
    

    //データの中から前回のデータから近いものを選択
}

//自分の現在位置を取得
void opponent_detector::cb_amcl_pose(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr &msg){
    //自己位置推定の結果をPose2Dに変換
    amcl_position.x = msg.get()->pose.pose.position.x;
    amcl_position.y = msg.get()->pose.pose.position.y;
    //クォータニオン→オイラー角
    tf::Quaternion quat(msg.get()->pose.pose.orientation.x,msg.get()->pose.pose.orientation.y,msg.get()->pose.pose.orientation.z,msg.get()->pose.pose.orientation.w);
    double r, p, yaw;
    tf::Matrix3x3(quat).getRPY(r, p, yaw);
    amcl_position.theta = yaw;
}

//cam_ball_recognitionから相手の情報を取得
void opponent_detector::cb_cam_pose(const geometry_msgs::Pose2D::ConstPtr &msg){
    //カメラで相手の位置が取得出来ていない場合、前回取得した時から10秒経っている場合camera_flagをおる
    if((msg.get()->x < 0.1) && (msg.get()->y < 0.1) && (msg.get()->theta < 0.1)){
        if((ros::Time::now() - camera_time).toSec() > 10.0){
            camera_flag = -1;
        }
    }
    //カメラで相手の位置を取得した場合、時間を記録、camera_flagを立ててマップ上の位置を計算
    else{
        //ROS_INFO("Get camera data");
        camera_time = ros::Time::now();
        camera_flag = 1;
        cam_opponent_pose.x = amcl_position.x + (msg.get()->x/1000.0) * cos(amcl_position.theta) - (msg.get()->y/1000.0) * sin(amcl_position.theta);
        cam_opponent_pose.y = amcl_position.y + (msg.get()->x/1000.0) * sin(amcl_position.theta) + (msg.get()->y/1000.0) * cos(amcl_position.theta);
    }
}

//関数
//1秒周期に実行される制御関数
void opponent_detector::cb_time_control(const ros::TimerEvent&){
    //cam_
}



int main(int argc, char** argv)
{
	ros::init(argc, argv, "opponent_detector");
	opponent_detector opponent_detector;
	ros::spin();
	return 0;
}
