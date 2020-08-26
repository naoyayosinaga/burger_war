#include <ros/ros.h>

//amcl
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <tf/transform_broadcaster.h>

//ball recognition
#include <geometry_msgs/Pose2D.h>

//Obstacle_detector
#include <obstacle_detector/Obstacles.h>

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
    obstacles_position fix_obstacles_position[5]={
        {   0.0,     0.0, 0.400},
        { 0.530,   0.530, 0.250},
        { -0.530,  0.530, 0.250},
        {  0.530, -0.530, 0.250},
        {  0.530,  0.530, 0.250},
    };
};

//コンストラクタ
opponent_detector::opponent_detector(){
    sub_raw_obstacles = nh.subscribe("/raw_obstacles", 5, &opponent_detector::cb_raw_obstacles, this);
    sub_cam_ball_relative_pose = nh.subscribe("/cam_ball_relative_pose", 5, &opponent_detector::cb_cam_pose, this);
    pub_opponent_position = nh.advertise<geometry_msgs::Pose2D>("opponent_position", 1);
};

//コールバック関数
//obstacle_detectorから障害物情報を取得
void opponent_detector::cb_raw_obstacles(const obstacle_detector::Obstacles::ConstPtr &msg){
    //円の障害物だけ取得
    obstacle_detector::CircleObstacle circle_obstacle = *msg.get()->circles.data();
    std::vector<obstacle_detector::CircleObstacle, std::allocator<obstacle_detector::CircleObstacle>> a;
    a = msg.get()->circles;
    ROS_INFO("%d",a.size());

    //障害物の数を取得する
    int obstacle_number = circle_obstacle.center.x;
    //データの中から障害物を排除

    //データの中から壁の外側のデータを排除

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
