#include <ros/ros.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/PoseStamped.h>
#include <tf/transform_broadcaster.h>
#include <math.h>

//war_state
#include <std_msgs/String.h>

//amcl
#include <geometry_msgs/PoseWithCovarianceStamped.h>

//ball recognition
#include <geometry_msgs/Pose2D.h>

//move_base_status
#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib_msgs/GoalStatusArray.h>
#include <actionlib/client/simple_action_client.h>

//json読み込み用ライブラリ
#include <boost/property_tree/ptree.hpp>
#include <boost/property_tree/json_parser.hpp>
#include <boost/foreach.hpp>
#include <boost/optional.hpp>

#define grid_size 1.0
#define two_marker_x 0.1
#define two_marker_y 0.5
#define four_marker 0.5

struct marker_read_position{
    double x;
    double y;
    double theta;
};

class war_state_navigation{
    public:
    war_state_navigation();
    private:
    //トピックのコールバック関数
    void cb_move_base_status(const actionlib_msgs::GoalStatusArray::ConstPtr &status);
    void cb_war_state(const std_msgs::String::ConstPtr &msg);
    void cb_time_control(const ros::TimerEvent&);
    void cb_amcl_pose(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr &msg);
    void cb_Pose_2D(const geometry_msgs::Pose2D::ConstPtr &msg);

    //関数
    void set_near_marker_target(geometry_msgs::Pose2D current_my_position);
    void navigation_goal_set(double x,double y,double theta);

    //ノードハンドラ作成
	ros::NodeHandle nh;
    
    //時間の関数作成
    ros::Timer timer;

    

    //使用するpub/sebの定義
    ros::Publisher pub_goal;
    ros::Subscriber sub_war_state;
    ros::Subscriber sub_navi_status;
    ros::Subscriber sub_amcl_pose;

    //navigationのステータスID
    int status_id = 0;
    //uint8 PENDING         = 0
    //uint8 ACTIVE          = 1
    //uint8 PREEMPTED       = 2
    //uint8 SUCCEEDED       = 3
    //uint8 ABORTED         = 4
    //uint8 REJECTED        = 5
    //uint8 PREEMPTING      = 6
    //uint8 RECALLING       = 7
    //uint8 RECALLED        = 8
    //uint8 LOST            = 9


    //試合時間
    double match_time = 180.0;

    //war_stateから得られる情報
    //現在の試合時間
    double current_time = 0.0;

    //自分と相手のスコア
    int r_point = 0;
    int b_point = 0;

    //試合が始まっているか判断するフラグ
    int start_flag = -1;
    //値の意味
    //-1:running以外
    //1:running

    //フィールド情報
    int field_info[18];
    //配列順番
    //Blue            0:B.1:L.2:R
    //Red             3:B.4:L.5:R
    //Tomato          6:N.7:S
    //Omelette        8:N.9:S
    //Pudding         10:N.11:S
    //OctopusWiener   12:N.13:S
    //FriedShrimp     14:N.15:E.16:W.17:S
    //値の意味
    //0：フリー、1：こちらが取得、-1：相手が取得

    //フィールドのマーカを読み取るために移動する位置
    //順番はフィールド情報と同じ
    marker_read_position field_marker_position[18]={
        {                        0,             0,       0},//Blue.B
        {                        0,             0,       0},//Blue.L
        {                        0,             0,       0},//Blue.R
        {                        0,             0,       0},
        {                        0,             0,       0},
        {                        0,             0,       0},
        {   grid_size-two_marker_x,  two_marker_y,    M_PI},
        {             two_marker_x,  two_marker_y,       0},
        {   grid_size-two_marker_x, -two_marker_y,    M_PI},
        {             two_marker_x, -two_marker_y,       0},
        {            -two_marker_x,  two_marker_y,    M_PI},
        {-(grid_size-two_marker_x),  two_marker_y,       0},
        {            -two_marker_x, -two_marker_y,    M_PI},
        {-(grid_size-two_marker_x), -two_marker_y,       0},
        {              four_marker,             0,    M_PI},//FriedShrimp.N
        {                        0,  -four_marker,  M_PI/2},//FriedShrimp.E
        {                        0,   four_marker, -M_PI/2},//FriedShrimp.W
        {             -four_marker,             0,       0},
    };

    //自己位置推定の結果
    geometry_msgs::Pose2D amcl_position;

    //カメラによって相手の位置が取得できているかのフラグ
    int camera_flag = -1;
    //値の意味
    //1:相手の位置が取得出来ている
    //-1:相手の位置は取得できていない
};

//コンストラクタ
war_state_navigation::war_state_navigation(){
    //購読するトピックの定義
    sub_war_state = nh.subscribe("/war_state", 5, &war_state_navigation::cb_war_state, this);
    sub_navi_status = nh.subscribe<actionlib_msgs::GoalStatusArray>("/move_base/status", 10, &war_state_navigation::cb_move_base_status, this);
    sub_amcl_pose = nh.subscribe("/amcl_pose", 5, &war_state_navigation::cb_amcl_pose, this);

    //配布するトピックの定義
    pub_goal = nh.advertise<geometry_msgs::PoseStamped>("move_base_simple/goal", 1);

    //時間で起動する関数の定義
    timer = nh.createTimer(ros::Duration(1.0), &war_state_navigation::cb_time_control,this);
}



//関数定義
void war_state_navigation::cb_war_state(const std_msgs::String::ConstPtr &msg){
    //データ受信
    std_msgs::String data = *msg;

    //StringからJsonデータ読み出し
    std::stringstream ss;
    ss << data.data;
    boost::property_tree::ptree pt;
    read_json(ss, pt);

    //stateとtimeを取得
    if (boost::optional<std::string> state = pt.get_optional<std::string>("state")){
        //stateを取得出来た時、timeも同時に取得
        boost::optional<double> time = pt.get_optional<double>("time");
        current_time = time.get();
        //stateがrunningかつtimeが0<ならstart_flagを立てる
        if((state.get().compare("running") == 0) && (0.0 <time.get()) && (time.get() < match_time)){
            start_flag = 1;
            //ROS_INFO("START");
        }else{
            start_flag = 0;
            //ROS_INFO("STOP!!");
        }
    }else{
        std::cout << "state is nothing" << std::endl;
    }

    //start_flagが立っていたら他の値も取得する
    if(start_flag ==1){
        //スコアを取得
        //scores.r
        if (boost::optional<int> value = pt.get_optional<int>("scores.r")) {
            r_point = value.get();
            //std::cout << " scores.r: " << value.get() << std::endl;
        }
        else {
            std::cout << "r_point is nothing" << std::endl;
        }
        //scores.b
        if (boost::optional<int> value = pt.get_optional<int>("scores.b")) {
            b_point = value.get();
            //std::cout << " scores.b: " << value.get() << std::endl;
        }
        else {
            std::cout << "b_point is nothing" << std::endl;
        }
        ROS_INFO("red = %d, blue = %d",r_point,b_point);
        //フィールド情報を取得
        int roop_number = 0;
        BOOST_FOREACH (const boost::property_tree::ptree::value_type& child, pt.get_child("targets")) {
            const boost::property_tree::ptree& info = child.second;

            //target.player
            if (boost::optional<std::string> player = info.get_optional<std::string>("player")) {
                //std::cout << "player : " << player.get() << std::endl;
                if(!(player.get().compare("r"))){
                    field_info[roop_number] = 1;
                }else if(!(player.get().compare("b"))){
                    field_info[roop_number] = -1;
                }else{
                    field_info[roop_number] = 0;
                }
                //ROS_INFO("%d,%d",roop_number,field_info[roop_number]);
            }
            else {
                std::cout << "player is nothing" << std::endl;
            }
            roop_number++;
        }
    }else{
        //何もしない
    }
}

void war_state_navigation::cb_move_base_status(const actionlib_msgs::GoalStatusArray::ConstPtr &status)
{
    //navigationの状態を取得
    if (!status->status_list.empty()){
        actionlib_msgs::GoalStatus goalStatus = status->status_list.back();
        status_id = goalStatus.status;
    }else{
        //何も取得できない場合は起動時と考えてstatu_idを-1に設定
        status_id = -1;
        ROS_INFO("navigation status lost");
    }
}

void war_state_navigation::cb_amcl_pose(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr &msg){
    //自己位置推定の結果をPose2Dに変換
    amcl_position.x = msg.get()->pose.pose.position.x;
    amcl_position.y = msg.get()->pose.pose.position.y;
    //クォータニオン→オイラー角
    tf::Quaternion quat(msg.get()->pose.pose.orientation.x,msg.get()->pose.pose.orientation.y,msg.get()->pose.pose.orientation.z,msg.get()->pose.pose.orientation.w);
    double r, p, yaw;
    tf::Matrix3x3(quat).getRPY(r, p, yaw);
    amcl_position.theta = yaw;
}

//関数
//1秒周期に実行される制御関数
void war_state_navigation::cb_time_control(const ros::TimerEvent&){
    //試合が開始していて、navigationのstatus_idが1以外なら自律移動していないと考えて、状態に応じて目標地点、行動を与える
    if(start_flag == 1 && status_id != 1){
        //試合時間が90秒以下なら、近くのマーカを取得
        if(current_time < 180.0){
            //自分が取得していない近くのマーカを検索して目標地点に設定
            war_state_navigation::set_near_marker_target(amcl_position);
        }else{
            ROS_INFO("180s over");
        }
        /*
        //90秒以上経過しているなら、得点差と相手の位置が取得できているかに応じて行動を決める
        else if(current_time > 90.0){
            //相手の位置が取得できていて引き分けもしくは勝っているなら、相手から離れて自分が取得していないマーカを目標地点に設定
            if(camera_flag == 1 && r_point >= b_point){

            }
            //相手の位置が取得出来ていなくて引き分けもしくは勝っているなら、自分が取得していない近くのマーカを検索して目標地点に設定
            else if(camera_flag == 0){

            }

        }
        */
    }
   
}

//自己位置推定の結果から一番近くのマーカに目標位置を設定
void war_state_navigation::set_near_marker_target(geometry_msgs::Pose2D current_my_position){
    //一番近くで自分のマーカではない位置を探索
    double min_distance = 100.0;
    int target_number = -1;
    for(int i=6; i<18; i++){
        //誰も取得していないマーカまでの距離を計算する
        if(field_info[i] == 0){
            double distance = sqrt( (pow((current_my_position.x - field_marker_position[i].x),2)) + (pow((current_my_position.y - field_marker_position[i].y), 2) ));
            if(distance < min_distance){
                min_distance = distance;
                target_number = i;
            }
        }
    }

    //target_numberが-1以外なら目標マーカの設定を行う
    if(target_number != -1){
        war_state_navigation::navigation_goal_set(field_marker_position[target_number].x, field_marker_position[target_number].y, field_marker_position[target_number].theta);
    }else{
        ROS_INFO("target_number lost");
    }
    
}

//navigationの目的地を設定して、Publishする
void war_state_navigation::navigation_goal_set(double x,double y,double theta){
    geometry_msgs::PoseStamped goal;
    goal.pose.position.x = x;
    goal.pose.position.y = y;
    tf::Quaternion quat=tf::createQuaternionFromRPY(0,0,theta);
    geometry_msgs::Quaternion geometry_quat;
    quaternionTFToMsg(quat, geometry_quat);
    goal.pose.orientation = geometry_quat;
    goal.header.frame_id = "map";
    pub_goal.publish(goal);
}

int main(int argc, char** argv)
{
	ros::init(argc, argv, "war_state_navigation");
	war_state_navigation war_state_navigation;
	ros::spin();
	return 0;
}
