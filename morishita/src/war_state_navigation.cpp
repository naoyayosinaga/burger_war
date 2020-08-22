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
#include <visualization_msgs/Marker.h>

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
    //状況に応じた目標マーカ位置設定
    void set_near_free_marker_target(geometry_msgs::Pose2D current_my_position);
    void set_near_marker_target(geometry_msgs::Pose2D current_my_position);
    void set_far_marker_target(geometry_msgs::Pose2D current_my_position, geometry_msgs::Pose2D current_opponent_position);
    void set_position(int target_number);

    //相手を追いかける動作
    void approach_opponent(geometry_msgs::Pose2D current_opponent_position);
    void serch_opponent_position(geometry_msgs::Pose2D current_my_position);

    //目標位置を設定する際の関数
    void navigation_goal_set(double x,double y,double theta);


    //ノードハンドラ作成
	ros::NodeHandle nh;
    
    //時間の関数作成
    ros::Timer timer;


    //使用するpub/sebの定義
    ros::Publisher pub_goal;
    ros::Publisher pub_marker;
    ros::Subscriber sub_war_state;
    ros::Subscriber sub_navi_status;
    ros::Subscriber sub_amcl_pose;
    ros::Subscriber sub_cam_pose;

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
    int field_info[18][2];
    //配列順番
    //Blue            0:B.1:L.2:R
    //Red             3:B.4:L.5:R
    //Tomato          6:N.7:S
    //Omelette        8:N.9:S
    //Pudding         10:N.11:S
    //OctopusWiener   12:N.13:S
    //FriedShrimp     14:N.15:E.16:W.17:S

    //値の意味
    //[][0]:どちらがマーカを取得したか
    //0：フリー、1：こちらが取得、-1：相手が取得
    //[][1]:目的地に設定したことがあるか
    //0:ない、1:ある

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

    //探索時のコーナ位置
    marker_read_position corner_position[4]={
        { -1.0,  0.0,     0.0},//N
        {  1.0,  0.0,    M_PI},//S
        {  0.0,  1.0, -M_PI/2},//W
        {  0.0, -1.0,  M_PI/2},//E
    };

    //自己位置推定の結果
    geometry_msgs::Pose2D amcl_position;
    //カメラから得られる相手の推定値
    geometry_msgs::Pose2D opponent_position;

    //カメラによって10秒以内に相手の位置が取得できているかのフラグ
    int camera_flag = -1;
    //値の意味
    //1:相手の位置が取得出来ている
    //-1:相手の位置は取得できていない

    //カメラで相手の位置を取得できた時間
    ros::Time camera_time;

    //デバック用のマーカ
    visualization_msgs::Marker marker_control ;

    //探索時の原点
    geometry_msgs::Pose2D origin_position;

    //90s後に一度だけ実行したい関数のためのフラグ
    int once_flag = 1;
    
};

//コンストラクタ
war_state_navigation::war_state_navigation(){
    //購読するトピックの定義
    sub_war_state = nh.subscribe("/war_state", 5, &war_state_navigation::cb_war_state, this);
    sub_navi_status = nh.subscribe<actionlib_msgs::GoalStatusArray>("/move_base/status", 10, &war_state_navigation::cb_move_base_status, this);
    sub_amcl_pose = nh.subscribe("/amcl_pose", 5, &war_state_navigation::cb_amcl_pose, this);
    sub_cam_pose = nh.subscribe("/cam_ball_relative_pose", 5, &war_state_navigation::cb_Pose_2D, this);

    //配布するトピックの定義
    pub_goal = nh.advertise<geometry_msgs::PoseStamped>("move_base_simple/goal", 1);
    pub_marker= nh.advertise<visualization_msgs::Marker>("visualization_marker", 1);

    //時間で起動する関数の定義
    timer = nh.createTimer(ros::Duration(1.0), &war_state_navigation::cb_time_control,this);

    //探索時の原点を設定
    origin_position.x = -1.0;
    origin_position.y = 0.0;
    origin_position.theta = 0.0;

    //field_infoの初期化
    for(int i = 0;i<18;i++){
        field_info[i][0] = 0;
        field_info[i][1] = 0;
    }
}



//コールバック関数
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
        //ROS_INFO("red = %d, blue = %d",r_point,b_point);
        //フィールド情報を取得
        int roop_number = 0;
        BOOST_FOREACH (const boost::property_tree::ptree::value_type& child, pt.get_child("targets")) {
            const boost::property_tree::ptree& info = child.second;

            //target.player
            if (boost::optional<std::string> player = info.get_optional<std::string>("player")) {
                //std::cout << "player : " << player.get() << std::endl;
                if(!(player.get().compare("r"))){
                    field_info[roop_number][0] = 1;
                }else if(!(player.get().compare("b"))){
                    field_info[roop_number][0] = -1;
                }else{
                    field_info[roop_number][0] = 0;
                }
                //ROS_INFO("%d,%d,%d",roop_number,field_info[roop_number][0], field_info[roop_number][1]);
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

void war_state_navigation::cb_Pose_2D(const geometry_msgs::Pose2D::ConstPtr &msg){
    //ROS_INFO("cam_topic");
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
        opponent_position.x = amcl_position.x + (msg.get()->x/1000.0) * cos(amcl_position.theta) - (msg.get()->y/1000.0) * sin(amcl_position.theta);
        opponent_position.y = amcl_position.y + (msg.get()->x/1000.0) * sin(amcl_position.theta) + (msg.get()->y/1000.0) * cos(amcl_position.theta);

        ROS_INFO("my x %f y %f", amcl_position.x, amcl_position.y);
        ROS_INFO("data x %f y %f",msg.get()->x,msg.get()->y);
        ROS_INFO("opp x %f y %f",opponent_position.x, opponent_position.y);

        //opponent_position.theta = amcl_position.theta + msg.get()->theta;
        //visual
        marker_control.header.frame_id = "/map";
        marker_control.header.stamp = ros::Time::now();
        marker_control.id = 0;
        marker_control.type = visualization_msgs::Marker::SPHERE;
        marker_control.action = visualization_msgs::Marker::ADD;
        marker_control.pose.position.x = opponent_position.x;
        marker_control.pose.position.y = opponent_position.y;
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

//関数
//1秒周期に実行される制御関数
void war_state_navigation::cb_time_control(const ros::TimerEvent&){
    //navigationのstatus_idが1以外なら自律移動していないと考えて、状態に応じて目標地点、行動を与える
    if(status_id != 1){
        //試合時間が90秒以下なら、誰も取得していない近くのマーカを取得
        if(current_time < 1000.0){
            //誰も取得していない近くのマーカを検索して目標地点に設定
            war_state_navigation::set_near_free_marker_target(amcl_position);
        }
        //180秒以下で、
        //勝っている-相手の位置が分かっている-相手から離れた位置のマーカを取りに行く
        //          -相手の位置が分かっていない-近くのフリーor相手のマーカを取りに行く、マーカを取り終えたら初期位置に戻る
        //負けている-相手の位置が分かっている-相手の近くに目標地点を設定する
        //          -相手の位置が分かっていない-相手の位置を探索する
        else if(current_time < 180.0){
            //一度だけ目標地点履歴を削除
            if(once_flag == 1){
                for(int i = 0;i<18;i++){
                    field_info[i][1] = 0;
                }
                once_flag = -1;
            }

            //勝っている
            if(r_point <= b_point){
                //相手の位置が分かっている
                if(camera_flag == 1){
                    //相手から離れた位置のマーカを取りに行く
                    war_state_navigation::set_far_marker_target(amcl_position, opponent_position);
                }
                //相手の位置が分かっていない
                else{
                    //近くの自分のではないマーカを取りに行く
                    war_state_navigation::set_near_marker_target(amcl_position);
                }
            }
            //負けている
            else{
                //相手の位置が分かっている
                if(camera_flag == 1){
                    //相手の近くに目標地点を設定する
                    war_state_navigation::approach_opponent(opponent_position);
                }
                //相手の位置が分かっていない
                else{
                    //相手の位置を探索する
                    war_state_navigation::serch_opponent_position(amcl_position);
                }
            }
        }
        //180秒以上は試合終了と考えて何もしない
        else{
            //何もしない
        }
    }
    //デバック
    //ROS_INFO("camera_flag = %d",camera_flag);
    for(int i = 0;i<18;i++){
        ROS_INFO("%d %d %d", i, field_info[i][0] ,field_info[i][1]);
    }
}

//マーカを取りに行く
//自己位置推定の結果から一番近くのマーカに目標位置を設定
//フリーなマーカのみ取得
void war_state_navigation::set_near_free_marker_target(geometry_msgs::Pose2D current_my_position){
    ROS_INFO("set_near_free_marker_target");
    //一番近くで自分のマーカではない位置を探索
    double min_distance = 100.0;
    int target_number = -1;
    for(int i=6; i<18; i++){
        //誰も取得していないかつ自分が設定したことないマーカまでの距離を計算する
        if(field_info[i][0] == 0 && field_info[i][1] == 0){
            double distance = sqrt( (pow((current_my_position.x - field_marker_position[i].x),2)) + (pow((current_my_position.y - field_marker_position[i].y), 2) ));
            if(distance < min_distance){
                min_distance = distance;
                target_number = i;
            }
        }
    }

    //target_numberから目標位置を決定
    war_state_navigation::set_position(target_number);
    /*
    //target_numberが-1以外なら目標マーカの設定を行い、目標位置を設定済みにする
    //target_numberが-1なら初期位置に退避
    if(target_number != -1){
        war_state_navigation::navigation_goal_set(field_marker_position[target_number].x, field_marker_position[target_number].y, field_marker_position[target_number].theta);
        field_info[target_number][1] = 1;
    }else{
        war_state_navigation::navigation_goal_set(origin_position.x,origin_position.y,origin_position.theta);
        ROS_INFO("target_number lost");
    }
    */
}

//相手・フリーなマーカを取得
void war_state_navigation::set_near_marker_target(geometry_msgs::Pose2D current_my_position){
    ROS_INFO("set_near_marker_target");
    //一番近くで自分のマーカではない位置を探索
    double min_distance = 100.0;
    int target_number = -1;
    for(int i=6; i<18; i++){
        //相手もしくはフリーかつ自分が設定したことないマーカまでの距離を計算する
        if(field_info[i][0] != 1 && field_info[i][1] == 0){
            double distance = sqrt( (pow((current_my_position.x - field_marker_position[i].x),2)) + (pow((current_my_position.y - field_marker_position[i].y), 2) ));
            if(distance < min_distance){
                min_distance = distance;
                target_number = i;
            }
        }
    }

    //target_numberから目標位置を決定
    war_state_navigation::set_position(target_number);
    /*
    //target_numberが-1以外なら目標マーカの設定を行い、目標位置を設定済みにする
    //target_numberが-1なら初期位置に退避
    if(target_number != -1){
        war_state_navigation::navigation_goal_set(field_marker_position[target_number].x, field_marker_position[target_number].y, field_marker_position[target_number].theta);
        field_info[target_number][1] = 1;
    }else{
        war_state_navigation::navigation_goal_set(origin_position.x,origin_position.y,origin_position.theta);
        //ROS_INFO("target_number lost");
    }
    */
}

//相手から遠い位置のマーカを取得
void war_state_navigation::set_far_marker_target(geometry_msgs::Pose2D current_my_position, geometry_msgs::Pose2D current_opponent_position){
    ROS_INFO("set_far_marker_target");
    //一番近くで自分のマーカではない位置を探索
    double min_distance = 100.0;
    int target_number = -1;
    for(int i=6; i<18; i++){
        //相手もしくはフリーかつ相手からの距離が最も遠いマーカまでの距離を計算する
        if(field_info[i][0] != 1 && field_info[i][1] == 0){
            double distance = sqrt( (pow((current_opponent_position.x - field_marker_position[i].x),2)) + (pow((current_opponent_position.y - field_marker_position[i].y), 2) ));
            if(distance < min_distance){
                min_distance = distance;
                target_number = i;
            }
        }
    }
    //target_numberから目標位置を決定
    war_state_navigation::set_position(target_number);
    /*
    //target_numberが-1以外なら目標マーカの設定を行い、目標位置を設定済みにする
    //target_numberが-1なら初期位置に退避
    if(target_number != -1){
        war_state_navigation::navigation_goal_set(field_marker_position[target_number].x, field_marker_position[target_number].y, field_marker_position[target_number].theta);
        field_info[target_number][1] = 1;
    }else{
        war_state_navigation::navigation_goal_set(origin_position.x,origin_position.y,origin_position.theta);
        //ROS_INFO("target_number lost");
    }
    */
}

//相手に近づく
//角度はランダム
void war_state_navigation::approach_opponent(geometry_msgs::Pose2D current_opponent_position){
    ROS_INFO("approach_opponent");
    //乱数生成器
    static std::mt19937_64 mt64(0);

    // [0, 2pi] の一様分布整数 (int) の分布生成器
    std::uniform_real_distribution<double> get_rand_uni_double(-3.14,3.14);

    war_state_navigation::navigation_goal_set(current_opponent_position.x, current_opponent_position.y, get_rand_uni_double(mt64));
}

//相手を探索
//初期位置から回転して相手の位置を探索する
void war_state_navigation::serch_opponent_position(geometry_msgs::Pose2D current_my_position){
    ROS_INFO("serch_opponent_position");
    //原点からxy座標は+-0.15、thetaは+-0.1以内でなければ初期位置に目標位置を設定する
    /*
    double xy_diff_tolerance = 0.15;
    double theta_diff_tolerance = 0.1;
    if(!(((origin_position.x-xy_diff_tolerance) < current_my_position.x) && (current_my_position.x < (origin_position.x+xy_diff_tolerance)) && ((origin_position.y-xy_diff_tolerance) < current_my_position.y) && (current_my_position.y < (origin_position.y+xy_diff_tolerance)) && ((origin_position.theta-theta_diff_tolerance) < current_my_position.theta) && (current_my_position.theta < (origin_position.theta+theta_diff_tolerance)))){
        war_state_navigation::navigation_goal_set(origin_position.x,origin_position.y,origin_position.theta);
    }
    //位置はそのままでランダムな角度を入れてその場で回転する
    else{
        //乱数生成器
        static std::mt19937_64 mt64(0);

        // [0, 2pi] の一様分布整数 (int) の分布生成器
        std::uniform_real_distribution<double> get_rand_uni_double(-3.14,3.14);
        ROS_INFO("%f",get_rand_uni_double(mt64));
        //乱数で目標角度を決定
        war_state_navigation::navigation_goal_set(origin_position.x,origin_position.y,get_rand_uni_double(mt64));
    }
    */
    //到達するコーナを決める
    //一番近くで自分のマーカではない位置を探索
    double min_distance = 100.0;
    int target_corner = 0;
    for(int i=0; i<4; i++){
        //最も近いコーナを決める
        double distance = sqrt( (pow((current_my_position.x - corner_position[i].x),2)) + (pow((current_my_position.y - corner_position[i].y), 2) ));
        if(distance < min_distance){
            min_distance = distance;
            target_corner = i;
        }
    }
    //姿勢を決める
    //乱数生成器
    static std::mt19937_64 mt64(0);
    // [0, 2pi] の一様分布整数 (int) の分布生成器
    std::uniform_real_distribution<double> get_rand_uni_double(corner_position[target_corner].theta-M_PI/4, corner_position[target_corner].theta+M_PI/4);
    //乱数で目標角度を決定
    war_state_navigation::navigation_goal_set(corner_position[target_corner].x,corner_position[target_corner].y,get_rand_uni_double(mt64));
}

//次の機体の目標位置を設定する
void war_state_navigation::set_position(int target_number){
    //前回のtarget_numberを保持
    static int old_target_number = -1;
    //target_numberが-1以外なら目標マーカの設定を行い、目標位置を設定済みにする
    if(target_number != -1){
        //次の目標値が前回のtarget_numberと違う場合、まずは方向転換を行う
        if(old_target_number != target_number){
            //次の目標マーカがFriedShrimpの場合旋回しない
            if(!(target_number == 14 || target_number == 15 || target_number ==16 || target_number == 17)){
                //TomatoとPuddingの場合、-90度に旋回する
                if(old_target_number == 6 || old_target_number == 7 || old_target_number == 10 || old_target_number == 11){
                    war_state_navigation::navigation_goal_set(field_marker_position[old_target_number].x, field_marker_position[old_target_number].y, -M_PI/2);
                }
                //OmeletteとOctopusWienerの場合、90度に旋回する
                else if(old_target_number == 8 || old_target_number == 9 || old_target_number == 12 || old_target_number == 13){
                    war_state_navigation::navigation_goal_set(field_marker_position[old_target_number].x, field_marker_position[old_target_number].y, M_PI/2);
                }
            }
            //old_taget_numberを更新する
            old_target_number = target_number;
        }
        //前回のtarget_numberと一致すれば、二回目の指定で機体角度もフィールドの内側を向いているので目標マーカへ向かう
        //更に目標地点を設定済みにする
        else{
            war_state_navigation::navigation_goal_set(field_marker_position[target_number].x, field_marker_position[target_number].y, field_marker_position[target_number].theta);
            field_info[target_number][1] = 1;
        }
    }
    //target_numberが-1なら初期位置に退避
    else{
        war_state_navigation::navigation_goal_set(origin_position.x,origin_position.y,origin_position.theta);
        //ROS_INFO("target_number lost");
    }

    /*
    //target_numberが-1以外なら目標マーカの設定を行い、目標位置を設定済みにする
    //target_numberが-1なら初期位置に退避
    if(target_number != -1){
        war_state_navigation::navigation_goal_set(field_marker_position[target_number].x, field_marker_position[target_number].y, field_marker_position[target_number].theta);
        field_info[target_number][1] = 1;
    }else{
        war_state_navigation::navigation_goal_set(origin_position.x,origin_position.y,origin_position.theta);
        //ROS_INFO("target_number lost");
    }
    */
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
