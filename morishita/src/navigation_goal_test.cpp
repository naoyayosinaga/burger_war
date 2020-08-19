#include <ros/ros.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/PoseStamped.h>
#include <tf/transform_broadcaster.h>
#include <random>

#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib_msgs/GoalStatusArray.h>
#include <actionlib/client/simple_action_client.h>

class navigation_goal_test{
    public:
    navigation_goal_test();
    private:
    //関数
    //10秒おきに適当な目標位置を設定
    void cb_time_goal(const ros::TimerEvent&);
    void navStatusCallBack(const actionlib_msgs::GoalStatusArray::ConstPtr &status);
    //ノードハンドラ作成
	ros::NodeHandle nh;

    //使用するpub/sebの定義
    ros::Publisher pub_goal;
    ros::Subscriber sub_navi_status;

    //時間の関数作成
    ros::Timer timer;
};

//コンストラクタ
navigation_goal_test::navigation_goal_test(){
    //購読するトピックの定義
    sub_navi_status = nh.subscribe<actionlib_msgs::GoalStatusArray>("/move_base/status", 10, &navigation_goal_test::navStatusCallBack, this);
    //配布するトピックの定義
    pub_goal = nh.advertise<geometry_msgs::PoseStamped>("move_base_simple/goal", 1);

    //時間で起動する関数の定義
    timer = nh.createTimer(ros::Duration(10.0), &navigation_goal_test::cb_time_goal,this);
}

//関数定義
void navigation_goal_test::cb_time_goal(const ros::TimerEvent&){

    // 乱数生成器
    static std::mt19937_64 mt64(0);

    // [min_val, max_val] の一様分布整数 (int) の分布生成器
    std::uniform_real_distribution<double> get_rand_uni_double(-1,1);

    //乱数で目標位置を決定
    double x = get_rand_uni_double(mt64);
    double y = get_rand_uni_double(mt64);

    ROS_INFO("x=%f y=%f",x,y);

    geometry_msgs::PoseStamped goal;
    goal.pose.position.x = x;
    goal.pose.position.y = y;

    //クオータニオン表記に対応
    tf::Quaternion quat=tf::createQuaternionFromRPY(0,0,0);
    geometry_msgs::Quaternion geometry_quat;
    quaternionTFToMsg(quat, geometry_quat);

    goal.pose.orientation = geometry_quat;

    goal.header.frame_id = "map";

    pub_goal.publish(goal);
}

void navigation_goal_test::navStatusCallBack(const actionlib_msgs::GoalStatusArray::ConstPtr &status)
{
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

    if (!status->status_list.empty()){
    actionlib_msgs::GoalStatus goalStatus = status->status_list.back();
    status_id = goalStatus.status;
    }
    if(status_id == 0){
        ROS_INFO("PENDING");
    }else if(status_id==1){
        //移動中
        ROS_INFO("ACTIVE");
    }else if (status_id == 2){
        ROS_INFO("PREEMPTED");
    }else if(status_id == 3){
        ROS_INFO("SUCCEEDED");
    }else{
        ROS_INFO("other");
    }
}



int main(int argc, char** argv)
{
	ros::init(argc, argv, "navigation_goal_test");
	navigation_goal_test navigation_goal_test;
	ros::spin();
	return 0;
}
