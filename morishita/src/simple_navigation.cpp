#include <ros/ros.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/PoseStamped.h>
#include <tf/transform_broadcaster.h>
#include <random>

#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib_msgs/GoalStatusArray.h>
#include <actionlib/client/simple_action_client.h>

class simple_navigation{
    public:
    simple_navigation();
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

    int status_id = 0;
    int old_status_id = 0;
    int target_number = -1;
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
};

//コンストラクタ
simple_navigation::simple_navigation(){
    //購読するトピックの定義
    sub_navi_status = nh.subscribe<actionlib_msgs::GoalStatusArray>("/move_base/status", 10, &simple_navigation::navStatusCallBack, this);
    //配布するトピックの定義
    pub_goal = nh.advertise<geometry_msgs::PoseStamped>("move_base_simple/goal", 1);
}

//関数定義
void simple_navigation::cb_time_goal(const ros::TimerEvent&){

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

void simple_navigation::navStatusCallBack(const actionlib_msgs::GoalStatusArray::ConstPtr &status)
{
    geometry_msgs::PoseStamped goal;

    if (!status->status_list.empty()){
        actionlib_msgs::GoalStatus goalStatus = status->status_list.back();
        status_id = goalStatus.status;
    }else{
        //最初の目的地を設定
        target_number = 0;
        //xy座標
        goal.pose.position.x = -0.5;
        goal.pose.position.y = 0;
        //yaw軸
        //クオータニオン表記に対応
        tf::Quaternion quat=tf::createQuaternionFromRPY(0,0,0);
        geometry_msgs::Quaternion geometry_quat;
        quaternionTFToMsg(quat, geometry_quat);
        goal.pose.orientation = geometry_quat;
        goal.header.frame_id = "map";
        pub_goal.publish(goal);
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

    if(target_number == 0 && old_status_id == 1 && status_id == 3){
        target_number = 1;
        goal.pose.position.x = 0.1;
        goal.pose.position.y = -0.5;
        //yaw軸
        //クオータニオン表記に対応
        tf::Quaternion quat=tf::createQuaternionFromRPY(0,0,0);
        geometry_msgs::Quaternion geometry_quat;
        quaternionTFToMsg(quat, geometry_quat);
        goal.pose.orientation = geometry_quat;
        goal.header.frame_id = "map";
        pub_goal.publish(goal);
    }else if(target_number == 1 && old_status_id == 1 && status_id == 3){
        target_number = 2;
        goal.pose.position.x = 0.1;
        goal.pose.position.y = 0.5;
        //yaw軸
        //クオータニオン表記に対応
        tf::Quaternion quat=tf::createQuaternionFromRPY(0,0,0);
        geometry_msgs::Quaternion geometry_quat;
        quaternionTFToMsg(quat, geometry_quat);
        goal.pose.orientation = geometry_quat;
        goal.header.frame_id = "map";
        pub_goal.publish(goal);
    }else if(target_number == 2 && old_status_id == 1 && status_id == 3){
        target_number = 3;
        goal.pose.position.x = -0.1;
        goal.pose.position.y = 0.5;
        //yaw軸
        //クオータニオン表記に対応
        tf::Quaternion quat=tf::createQuaternionFromRPY(0,0,3.14);
        geometry_msgs::Quaternion geometry_quat;
        quaternionTFToMsg(quat, geometry_quat);
        goal.pose.orientation = geometry_quat;
        goal.header.frame_id = "map";
        pub_goal.publish(goal);
    }else if(target_number == 3 && old_status_id == 1 && status_id == 3){
        target_number = 4;
        goal.pose.position.x = -0.1;
        goal.pose.position.y = -0.5;
        //yaw軸
        //クオータニオン表記に対応
        tf::Quaternion quat=tf::createQuaternionFromRPY(0,0,3.14);
        geometry_msgs::Quaternion geometry_quat;
        quaternionTFToMsg(quat, geometry_quat);
        goal.pose.orientation = geometry_quat;
        goal.header.frame_id = "map";
        pub_goal.publish(goal);
    }



    //次の周期のために値を保持
    old_status_id = status_id;
}



int main(int argc, char** argv)
{
	ros::init(argc, argv, "simple_navigation");
	simple_navigation simple_navigation;
	ros::spin();
	return 0;
}
