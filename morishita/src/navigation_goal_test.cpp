#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>

#include <random>

class navigation_goal_test{
    public:
    navigation_goal_test();
    private:
    //10秒おきに適当な目標位置を設定
    void cb_time_goal(const ros::TimerEvent&);
    //ノードハンドラ作成
	ros::NodeHandle nh;

    //使用するpub/sebの定義
    ros::Publisher pub_goal;

    //時間の関数作成
    ros::Timer timer;
};

//コンストラクタ
navigation_goal_test::navigation_goal_test(){
    //配布するトピックの定義
    pub_goal = nh.advertise<geometry_msgs::PoseStamped>("move_base_simple/goal", 1);

    //時間で起動する関数の定義
    timer = nh.createTimer(ros::Duration(5.0), &navigation_goal_test::cb_time_goal,this);


}

//関数定義
void navigation_goal_test::cb_time_goal(const ros::TimerEvent&){

    // 乱数生成器
    static std::mt19937_64 mt64(0);

    // [min_val, max_val] の一様分布整数 (int) の分布生成器
    std::uniform_int_distribution<uint64_t> get_rand_uni_int(-10,10);

    //
    int x = get_rand_uni_int(mt64);
    int y = get_rand_uni_int(mt64);

    ROS_INFO("x=%d y=%d",x,y);
}


int main(int argc, char** argv)
{
	ros::init(argc, argv, "navigation_goal_test");
	navigation_goal_test navigation_goal_test;
	ros::spin();
	return 0;
}
