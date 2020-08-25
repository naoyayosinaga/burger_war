#include <ros/ros.h>

//Obstacle_detector
#include <obstacle_detector/Obstacles.h>


class opponent_detector{
    public:
    opponent_detector();
    private:


    //ノードハンドラ作成
	ros::NodeHandle nh;

    //使用するpub/sebの定義
    ros::Subscriber sub_string;
};



int main(int argc, char** argv)
{
	ros::init(argc, argv, "opponent_detector");
	opponent_detector opponent_detector;
	ros::spin();
	return 0;
}
