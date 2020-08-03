#include <ros/ros.h>
#include "std_msgs/String.h"

//json読み込み用ライブラリ
#include <boost/property_tree/ptree.hpp>
#include <boost/property_tree/json_parser.hpp>
#include <boost/foreach.hpp>
#include <boost/optional.hpp>

#include <iostream>
#include <sstream>

class war_state_test{
    public:
    war_state_test();
    private:
    void cb_string(const std_msgs::String::ConstPtr &msg);

    //ノードハンドラ作成
	ros::NodeHandle nh;

    //使用するpub/sebの定義
    ros::Subscriber sub_string;
};

war_state_test::war_state_test(){
    //購読するトピックの定義
    sub_string=nh.subscribe("/war_state", 5, &war_state_test::cb_string, this);
}

//関数定義
void war_state_test::cb_string(const std_msgs::String::ConstPtr &msg){
    //データ受信
    std_msgs::String data = *msg;

    std::stringstream ss;
    ss << data.data;
    boost::property_tree::ptree pt;
    
    read_json(ss, pt);

    //scores.b
    if (boost::optional<int> value = pt.get_optional<int>("scores.b")) {
        std::cout << " scores.b: " << value.get() << std::endl;
    }
    else {
        std::cout << "value is nothing" << std::endl;
    }

    //time
    if (boost::optional<double> value = pt.get_optional<double>("time")) {
        std::cout << " time: " << value.get() << std::endl;
    }
    else {
        std::cout << "value is nothing" << std::endl;
    }

    //players.r
    if (boost::optional<std::string> str = pt.get_optional<std::string>("players.r")) {
        std::cout << "str : " << str.get() << std::endl;
    }
    else {
        std::cout << "str is nothing" << std::endl;
    }

    //ROS_INFO("%s",data.data.c_str());
}

int main(int argc, char** argv)
{
	ros::init(argc, argv, "war_state_test");
	war_state_test war_state_test;
	ros::spin();
	return 0;
}
