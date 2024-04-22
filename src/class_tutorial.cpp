#include <ros/ros.h>

class TestClass
{
private:
	double position_x_ = 0;
	void __add_num(double add);
public:
	double position_x_public = 0;
	TestClass(double x = 0);
	~TestClass();
    
	void print_num(double num = 0);
};

TestClass::TestClass(double x)
{
	ROS_INFO("init TestClass");
	position_x_ = x;
}

TestClass::~TestClass()
{
	ROS_INFO("delete TestClass");
}

void TestClass::__add_num(double add)
{
	position_x_ += add;
}
void TestClass::print_num(double num)
{
	ROS_INFO("num : %f", num);
	ROS_INFO("x : %f", position_x_);
	ROS_INFO("x public : %f", position_x_public);
}


//(ここから実際に使用するプログラム----------------------------------------------------------------------------------------------------------
int main(int argc,char **argv){
	ros::init(argc,argv,"class_tutorial");

	TestClass tc(4);
	tc.position_x_public = 2.3;
	tc.print_num(4.7);
	delete &tc;


	return 0;
}