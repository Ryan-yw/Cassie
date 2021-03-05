#ifndef PLAN_H_
#define PLAN_H_




///功能：生成0->1的梯形曲线。可根据输入的加速度和速度判断曲线为梯形还是三角形
//   ##参数定义##
//  Tc:生成曲线总共需要的时间，由输入的加速度和速度计算
//   v:速度，由用户输入，构造函数初始化
//   a:加速度，由用户输入，构造函数初始化
//  ta:加速段所需的时间，由输入的速度和加速度计算得到
class TCurve
{
private:
	double Tc_;
	double v_;
	double a_;
	double ta_;

public:
	auto getTCurve(int count)->double;
	auto getCurveParam()->void;
	auto getTc()->double { return Tc_; };
	TCurve(double a, double v) { a_ = a; v_ = v; }
	~TCurve() {}
};



///功能：生成椭圆轨迹。在Tc时间内  x方向0->a;y方向0->b->0;z方向0->c
//   ##参数定义##
//   a:x方向步长，由用户输入，构造函数初始化
//   b:y方向步高，由用户输入，构造函数初始化
//   c:z方向步长，由用户输入，构造函数初始化
//   x:x方向在t时刻时的位置
//   y:y方向在t时刻时的位置
//   z:z方向在t时刻时的位置
//   s:梯形曲线
class EllipseTrajectory
{
private:
	double x_;
	double y_;
	double z_;
	double a_;
	double b_;
	double c_;
	TCurve s_;

public:
	auto getEllipseTrajectory(int count)->void;
	auto get_x()->double { return x_; };
	auto get_y()->double { return y_; };
	auto get_z()->double { return z_; };
	auto get_a()->double { return a_; };
	auto get_b()->double { return b_; };
	auto get_c()->double { return c_; };
	auto get_s()->TCurve { return s_; };
	EllipseTrajectory(double a, double b, double c, TCurve& s) :a_(a), b_(b), c_(c), s_(s), x_(0), y_(0), z_(0) {}
	~EllipseTrajectory() {}
};



class Gait
{
public:
	EllipseTrajectory E;


	~Gait() {}
};



///功能：生成机器人绕xyz三个轴旋转的角度轨迹
//   ##参数定义##
// double roll_angle_x_：绕x轴旋转的角度，由用户输入，构造函数初始化
// double yaw_angle_y_：绕y轴旋转的角度，由用户输入，构造函数初始化
// double pitch_angle_z_：绕z轴旋转的角度，由用户输入，构造函数初始化
// double pitch_：在t时刻的俯仰角
// double roll_：在t时刻的横滚角
// double yaw_：在t时刻的偏航角角
// TCurve b_r_s_:梯形曲线
//   s:梯形曲线
class BodyPose
{
private:
	double roll_angle_x_;
	double yaw_angle_y_;
	double pitch_angle_z_;
	double pitch_;
	double roll_;
	double yaw_;
	TCurve b_r_s_;

public:
	auto getBodyRotationTrajectory(int count)->void;
	auto getCurrentRoll()->double { return roll_; };
	auto getCurrentPitch()->double { return pitch_; };
	auto getCurrentYaw()->double { return yaw_; };
	auto getRollTotalAngle()->double { return roll_angle_x_; };
	auto getPitchTotalAngle()->double { return pitch_angle_z_; };
	auto getYawTotalAngle()->double { return yaw_angle_y_; };
	auto getTcurve()->TCurve { return b_r_s_; };
	BodyPose(double roll, double yaw, double pitch, TCurve s) : roll_angle_x_(roll), yaw_angle_y_(yaw), pitch_angle_z_(pitch), b_r_s_(s), pitch_(0), yaw_(0), roll_(0) {}
	~BodyPose() {};
};






/***********************函数变量声明***********************/
auto trotPlan(int n, int count, EllipseTrajectory* param, double* input)->int;
auto walkPlan(int n, int count, EllipseTrajectory* param, double* input)->int;
auto posePlan(int count, EllipseTrajectory* Ellipse, BodyPose* body_pose, double* input)->int;
auto upPlan(int count, EllipseTrajectory* Ellipse, double* input)->int;
auto downPlan(int count, EllipseTrajectory* Ellipse, double* input)->int;
auto downPlanPrepare(int count, EllipseTrajectory* Ellipse, double* input)->int;
#endif



