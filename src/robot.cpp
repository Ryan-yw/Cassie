#include <algorithm>
#include <array>
#include <stdlib.h>
#include <string>
#include <bitset>

#include "robot.h"
#include"plan.h"

double input_angle[10] = {0};

//输出参数，模型曲线测试使用
double file_current_leg[6] = { 0 };
double file_current_body[16] = { 0 };
double time_test = 0;

using namespace aris::dynamic;
using namespace aris::plan;
const double PI = aris::PI;
namespace robot
{

auto MoveJoint::prepareNrt()->void
{
    dir_ = doubleParam("direction");

    for(auto &m:motorOptions()) m = aris::plan::Plan::NOT_CHECK_ENABLE;
}
auto MoveJoint::executeRT()->int
{

    static double begin_angle[10];

    if (count() == 1)
    {
        begin_angle[0] = controller()->motionPool()[0].targetPos();
        begin_angle[1] = controller()->motionPool()[1].targetPos();
        begin_angle[2] = controller()->motionPool()[2].targetPos();
        begin_angle[3] = controller()->motionPool()[3].targetPos();
        begin_angle[4] = controller()->motionPool()[4].targetPos();
        begin_angle[5] = controller()->motionPool()[5].targetPos();
        begin_angle[6] = controller()->motionPool()[6].targetPos();
        begin_angle[7] = controller()->motionPool()[7].targetPos();
        begin_angle[8] = controller()->motionPool()[8].targetPos();
        begin_angle[9] = controller()->motionPool()[9].targetPos();

    }

    TCurve s1(0.016,0.3);
    s1.getCurveParam();
    double angle0 = begin_angle[0] + dir_  *  s1.getTCurve(count());
    double angle1 = begin_angle[1] + dir_  *  s1.getTCurve(count());
    double angle2 = begin_angle[2] + dir_  *  s1.getTCurve(count());
    double angle3 = begin_angle[3] + dir_  *  s1.getTCurve(count());
    double angle4 = begin_angle[4] + dir_  *  s1.getTCurve(count());
    double angle5 = begin_angle[5] + dir_  *  s1.getTCurve(count());
    double angle6 = begin_angle[6] + dir_  *  s1.getTCurve(count());
    double angle7 = begin_angle[7] + dir_  *  s1.getTCurve(count());
    double angle8 = begin_angle[8] + dir_  *  s1.getTCurve(count());
    double angle9 = begin_angle[9] + dir_  *  s1.getTCurve(count());


    controller()->motionPool()[0].setTargetPos(angle0);
    controller()->motionPool()[1].setTargetPos(angle1);
    controller()->motionPool()[2].setTargetPos(angle2);
    controller()->motionPool()[3].setTargetPos(angle3);
    controller()->motionPool()[4].setTargetPos(angle4);
    controller()->motionPool()[5].setTargetPos(angle5);
    controller()->motionPool()[6].setTargetPos(angle6);
    controller()->motionPool()[7].setTargetPos(angle7);
    controller()->motionPool()[8].setTargetPos(angle8);
    controller()->motionPool()[9].setTargetPos(angle9);

       return s1.getTc() * 1000-count();
}
auto MoveJoint::collectNrt()->void {}
MoveJoint::MoveJoint(const std::string &name)
{
    aris::core::fromXmlString(command(),
        "<Command name=\"test_mvj\">"
        "		<Param name=\"direction\" default=\"1.0\" abbreviation=\"d\"/>"
        "</Command>");
}

//前进
auto DogForward::prepareNrt()->void
{
    step_ = doubleParam("step");
    for (auto& m : motorOptions()) m = aris::plan::Plan::NOT_CHECK_ENABLE;
}
auto DogForward::executeRT()->int
{

    if (count() == 1)this->master()->logFileRawName("forward");

    int ret = 0;

    //if (count() == 1)
    //{
    //    input_angle[0] = controller()->motionPool()[0].actualPos();
    //    input_angle[1] = controller()->motionPool()[1].actualPos();
    //    input_angle[2] = controller()->motionPool()[2].actualPos();
    //    input_angle[3] = controller()->motionPool()[3].actualPos();
    //    input_angle[4] = controller()->motionPool()[4].actualPos();
    //    input_angle[5] = controller()->motionPool()[5].actualPos();
    //    input_angle[6] = controller()->motionPool()[6].actualPos();
    //    input_angle[7] = controller()->motionPool()[7].actualPos();
    //    input_angle[8] = controller()->motionPool()[8].actualPos();
    //    input_angle[9] = controller()->motionPool()[9].actualPos();
    //}

    TCurve s1(1, 6);
    s1.getCurveParam();
    EllipseTrajectory e1(400, 200, 0, s1);

    //步态规划

        ret = trotPlan(step_, count() - 1, &e1, input_angle);
   

    //输出角度，用于仿真测试
    {
        //输出电机角度
        for (int i = 0; i < 10; ++i)
        {
            lout() << input_angle[i] << "\t";
        }
        time_test += 0.001;
        lout() << time_test << "\t";

        //输出身体和足尖曲线
        for (int i = 0; i < 6; ++i)
        {
            lout() << file_current_leg[i] << "\t";
        }
        lout() << file_current_body[3] << "\t" << file_current_body[7] << "\t" << file_current_body[11] << std::endl;
    }


    //发送电机角度
    //for (int i = 0; i < 12; ++i)
    //{
    //    if (i == 2 || i == 5 || i == 8 || i == 11)
    //        controller()->motionPool()[i].setTargetPos(1.5 * input_angle[i]);
    //    else
    //        controller()->motionPool()[i].setTargetPos(input_angle[i]);
    //}

    return ret;
}
DogForward::DogForward(const std::string& name)
{
    aris::core::fromXmlString(command(),
        "<Command name=\"dog_forward\">"
        "	<Param name=\"step\" default=\"1\" abbreviation=\"n\"/>"
        "</Command>");
}
DogForward::~DogForward() = default;

auto createControllerCassie()->std::unique_ptr<aris::control::Controller>
{
    std::unique_ptr<aris::control::Controller> controller(new aris::control::EthercatController);

    for (aris::Size i = 0; i < 1; ++i)
    {
#ifdef ARIS_USE_ETHERCAT_SIMULATION
        double pos_offset[12]
        {
            0,0,0,0,0,0
        };
#else
        double pos_offset[12]
        {
            //-0.0405786,
            //-0.128355,
            //1.11861,
            //-0.270588,
            //0.429535,
            //1.45398,
            //-0.0726683,
            //0.208502,
            //0.0887512,
            //0.911768,
            //0,
            //0

        };
#endif
        double pos_factor[10]
        {
            131072.0 / 2 / PI, 131072.0  / 2 / PI, 131072.0 / 2 / PI,131072.0  / 2 / PI, 131072.0 / 2 / PI,
            131072.0 / 2 / PI, 131072.0  / 2 / PI, 131072.0 / 2 / PI,131072.0  / 2 / PI, 131072.0 / 2 / PI,
        };
        double max_pos[12]
        {
            PI,PI,PI,PI,PI,
            PI,PI,PI,PI,PI,
        };
        double min_pos[12]
        {
            -PI,-PI,-PI,-PI,-PI,
            -PI,-PI,-PI,-PI,-PI,
        };
        double max_vel[12]
        {
            3300 / 60 * 2 * PI, 330 / 60 * 2 * PI,  330 / 60 * 2 * PI,
            330 / 60 * 2 * PI, 330 / 60 * 2 * PI,  330 / 60 * 2 * PI,
            330 / 60 * 2 * PI, 330 / 60 * 2 * PI,  330 / 60 * 2 * PI,
            330 / 60 * 2 * PI,
        };
        double max_acc[12]
        {
            30000,  3000,  3000,
            3000,  3000,  3000,
            3000,  3000,  3000,
            3000,
        };

        int phy_id[10]={0,1,2,3,4,5,6,7,8,9};


        std::string xml_str =
            "<EthercatMotor phy_id=\"" + std::to_string(phy_id[i]) + "\" product_code=\"0x00\""
            " vendor_id=\"0x00\" revision_num=\"0x00\" dc_assign_activate=\"0x0300\""
            " min_pos=\"" + std::to_string(min_pos[i]) + "\" max_pos=\"" + std::to_string(max_pos[i]) + "\" max_vel=\"" + std::to_string(max_vel[i]) + "\" min_vel=\"" + std::to_string(-max_vel[i]) + "\""
            " max_acc=\"" + std::to_string(max_acc[i]) + "\" min_acc=\"" + std::to_string(-max_acc[i]) + "\" max_pos_following_error=\"10.0\" max_vel_following_error=\"20.0\""
            " home_pos=\"0\" pos_factor=\"" + std::to_string(pos_factor[i]) + "\" pos_offset=\"" + std::to_string(pos_offset[i]) + "\">"
        "	<SyncManagerPoolObject>"
        "		<SyncManager is_tx=\"false\"/>"
        "		<SyncManager is_tx=\"true\"/>"
        "		<SyncManager is_tx=\"false\">"
        "			<Pdo index=\"0x1600\" is_tx=\"false\">"
        "				<PdoEntry name=\"control_word\" index=\"0x6040\" subindex=\"0x00\" size=\"16\"/>"
        "				<PdoEntry name=\"target_pos\" index=\"0x607A\" subindex=\"0x00\" size=\"32\"/>"
        "				<PdoEntry name=\"offset_vel\" index=\"0x60B1\" subindex=\"0x00\" size=\"32\"/>"
        "				<PdoEntry name=\"tor_offset\" index=\"0x60B2\" subindex=\"0x00\" size=\"16\"/>"
        "			</Pdo>"
        "		</SyncManager>"
        "		<SyncManager is_tx=\"true\">"
        "			<Pdo index=\"0x1a00\" is_tx=\"true\">"
        "				<PdoEntry name=\"status_word\" index=\"0x6041\" subindex=\"0x00\" size=\"16\"/>"
        "				<PdoEntry name=\"pos_actual_value\" index=\"0x6064\" subindex=\"0x00\" size=\"32\"/>"
        "				<PdoEntry name=\"following_error\" index=\"0x60F4\" subindex=\"0x00\" size=\"32\"/>"
        "				<PdoEntry name=\"vel_actual_value\" index=\"0x606c\" subindex=\"0x00\" size=\"32\"/>"
        "				<PdoEntry name=\"cur_actual_value\" index=\"0x6077\" subindex=\"0x00\" size=\"16\"/>"
        "			</Pdo>"
        "		</SyncManager>"
        "	</SyncManagerPoolObject>"
            "</EthercatMotor>";

        aris::core::fromXmlString(controller->slavePool().add<aris::control::EthercatMotor>(), xml_str);


#ifdef WIN32
        dynamic_cast<aris::control::EthercatMotor&>(controller->slavePool().back()).setVirtual(true);
#endif

#ifndef WIN32
        std::cout << "before" << std::endl;
        dynamic_cast<aris::control::EthercatMotor&>(controller->slavePool().back()).scanInfoForCurrentSlave();
        //dynamic_cast<aris::control::EthercatMotor&>(controller->slavePool().back()).writeSdo(xml_str);

        std::cout << "end" << std::endl;
#endif

    };
    return controller;
}
auto createPlanCassie()->std::unique_ptr<aris::plan::PlanRoot>
{
    std::unique_ptr<aris::plan::PlanRoot> plan_root(new aris::plan::PlanRoot);

    plan_root->planPool().add<aris::plan::Enable>();
    plan_root->planPool().add<aris::plan::Disable>();
    plan_root->planPool().add<aris::plan::Home>();
    plan_root->planPool().add<aris::plan::Mode>();
    plan_root->planPool().add<aris::plan::Show>();
    plan_root->planPool().add<aris::plan::Sleep>();
    plan_root->planPool().add<aris::plan::Clear>();
    plan_root->planPool().add<aris::plan::Recover>();
    auto &rs = plan_root->planPool().add<aris::plan::Reset>();
    rs.command().findParam("pos")->setDefaultValue("{0.5,0.392523364485981,0.789915966386555,0.5,0.5,0.5}");

    auto &mvaj = plan_root->planPool().add<aris::plan::MoveAbsJ>();
    mvaj.command().findParam("vel")->setDefaultValue("0.1");

    plan_root->planPool().add<aris::plan::MoveL>();
    plan_root->planPool().add<aris::plan::MoveJ>();

    plan_root->planPool().add<aris::plan::GetXml>();
    plan_root->planPool().add<aris::plan::SetXml>();
    plan_root->planPool().add<aris::plan::Start>();
    plan_root->planPool().add<aris::plan::Stop>();



    //自己写的命令
    //plan_root->planPool().add<Enable2>();
     plan_root->planPool().add<DogForward>();
     plan_root->planPool().add<MoveJoint>();
    return plan_root;
}

}
