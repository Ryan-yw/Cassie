
#include"plan.h"
#include<cmath>
#include<iostream>
#include"kinematics.h"
using namespace std;


//���߲�����̬����
static double foot_position_start_point[6] = {
										 -kBodyLong+65, -kBodyHigh, -kBodyWidth / 2,  //left
										 -kBodyLong+65, -kBodyHigh,  kBodyWidth / 2,  //right

};
static double body_pisiton_start_point[16] = { 1,0,0,0,
											   0,1,0,0,
											   0,0,1,0,
											   0,0,0,1 };



extern double file_current_leg[6];
extern double file_current_body[16];
//---------------------------------------------------��������-------------------------------------------//
//������������0->1
//���룺ʱ�䣬ÿ�������һ��
//�������ǰʱ��s��ֵ
auto TCurve::getTCurve(int count)->double
{
	//double ta = p.ta_;
	//double a = p.a_;
	//double v = p.v_;
	//double T_c = p.Tc_;
	int t = count + 1;
	double s = 0;

	if (2 * ta_ == Tc_)   //����������
	{
		if (t < ta_ * 1000)
		{
			s = 0.5 * a_ * t * t / 1000.0 / 1000.0;
		}
		else
		{
			s = 0.5 * a_ * ta_ * ta_ + 0.5 * (t / 1000.0 - ta_) * (2 * v_ - a_ * (t / 1000.0 - ta_));
		}
	}
	else    //��������
	{
		if (t < ta_ * 1000)
		{
			s = 0.5 * a_ * t * t / 1000.0 / 1000.0;
		}
		else if (t >= ta_ * 1000 && t < (Tc_ * 1000 - ta_ * 1000))
		{
			s = v_ * t / 1000 - v_ * v_ / 2.0 / a_;
		}
		else
		{
			s = (2 * a_ * v_ * Tc_ - 2 * v_ * v_ - a_ * a_ * (t / 1000.0 - Tc_) * (t / 1000.0 - Tc_)) / (2 * a_);
		}
	}
	//std::cout << s << std::endl;
	return s;
}

//�����������ߵĲ������ɳ�Ա������ʼ������Ӧ��������ɹ��캯����ʼ��
auto TCurve::getCurveParam()->void
{
	if (v_ * v_ / a_ <= 1)
	{
		this->Tc_ = (a_ + v_ * v_) / v_ / a_;
		this->a_ = a_;
		this->v_ = v_;
	}
	else
	{
		//���ٶȼ��㣬��ʱ�����ļ��ٶȲ�������
		this->Tc_ = 2.0 / v_;
		this->a_ = v_ * v_;
		this->v_ = v_;
	}
	this->ta_ = v_ / a_;
}



//-----------------------------------------------------��Բ�켣--------------------------------------------------------//

//������Բ�켣����Tcʱ����  x����0->a;y����0->b->0;z����0->c����Ӧ��������ɹ��캯����ʼ����
auto EllipseTrajectory::getEllipseTrajectory(int count)->void
{
	x_ = a_ * (1 + cos(PI - PI * s_.getTCurve(count))) / 2.0;
	y_ = b_ * sin(PI - PI * s_.getTCurve(count));
	z_ = c_ * (1 + cos(PI - PI * s_.getTCurve(count))) / 2.0;
	//std::cout << y << std::endl;
}



 
//------------------------------------------------�ź�����λ�ú���̬�Ĺ滮-------------------------------------------------//

//�Խǲ�̬����ڵѿ����ռ��µ�����滮
//��ǰ�ŵ�λ�� = ��һ���ŵ�λ�� + ��λ������
//ÿ��������һ���������ߣ��ŵ�λ�ø���һ��
//ע�⣺#�ŵ�λ���ڳ�ʼʱ��Ҫ��ʼ��һ�£�����һ��ʼֻ�н�13����24�ŵĳ�ʼλ��ʱ0
//#ע�⣺Ŀǰֻ������ƽ������
auto planLegTrot(int e_1, int n, double* current_leg, int count, EllipseTrajectory* Ellipse)->void
{
	if (count == 0)//��ʼ���ŵ�λ�ã�����24�ų�ʼλ��Ϊ0
	{
		for (int i = 0; i < 6; i++)
		{
			current_leg[i] = foot_position_start_point[i];
		}
	}

	Ellipse->getEllipseTrajectory(count);

	if (e_1 % 2 == 0)  //ż��13���ȣ�24ͣ
	{
		if (e_1 == 0)   //���ٶ�
		{
			//�滮leg1
			current_leg[0] = foot_position_start_point[0] + Ellipse->get_x() / 2;
			current_leg[1] = foot_position_start_point[1] + Ellipse->get_y();
			current_leg[2] = foot_position_start_point[2] + Ellipse->get_z() / 2;

		}
		else
		{
			//�滮leg1
			current_leg[0] = foot_position_start_point[0] + Ellipse->get_x();
			current_leg[1] = foot_position_start_point[1] + Ellipse->get_y();
			current_leg[2] = foot_position_start_point[2] + Ellipse->get_z();

		}
	}
	else if (e_1 % 2 == 1)  //����24����13ͣ
	{
		if (e_1 == (2 * n - 1))//���ٶ�
		{
			//�滮leg4
			current_leg[3] = foot_position_start_point[3] + Ellipse->get_x() / 2;
			current_leg[4] = foot_position_start_point[4] + Ellipse->get_y();
			current_leg[5] = foot_position_start_point[5] + Ellipse->get_z() / 2;
		}
		else
		{
			//�滮leg4
			current_leg[3] = foot_position_start_point[3] + Ellipse->get_x();
			current_leg[4] = foot_position_start_point[4] + Ellipse->get_y();
			current_leg[5] = foot_position_start_point[5] + Ellipse->get_z();
		}
	}

	if (count + 1 == floor(Ellipse->get_s().getTc() * 1000))
	{
		for (int i = 0; i < 6; i++)
		{
			foot_position_start_point[i] = current_leg[i];
		}
	}
}



//���������ڹ滮����������ڶԽǲ�̬�������λ�ù켣��������ת����̬�任��
//��ǰ�����λ�� = ��һ�������λ�� + ����λ������
//ÿ������һ�������ǣ������λ�ø���һ��
//#ע�⣺Ŀǰֻ������ƽ������
auto planBodyTransformTrot(int e_1, int n, double* current_body, int count, EllipseTrajectory* Ellipse)->void
{
	double a = 100;
	int per_step_count = Ellipse->get_s().getTc() * 1000;
	if (count == 0) //���ã�����ɾ�������㲻���Ƕ�
	{
		for (int i = 0; i < 16; i++)
		{
			current_body[i] = body_pisiton_start_point[i];
		}
	}

	if (e_1 == 0)   //���ٶ�
	{
		//�滮����
		current_body[3] = body_pisiton_start_point[3] + Ellipse->get_a() * count * count / (4.0 * per_step_count * per_step_count);
		current_body[7] = body_pisiton_start_point[7];
		current_body[11] = body_pisiton_start_point[11] + Ellipse->get_s().getTCurve(count % per_step_count) * a;
	}
	else if (e_1 == (2 * n - 1))//���ٶ�
	{
		//�滮����
		int t = (2 * n - 1) * per_step_count + per_step_count;
		current_body[3] = body_pisiton_start_point[3] + 0 - Ellipse->get_a() * (count - t) * (count - t) / (4.0 * per_step_count * per_step_count) + Ellipse->get_a() * n - Ellipse->get_a() / 2.0;//n * a
		current_body[7] = body_pisiton_start_point[7];
		current_body[11] = body_pisiton_start_point[11] - Ellipse->get_s().getTCurve(count % per_step_count) * a;
	}
	else //���ٶ�
	{
		//�滮����
		current_body[3] = body_pisiton_start_point[3] + Ellipse->get_a() / 4.0 + Ellipse->get_a() * (count - per_step_count) / per_step_count / 2;//�ٶ�Ϊ100mm/s  ÿ�����per_step_count
		current_body[7] = body_pisiton_start_point[7];
		if (e_1 % 2 == 0)
		{
			current_body[11] = body_pisiton_start_point[11] + Ellipse->get_s().getTCurve(count % per_step_count) * a*2;
		}
		else
		{
			current_body[11] = body_pisiton_start_point[11] - Ellipse->get_s().getTCurve(count % per_step_count) * a*2;
		}
	}


	//���һ���������ߺ��һ������ҡ�ڵı仯ֵ
	if (count % per_step_count == floor(Ellipse->get_s().getTc() * 1000) - 1)
	{
		body_pisiton_start_point[11] = current_body[11];
	}

	if (count + 1 >= 2 * n * per_step_count)
	{
		for (int i = 0; i < 16; i++)
		{
			body_pisiton_start_point[i] = current_body[i];
		}
	}
}



/////////////////////////////////////////////////////////////////��̬�滮///////////////////////////////////////////////

//���º�����robot.cpp�б�����


//���������߶Խǲ�̬������ԭ��̤����ǰ�������ˡ����ơ����ơ�
//���в������ߺͲ��������û����롣��һ����ʱ�䣨�����߿����������û������������ߵ��ٶȺͼ��ٶ�ȷ��
//#ע�⣺��������ٶȺͼ��ٶȻ�û����
auto trotPlan(int n, int count, EllipseTrajectory* Ellipse, double* input)->int
{

	int per_step_count = Ellipse->get_s().getTc() * 1000;

	static double current_leg_in_ground[12] = { 0 };
	static double current_body_in_ground[16] = { 0 };


	//�ж�����״̬
	int e_1 = count / per_step_count;  //�жϵ�ǰ������һ��,����һ��e1��1

	//�滮��
	planLegTrot(e_1, n, current_leg_in_ground, count % per_step_count, Ellipse);
	//�滮����
	planBodyTransformTrot(e_1, n, current_body_in_ground, count, Ellipse);

	//ģ�Ͳ���ʹ��
	for (int j = 0; j < 6; ++j)
	{
		file_current_leg[j] = current_leg_in_ground[j];
	}
	for (int j = 0; j < 16; ++j)
	{
		file_current_body[j] = current_body_in_ground[j];
	}
	//ģ�Ͳ���ʹ��
	inverse(current_leg_in_ground, current_body_in_ground, input);

	return 2 * n * per_step_count - count - 1;
}




//-----------------------------------����ͽŷֿ���---------------------------------------------//
//----------------------------------walk��̬�ȶ�����+���������ƶ��滮-------------------------------------//
//�������ͬʱ�˶�
auto walkPlan(int n, int count, EllipseTrajectory* Ellipse, TCurve* s1, double* input, double* body_cm)->int
{
	int per_step_count = Ellipse->get_s().getTc() * 1000;

	static double current_leg_in_ground[12] = { 0 };
	static double current_body_in_ground[16] = { 0 };

	int e_1 = count / per_step_count;  //�жϵ�ǰ������һ��,����һ��e1��1
	int e_2 = count % per_step_count;  //0-Tc 
	if (count == 0)
	{
		for (auto i = 0; i < 12; i++)
			current_leg_in_ground[i] = foot_position_start_point[i];
		for (auto i = 0; i < 16; i++)
			current_body_in_ground[i] = body_pisiton_start_point[i];
	}

	Ellipse->getEllipseTrajectory(e_2);
	//�ж�����״̬
	if ((e_1 + 1) % 8 == 1)  //�����ƶ�
	{
		if (e_2 == 0)
		{
			for (int i = 0; i < 16; i++)
			{
				current_body_in_ground[i] = body_pisiton_start_point[i];
			}
		}
		current_body_in_ground[3] = body_pisiton_start_point[3] + body_cm[0] * s1->getTCurve(e_2);
		current_body_in_ground[7] = body_pisiton_start_point[7] + body_cm[1] * s1->getTCurve(e_2);
		current_body_in_ground[11] = body_pisiton_start_point[11] + body_cm[2] * s1->getTCurve(e_2);

		if (e_2 == floor(Ellipse->get_s().getTc() * 1000) - 1)
		{
			for (int i = 0; i < 16; i++)
			{
				body_pisiton_start_point[i] = current_body_in_ground[i];
			}
		}
		//std::cout << "111111" << "\t" << count << std::endl;
	}
	else if ((e_1 + 1) % 8 == 2)  //��3��
	{
		if (e_2 == 0)//ÿ���ߵ�ʱ���ʼ���ŵ�λ��
		{
			for (int i = 0; i < 12; i++)
			{
				current_leg_in_ground[i] = foot_position_start_point[i];
			}
		}

		current_leg_in_ground[6] = foot_position_start_point[6] + Ellipse->get_x();
		current_leg_in_ground[7] = foot_position_start_point[7] + Ellipse->get_y();
		current_leg_in_ground[8] = foot_position_start_point[8] + Ellipse->get_z();

		if (e_2 == floor(Ellipse->get_s().getTc() * 1000) - 1)
		{
			for (int i = 0; i < 12; i++)
			{
				foot_position_start_point[i] = current_leg_in_ground[i];
			}
		}
		//std::cout << "222222" << "\t" << count << std::endl;
	}
	else if ((e_1 + 1) % 8 == 3)  //�����ƶ�
	{
		if (e_2 == 0) //���ã�����ɾ�������㲻���Ƕ�
		{
			for (int i = 0; i < 16; i++)
			{
				current_body_in_ground[i] = body_pisiton_start_point[i];
			}
		}
		current_body_in_ground[3] = body_pisiton_start_point[3] + body_cm[3] * s1->getTCurve(e_2);
		current_body_in_ground[7] = body_pisiton_start_point[7] + body_cm[4] * s1->getTCurve(e_2);
		current_body_in_ground[11] = body_pisiton_start_point[11] + body_cm[5] * s1->getTCurve(e_2);
		if (e_2 == floor(Ellipse->get_s().getTc() * 1000) - 1)
		{
			for (int i = 0; i < 16; i++)
			{
				body_pisiton_start_point[i] = current_body_in_ground[i];
			}
		}
		//std::cout << "333333" << "\t" << count << std::endl;
	}
	else if ((e_1 + 1) % 8 == 4)  //��4��
	{
		if (e_2 == 0)//ÿ���ߵ�ʱ���ʼ���ŵ�λ��
		{
			for (int i = 0; i < 12; i++)
			{
				current_leg_in_ground[i] = foot_position_start_point[i];
			}
		}

		current_leg_in_ground[9] = foot_position_start_point[9] + Ellipse->get_x();
		current_leg_in_ground[10] = foot_position_start_point[10] + Ellipse->get_y();
		current_leg_in_ground[11] = foot_position_start_point[11] + Ellipse->get_z();
		if (e_2 == floor(Ellipse->get_s().getTc() * 1000) - 1)
		{
			for (int i = 0; i < 12; i++)
			{
				foot_position_start_point[i] = current_leg_in_ground[i];
			}
		}
		//std::cout << "444444" << "\t" << count << std::endl;
	}
	else if ((e_1 + 1) % 8 == 5)  //�����ƶ�
	{
		if (e_2 == 0) //���ã�����ɾ�������㲻���Ƕ�
		{
			for (int i = 0; i < 16; i++)
			{
				current_body_in_ground[i] = body_pisiton_start_point[i];
			}
		}
		current_body_in_ground[3] = body_pisiton_start_point[3] + body_cm[6] * s1->getTCurve(e_2);
		current_body_in_ground[7] = body_pisiton_start_point[7] + body_cm[7] * s1->getTCurve(e_2);
		current_body_in_ground[11] = body_pisiton_start_point[11] + body_cm[8] * s1->getTCurve(e_2);
		if (e_2 == floor(Ellipse->get_s().getTc() * 1000) - 1)
		{
			for (int i = 0; i < 16; i++)
			{
				body_pisiton_start_point[i] = current_body_in_ground[i];
			}
		}
		//std::cout << "555555" << "\t" << count << std::endl;
	}
	else if ((e_1 + 1) % 8 == 6)  //��2��
	{
		if (e_2 == 0)//ÿ���ߵ�ʱ���ʼ���ŵ�λ��
		{
			for (int i = 0; i < 12; i++)
			{
				current_leg_in_ground[i] = foot_position_start_point[i];
			}
		}

		current_leg_in_ground[3] = foot_position_start_point[3] + Ellipse->get_x();
		current_leg_in_ground[4] = foot_position_start_point[4] + Ellipse->get_y();
		current_leg_in_ground[5] = foot_position_start_point[5] + Ellipse->get_z();
		if (e_2 == floor(Ellipse->get_s().getTc() * 1000) - 1)
		{
			for (int i = 0; i < 12; i++)
			{
				foot_position_start_point[i] = current_leg_in_ground[i];
			}
		}
		//std::cout << "666666" << "\t" << count << std::endl;
	}
	else if ((e_1 + 1) % 8 == 7)  //�����ƶ�
	{
		if (e_2 == 0) //���ã�����ɾ�������㲻���Ƕ�
		{
			for (int i = 0; i < 16; i++)
			{
				current_body_in_ground[i] = body_pisiton_start_point[i];
			}
		}
		current_body_in_ground[3] = body_pisiton_start_point[3] + body_cm[9] * s1->getTCurve(e_2);
		current_body_in_ground[7] = body_pisiton_start_point[7] + body_cm[10] * s1->getTCurve(e_2);
		current_body_in_ground[11] = body_pisiton_start_point[11] + body_cm[11] * s1->getTCurve(e_2);
		if (e_2 == floor(Ellipse->get_s().getTc() * 1000) - 1)
		{
			for (int i = 0; i < 16; i++)
			{
				body_pisiton_start_point[i] = current_body_in_ground[i];
			}
		}
		//std::cout << "777777" << "\t" << count << std::endl;
	}
	else //��1��
	{
		if (e_2 == 0)//ÿ���ߵ�ʱ���ʼ���ŵ�λ��
		{
			for (int i = 0; i < 12; i++)
			{
				current_leg_in_ground[i] = foot_position_start_point[i];
			}
		}

		current_leg_in_ground[0] = foot_position_start_point[0] + Ellipse->get_x();
		current_leg_in_ground[1] = foot_position_start_point[1] + Ellipse->get_y();
		current_leg_in_ground[2] = foot_position_start_point[2] + Ellipse->get_z();
		if (e_2 == floor(Ellipse->get_s().getTc() * 1000) - 1)
		{
			for (int i = 0; i < 12; i++)
			{
				foot_position_start_point[i] = current_leg_in_ground[i];
			}
		}
		//std::cout << "888888" << "\t" << count << std::endl;
	}

	////�滮��
	//planLegWalk(e_1, n, current_leg_in_ground, count % per_step_count, Ellipse);
	////�滮����
	//planBodyTransformWalk(e_1, n, current_body_in_ground, count, Ellipse);



	//ģ�Ͳ���ʹ��
	for (int j = 0; j < 12; j++)
	{
		file_current_leg[j] = current_leg_in_ground[j];
	}
	for (int j = 0; j < 12; j++)
	{
		file_current_body[j] = current_body_in_ground[j];
	}
	//ģ�Ͳ���ʹ��
	inverse(current_leg_in_ground, current_body_in_ground, input);

	return 8 * n * per_step_count - count - 1;
}




