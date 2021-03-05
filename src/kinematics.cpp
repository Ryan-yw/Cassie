#include"kinematics.h"
#include<cmath>
#include<iostream>
#include<fstream>
#include<aris.hpp>

extern double input_angle[10];


//身体在腿坐标系下的变换矩阵
double PL[16] =
{
     1, 0,  0,  kBodyLong,
     0, 1,  0,  0           ,
     0, 0,  1,  kBodyWidth / 2,
     0, 0,  0,  1
};

double PR[16] =
{
     1, 0,  0,  kBodyLong,
     0, 1,  0,  0           ,
     0, 0,  1,  -kBodyWidth / 2,
     0, 0,  0,  1
};


//运动学反解



void inverseLeg(double x, double y, double z, double a, double b, double c, double l,double * pos) 
{

    //-----已知数据-----
    //L、l为长
    //v为向量
    //a为角度
    const double kLAO = 154.5;
    const double kLAD = 150;
    const double kLCD = 422.02;
    const double kLAB = 487.46;
    const double kLBC = 88.82;
    const double kLBE = 509.51;
    const double kLCE = 426.61;
    const double kLEG = 326.82;
    const double kLHI = 280;
    const double kLGH = 120;
    const double kLIJ = 85;
    const double kLEJ = 40;
    const double kLEI = std::sqrt(kLIJ * kLIJ + kLEJ * kLEJ);
    const double kQ30 = 2.768114255522946;
    const double kQ50 = 1.020376369330288;

    double v_om[3] =
    {
        x, y, z
    };
    double v_vn[3] =
    {
        a, b, c
    };

    //-----计算数据-----
    double q1, q2, q3, q4, q5;
    double test = 3;//判断左右，0左1右
    double x0, y0, x1, y1;
    double theta1, theta2, theta3, theta4;
    double a_nop, a_mpn, a_lim, a_eij, a_eik, a_cbe, a_abe, a_abc, a_bac, a_cad, a_bad, a_bae, a_eaf,
        a_daf, a_adc, a_aec, a_aeb, a_aef, a_bec, a_gei, a_gei_b, a_gei_s, a_hgi, a_egi, a_egh;
    double l_np, l_qs, l_rs, l_or, l_os, l_lm, l_il, l_ik, l_ek, l_ae, l_af, l_ef, l_ac, l_gi;

    double l_im = l;
    double l_mn = std::abs(x);
    double l_no = std::sqrt(y * y + z * z);
    double l_mo = std::sqrt(x * x + y * y + z * z);



    //-----求q1-----
    //求末端和向量n所在平面与yoz平面的交线的方程
    double v_m[3] =
    {
        y * c - z * b, z * a - x * c, x * b - y * a
    };

    double x_c = v_m[0];
    double y_c = v_m[1];
    double z_c = v_m[2];

    //求q1
    double k = -z_c / y_c;
    theta1 = std::atan(k);

    if (theta1 > 0 && theta1 < PI / 2)
    {
        q1 = PI / 2 - theta1;
    }
    else if (theta1 == 0)
    {
        if (z > 0)
        {
            q1 = -PI / 2;
        }
        else if (z < 0)
        {
            q1 = PI / 2;
        }
    }
    else if (theta1 > -PI / 2 && theta1 < 0)
    {
        q1 = -theta1 - PI / 2;
    }
    else if (theta1 == PI / 2 || theta1 == -PI / 2)
    {
        q1 = 0;
    }

    //-----q2-----
    //判断末端在交线的左边，还是右边
    theta2 = std::acos(std::abs(z) / l_no);
    if (theta1 > 0 && theta1 < PI / 2)
    {
        if (y - k * z > 0)
        {
            test = 1;  // 右边
        }
        else if (y - k * z < 0)
        {
            test = 0;//左边
        }
        else if (y - k * z == 0)
        {
            q2 = 0;
        }
    }
    else if (theta1 == 0)
    {
        if (q1 == PI / 2)
        {
            if (y > 0)
            {
                test = 1;
            }
            else if (y < 0)
            {
                test = 0;
            }
            else if (y == 0)
            {
                q2 = 0;
            }
        }
        else if (q1 == -PI / 2)
        {
            if (y > 0)
            {
                test = 0;
            }
            else if (y < 0)
            {
                test = 1;
            }
            else if (y == 0)
            {
                q2 = 0;
            }
        }
    }
    else if (theta1 > -PI / 2 && theta1 < 0)
    {
        if (y - k * z > 0)
        {
            test = 0;
        }
        else if (y - k * z < 0)
        {
            test = 1;
        }
        else if (y - k * z == 0)
        {
            q2 = 0;
        }
    }
    else if (theta1 == PI / 2 || theta1 == -PI / 2)
    {
        if (z > 0)
        {
            test = 0;
        }
        else if (z < 0)
        {
            test = 1;
        }
        else if (z == 0)
        {
            q2 = 0;
        }
    }

    //求q2
    if (test == 1)
    {
        if (theta1 > 0 && theta1 < PI / 2)
        {
            a_nop = theta1 - theta2;
        }
        else if (theta1 == 0)
        {
            a_nop = theta2;
        }
        else if (theta1 > -PI / 2 && theta1 < 0)
        {
            if (z > 0)
            {
                a_nop = theta1 + theta2;
            }
            else if (z < 0)
            {
                a_nop = PI + theta1 - theta2;
            }
            else if (z == 0)
            {
                a_nop = PI / 2 + theta1;
            }
        }
        else if (theta1 == PI / 2)
        {
            a_nop = PI / 2 - theta2;
        }

        l_np = l_no * std::sin(a_nop);
        a_mpn = std::atan(l_mn / l_np);

        if (x > 0)
        {
            q2 = PI / 2 - a_mpn;
        }
        else if (x < 0)
        {
            q2 = -(PI / 2 - a_mpn);
        }
        else if (x == 0)
        {
            if (c > 0)
            {
                q2 = PI / 2;
            }
            else if (c < 0)
            {
                q2 = -PI / 2;
            }
        }
    }
    else if (test == 0)
    {
        if (theta1 > 0 && theta1 < PI / 2)
        {
            if (z > 0)
            {
                a_nop = PI - theta1 - theta2;
            }
            else if (z < 0)
            {
                a_nop = theta2 - theta1;
            }
            else if (z == 0)
            {
                a_nop = PI / 2 - theta2;
            }
        }
        else if (theta1 == 0)
        {
            a_nop = theta2;
        }
        else if (theta1 > -PI / 2 && theta1 < 0)
        {
            a_nop = theta1 + theta2;
        }
        else if (theta1 == PI / 2)
        {
            a_nop = PI / 2 - theta2;
        }

        l_np = l_no * std::sin(a_nop);
        a_mpn = std::atan(l_mn / l_np);

        if (x > 0)
        {
            q2 = -(PI / 2 - a_mpn);
        }
        else if (x < 0)
        {
            q2 = PI / 2 - a_mpn;
        }
        else if (x == 0)
        {
            if (c > 0)
            {
                q2 = -PI / 2;
            }
            else if (c < 0)
            {
                q2 = PI / 2;
            }
        }
    }

    //-----q3-----
    //将末端坐标转换为腿平面坐标。
    //设腿上的四个转动副组成的腿平面为平面x0Oy0。
    //点O与原坐标一致，x0轴与x轴一致，
    //q1为0时，y0轴与y轴一致，q2不为0时，y0轴以y轴为轴，转动q2后的新轴。
    if (q2 == 0)
    {
        x0 = x;
        y0 = -std::sqrt(y * y + z * z);
    }
    else if (q2 != 0)
    {
        x0 = l_mn / std::sin(a_mpn) * x / std::abs(x);
        y0 = -std::sqrt(l_no * l_no - l_np * l_np);
    }

    //求脚踝转动副坐标（x1，y1）
    l_qs = a /std::cos(q2);
    l_rs = std::sqrt(l_qs * l_qs - a * a);
    l_or = std::sqrt(b * b + c * c);
    l_os = std::sqrt(l_or * l_or - l_rs * l_rs);
    a_lim = std::atan(l_os / l_qs);
    a_eij = std::atan(kLEJ / kLIJ);
    l_lm = l_im * std::sin(a_lim);
    l_il = l_im *std::cos(a_lim);
    theta2 = std::acos(std::abs(z) / l_no);

    if (std::abs(theta1) == PI / 2)
    {
        if (b > 0)
        {
            if (a_eij + a_lim > PI / 2)
            {
                a_eik = PI - a_eij - a_lim;
                l_ik = kLEI *std::cos(a_eik);
                l_ek = kLEI * std::sin(a_eik);

                x1 = x0 - l_ik - l_il;
                y1 = y0 - l_lm + l_ek;
            }
            else if (a_eij + a_lim == PI / 2)
            {
                x1 = x0 - l_il;
                y1 = y0 - l_lm + kLEI;
            }
            else if (a_eij + a_lim < PI / 2)
            {
                a_eik = a_eij + a_lim;
                l_ik = kLEI *std::cos(a_eik);
                l_ek = kLEI * std::sin(a_eik);

                x1 = x0 + l_ik - l_il;
                y1 = y0 - l_lm + l_ek;
            }
        }
        else if (b == 0)
        {
            x1 = x0 - l_im + kLIJ;
            y1 = y0 + kLEJ;
        }
        else if (b < 0)
        {
            a_eik = a_eij - a_lim;
            l_ik = kLEI *std::cos(a_eik);
            l_ek = kLEI * std::sin(a_eik);

            x1 = x0 + l_ik - l_il;
            y1 = y0 + l_lm + l_ek;
        }
    }
    else if (-k * c < b)
    {
        if (a_eij + a_lim > PI / 2)
        {
            a_eik = PI - a_eij - a_lim;
            l_ik = kLEI *std::cos(a_eik);
            l_ek = kLEI * std::sin(a_eik);

            x1 = x0 - l_ik - l_il;
            y1 = y0 - l_lm + l_ek;
        }
        else if (a_eij + a_lim == PI / 2)
        {
            x1 = x0 - l_il;
            y1 = y0 - l_lm + kLEI;
        }
        else if (a_eij + a_lim < PI / 2)
        {
            a_eik = a_eij + a_lim;
            l_ik = kLEI *std::cos(a_eik);
            l_ek = kLEI * std::sin(a_eik);

            x1 = x0 + l_ik - l_il;
            y1 = y0 - l_lm + l_ek;
        }
    }
    else if (-k * c == b)
    {
        x1 = x0 - l_im + kLIJ;
        y1 = y0 + kLEJ;
    }
    else if (-k * c > b)
    {
        a_eik = a_eij - a_lim;
        l_ik = kLEI *std::cos(a_eik);
        l_ek = kLEI * std::sin(a_eik);

        x1 = x0 + l_ik - l_il;
        y1 = y0 + l_lm + l_ek;
    }

    //求q3
    l_ae = std::sqrt(x1 * x1 + (y1 + kLAO) * (y1 + kLAO));
    l_af = -(y1 + kLAO);
    l_ef = std::abs(x1);
    a_cbe = std::acos((kLBC * kLBC + kLBE * kLBE - kLCE * kLCE) / (2 * kLBC * kLBE));
    a_abe = std::acos((kLAB * kLAB + kLBE * kLBE - l_ae * l_ae) / (2 * kLAB * kLBE));
    a_abc = a_abe - a_cbe;
    l_ac = std::sqrt(kLAB * kLAB + kLBC * kLBC - 2 * kLAB * kLBC * std::cos(a_abc));
    a_bac = std::acos((kLAB * kLAB + l_ac * l_ac - kLBC * kLBC) / (2 * kLAB * l_ac));
    a_cad = std::acos((l_ac * l_ac + kLAD * kLAD - kLCD * kLCD) / (2 * l_ac * kLAD));
    a_bad = a_bac + a_cad;
    a_bae = std::acos((kLAB * kLAB + l_ae * l_ae - kLBE * kLBE) / (2 * kLAB * l_ae));
    a_eaf = std::acos((l_ae * l_ae + l_af * l_af - l_ef * l_ef) / (2 * l_ae * l_af));

    if (x1 > 0)
    {
        a_daf = a_bad - a_bae + a_eaf;
    }
    else if (x1 == 0)
    {
        a_daf = a_bad - a_bae;
    }
    else if (x1 < 0)
    {
        a_daf = a_bad - a_bae - a_eaf;
    }

    q3 = PI - a_daf - kQ30;

    //-----q4-----
    a_adc = std::acos((kLAD * kLAD + kLCD * kLCD - l_ac * l_ac) / (2 * kLAD * kLCD));

    q4 = a_adc - 3.0 / 4.0 * PI;

    //-----q5-----
    //求∠GEI
    a_aec = std::acos((l_ae * l_ae + kLCE * kLCE - l_ac * l_ac) / (2 * l_ae * kLCE));
    a_aeb = std::acos((l_ae * l_ae + kLBE * kLBE - kLAB * kLAB) / (2 * l_ae * kLBE));
    a_aef = std::acos((l_ae * l_ae + x1 * x1 - (y1 + kLAO) * (y1 + kLAO)) / (2 * l_ae * std::abs(x1)));
    a_bec = std::acos((kLCE * kLCE + kLBE * kLBE - kLBC * kLBC) / (2 * kLCE * kLBE));

    if (x1 > 0)
    {
        if (a_aeb > a_bec)
        {
            theta3 = a_aef - (a_aeb - a_bec);
        }
        else if (a_aeb == a_bec)
        {
            theta3 = a_aef;
        }
        else if (a_aeb < a_bec)
        {
            theta3 = a_aef + (a_bec - a_aeb);
        }

        if (a_lim == 0)
        {
            theta4 = a_eij;
        }
        else if (a_lim != 0)
        {
            if (b > 0)
            {
                theta4 = a_eij + a_lim;
            }
            else if (b < 0)
            {
                theta4 = a_eij - a_lim;
            }
        }
    }
    else if (x1 == 0)
    {
        if (a_aeb > a_bec)
        {
            theta3 = PI / 2 - (a_aeb - a_bec);
        }
        else if (a_aeb == a_bec)
        {
            theta3 = PI / 2;
        }
        else if (a_aeb < a_bec)
        {
            theta3 = PI / 2 - (a_aeb - a_bec);
        }

        if (a_lim == 0)
        {
            theta4 = a_eij;
        }
        else if (a_lim != 0)
        {
            if (b > 0)
            {
                theta4 = a_eij + a_lim;
            }
            else if (b < 0)
            {
                theta4 = a_eij - a_lim;
            }
        }
    }
    else if (x1 < 0)
    {
        theta3 = PI - a_aef - a_aeb + a_bec;

        if (a_lim == 0)
        {
            theta4 = a_eij;
        }
        else if (a_lim != 0)
        {
            if (b > 0)
            {
                theta4 = a_eij + a_lim;
            }
            else if (b < 0)
            {
                theta4 = a_eij - a_lim;
            }
        }
    }

    a_gei = theta3 + theta4;

    //求∠GEI最大值和最小值
    a_gei_b = std::acos((kLEG * kLEG + kLEI * kLEI - (kLGH + kLHI) * (kLGH + kLHI)) / (2 * kLEG * kLEI));
    a_gei_s = std::acos((kLEG * kLEG + (kLEI + kLHI) * (kLEI + kLHI) - kLGH * kLGH) / (2 * kLEG * (kLEI + kLHI)));

    //求q5
    if (a_gei > a_gei_b)
    {
        a_gei = a_gei_b;
    }
    else if (a_gei < a_gei_s)
    {
        a_gei = a_gei_s;
    }

    l_gi = std::sqrt(kLEG * kLEG + kLEI * kLEI - 2 * kLEG * kLEI *std::cos(a_gei));
    a_hgi = std::acos((kLGH * kLGH + l_gi * l_gi - kLHI * kLHI) / (2 * kLGH * l_gi));
    a_egi = std::acos((kLEG * kLEG + l_gi * l_gi - kLEI * kLEI) / (2 * kLEG * l_gi));
    a_egh = a_egi + a_hgi;

    q5 = a_egh - kQ50;



    pos[0] = q1;
    pos[1] = q2;
    pos[2] = q3;
    pos[3] = q4;
    pos[4] = q5;

}





auto inverse(double* leg_in_ground, double* body_in_ground, double* input)->int
{





    double real_pm_l[16] = { 0 }, real_pm_r[16] = { 0 };



    aris::dynamic::s_pm_dot_inv_pm(PL, body_in_ground, real_pm_l);
    aris::dynamic::s_pm_dot_inv_pm(PR, body_in_ground, real_pm_r);


    double xyz_in_leg[6] = { 0 }; //腿末端在腿坐标系下的表达
    aris::dynamic::s_pp2pp(real_pm_l, leg_in_ground + 0 * 3, xyz_in_leg + 0 * 3);
    aris::dynamic::s_pp2pp(real_pm_r, leg_in_ground + 1 * 3, xyz_in_leg + 1 * 3);


    inverseLeg(xyz_in_leg[0], xyz_in_leg[1], xyz_in_leg[2], 1, 0, 0, 150, input_angle + 0 * 5);//left
    inverseLeg(xyz_in_leg[3], xyz_in_leg[4], xyz_in_leg[5], 1, 0, 0, 150, input_angle + 1 * 5);//ringht




    return 0;
}



