/**
* @brief       : 
*
* @file        : acc_algorithm.c
* @author      : wangyaoting
* @version     : v0.0.1
* @date        : 2016/10/8
*
* Change Logs  :
*
* Date        Version      Author      Notes
* 2016/10/8    v0.0.1     wangyaoting    first version
*/
#include "acc_algorithm.h"
#include "math.h"

void setting (algo_acc_control_t algo_acc_control);
double analysis(acc_value_t * acc_buf, int len, acc_test_type_e acc_test_type);

/**
 * @brief 加速度运动检测算法参数设置结构体初始化
 */
algo_acc_control_t m_algo_acc_control = 
{
    .test_type = VARIANCE_MOVE,
    .upper_limit = 1.7,
    .lower_limit = 1.5,
    .angle_mount_x = 0,
    .angle_mount_y = 0,
    .angle_mount_z = 0,
};

/**
 * @brief 加速度运动检测算法接口初始化
 */
algorithm_acc_t algorithm_acc =
{
    .setting = setting,
    .analysis = analysis,
};

/**
 * @breaf 标准差求解函数
 * @param[in] D_buff 标准差求解数组.
 * @param[in] D_buff 标准差求解数组长度.
 * @return D_buff数据标准差.
 */
double standard_deviation(double * D_buff, int len)
{
    double D_value = 0.0;
    double E_value = 0.0;
    double sum = 0;
    
    for(int i = 0; i < len; i++)
    {
        E_value += D_buff[i] / len;
    }
        
    sum = 0.0;
    for(int i = 0; i < len; i++)
    {
        sum += (D_buff[i] - E_value) * (D_buff[i] - E_value);
    }
    D_value = sqrt(sum/(len - 1));
    return D_value;
}

/**
 * @breaf 加速度标准差计算函数
 * @param[in] acc_buf 加速度检测结果数组.
 * @param[in] acc_buf 加速度检测结果数组大小.
 * @return 单位时间加速度标准差.
 */
double variance_detect(acc_value_t * acc_buf, int len)
{
    // double acc_xyz[len], variance_acc;
    double variance_acc = 0.0;
    double *acc_xyz = (double *)malloc(len*sizeof(double));
    for(int i = 0; i < len; i++)
    {
        acc_xyz[i] = sqrt(acc_buf[i].x * acc_buf[i].x + 
                          acc_buf[i].y * acc_buf[i].y + 
                          acc_buf[i].z * acc_buf[i].z);
    }
    variance_acc = standard_deviation(acc_xyz, len);
    free(acc_xyz);
    return variance_acc;
}

/**
 * @breaf 积分求速度增量函数
 * @param[in] acc_buf 加速度检测结果数组.
 * @return 单位时间内速度增量.
 */
double velocity_detect(acc_value_t * acc_buf)
{
    double vel_x, vel_y, vel_z, vel_xyz;
    int16_t sum_accx = 0, sum_accy = 0, sum_accz = 0;
    for(int i = 0; i < 10; i++)
    {
        sum_accx += (acc_buf[i].x + 10);
        sum_accy += (acc_buf[i].y + 3);
        sum_accz += (acc_buf[i].z + 246);
    }
    vel_x = (2 * sum_accx - acc_buf[0].x - acc_buf[9].x) * 0.1 / 2;
    vel_y = (2 * sum_accy - acc_buf[0].y - acc_buf[9].y) * 0.1 / 2;
    vel_z = (2 * sum_accz - acc_buf[0].z - acc_buf[9].z) * 0.1 / 2;
    vel_xyz = sqrt(vel_x * vel_x + vel_y * vel_y + vel_z * vel_z);

    return vel_xyz;
}
/**
 * @breaf 加速度运动检测算法参数设置函数
 * @param[in] algo_acc_control 加速度运动检测算法设置值.
 */
void setting(algo_acc_control_t algo_acc_control)
{
    m_algo_acc_control = algo_acc_control;
}

/**
 * @breaf 加速度运动检测算法分析函数
 * @param[in] acc_buf 加速度检测结果数组.
 * @param[in] int  加速度检测结果数组大小
 * @param[in] acc_test_type 处理算法类型选择.
 * @return 相应算法处理结果.
 */
double analysis(acc_value_t * acc_buf, int len, acc_test_type_e acc_test_type)
{
    double analysis_result = 0.0;
    switch (acc_test_type)
    {
        case VARIANCE_MOVE:
            analysis_result = variance_detect(acc_buf, len);
            break;
        case VELOCITY_MOVE:
            // do not use
            // analysis_result = velocity_detect(acc_buf);
            break;
        default:
            break;
    }
    return analysis_result;
}