#include "mag_calibration.h"
#include "stdbool.h"
#include "math.h"

#define DATA_COLLECT_TIMES 1500
#define MATRIX_ORDER    3
#define EPS 1e-10
#define MAX_ITER 1000

static double off_diagonal_norm(double CovMatrix[MATRIX_ORDER][MATRIX_ORDER])  ; 
static void find_max_off_diagonal(double CovMatrix[MATRIX_ORDER][MATRIX_ORDER], int *p, int *q) ; 
static void compute_jacobi_rotation(double CovMatrix[MATRIX_ORDER][MATRIX_ORDER], int p, int q, double *c, double *s, double *theta) ; 
static void apply_jacobi_rotation(double CovMatrix[MATRIX_ORDER][MATRIX_ORDER], int p, int q, double c, double s) ;
static void update_eigenvectors(double V[MATRIX_ORDER][MATRIX_ORDER], int p, int q, double c, double s) ; 
static void jacobi_eigen(double A[MATRIX_ORDER][MATRIX_ORDER], double eigenvalues[MATRIX_ORDER], 
                        double eigenvectors[MATRIX_ORDER][MATRIX_ORDER], double tolerance, 
                        int max_iterations, bool verbose) ; 
static void Get_Feature(float CovMatrix[3][3], float SoftCal[3][3]) ; 


static void Get_CovMatrix(float data[][3], uint16_t num_samples, float CovMatrix[3][3]) {
    // 初始化为 0
    for(int m=0; m<3; m++) for(int n=0; n<3; n++) CovMatrix[m][n] = 0.0f;
    
    // 计算协方差 
    for(int m=0; m<3; m++) {
        for(int n=0; n<3; n++) {
            for(int i=0; i<num_samples; i++) {
                CovMatrix[m][n] += data[i][m] * data[i][n]; 
            }
            CovMatrix[m][n] /= (num_samples - 1.0f);
        }
    }
}
 
// 计算矩阵的非对角元素范数（用于判断收敛）
static double off_diagonal_norm(double CovMatrix[MATRIX_ORDER][MATRIX_ORDER]) {
    double norm = 0.0 ;
    for (int i = 0 ; i < MATRIX_ORDER ; i++) {
        for (int j = 0 ; j < MATRIX_ORDER ; j++) {
            if (i != j) {
                norm += CovMatrix[i][j] * CovMatrix[i][j] ;
            }
        }
    }
    return sqrt(norm) ;
}
// 寻找绝对值最大的非对角元素位置
static void find_max_off_diagonal(double CovMatrix[MATRIX_ORDER][MATRIX_ORDER], int *p, int *q) {
    double max_val = 0.0 ;
    *p = 0 ;
    *q = 1 ;
    
    for (int i = 0 ; i < MATRIX_ORDER ; i++) {
        for (int j = i+1 ; j < MATRIX_ORDER ; j++) {
            if (fabs(CovMatrix[i][j]) > max_val) {
                max_val = fabs(CovMatrix[i][j]) ;
                *p = i ;
                *q = j ;
            }
        }
    }
}
// Jacobi旋转矩阵计算
static void compute_jacobi_rotation(double CovMatrix[MATRIX_ORDER][MATRIX_ORDER], int p, int q, 
                            double *c, double *s, double *theta) {
    double app = CovMatrix[p][p] ;
    double aqq = CovMatrix[q][q] ;
    double apq = CovMatrix[p][q] ;
    
   // 计算旋转角度
    if (fabs(apq) < EPS) {
        *c = 1.0 ;
        *s = 0.0 ;
        *theta = 0.0 ;
        return ;
    }
    
    double tau = (aqq - app) / (2.0 * apq) ;
    double t ;
    
    if (tau >= 0) {
        t = 1.0 / (tau + sqrt(1.0 + tau * tau)) ;
    } else {
        t = -1.0 / (-tau + sqrt(1.0 + tau * tau)) ;
    }
    
    *c = 1.0 / sqrt(1.0 + t * t) ;
    *s = t * (*c) ;
    *theta = atan(t) ;
}
// 应用Jacobi旋转到矩阵A
static void apply_jacobi_rotation(double CovMatrix[MATRIX_ORDER][MATRIX_ORDER], int p, int q, double c, double s) {
      // 保存p行和q行的旧值
    double old_p[MATRIX_ORDER], old_q[MATRIX_ORDER] ;
    for (int j = 0 ; j < MATRIX_ORDER ; j++) {
        old_p[j] = CovMatrix[p][j] ;
        old_q[j] = CovMatrix[q][j] ;
    }
    
      // 更新p行和q行
    for (int j = 0 ; j < MATRIX_ORDER ; j++) {
        if (j != p && j != q) {
            CovMatrix[p][j] = CovMatrix[j][p] = c * old_p[j] - s * old_q[j] ;
            CovMatrix[q][j] = CovMatrix[j][q] = s * old_p[j] + c * old_q[j] ;
        }
    }
    
   // 更新对角线元素
    CovMatrix[p][p] = c * c * old_p[p] - 2 * s * c * old_p[q] + s * s * old_q[q] ;
    CovMatrix[q][q] = s * s * old_p[p] + 2 * s * c * old_p[q] + c * c * old_q[q] ;
    CovMatrix[p][q] = CovMatrix[q][p] = 0.0 ;
}
// 更新特征向量矩阵V
static void update_eigenvectors(double V[MATRIX_ORDER][MATRIX_ORDER], int p, int q, double c, double s) {
    for (int i = 0 ; i < MATRIX_ORDER ; i++) {
        double vip = V[i][p] ;
        double viq = V[i][q] ;
        V[i][p] = c * vip - s * viq ;
        V[i][q] = s * vip + c * viq ;
    }
}
 // Jacobi迭代法主函数
static void jacobi_eigen(double A[MATRIX_ORDER][MATRIX_ORDER], double eigenvalues[MATRIX_ORDER], 
                        double eigenvectors[MATRIX_ORDER][MATRIX_ORDER], 
                 double tolerance, int max_iterations, bool verbose) {
     // 初始化特征向量矩阵为单位矩阵
    for (int i = 0 ; i < MATRIX_ORDER ; i++) {
        for (int j = 0 ; j < MATRIX_ORDER ; j++) {
            eigenvectors[i][j] = (i == j) ? 1.0 : 0.0 ;
        }
    }
    
       // 复制矩阵A到工作矩阵
    double B[MATRIX_ORDER][MATRIX_ORDER] ;
    for (int i = 0 ; i < MATRIX_ORDER ; i++) {
        for (int j = 0 ; j < MATRIX_ORDER ; j++) {
            B[i][j] = A[i][j] ;
        }
    }
    
    
    int iteration = 0 ;
    double off_norm = off_diagonal_norm(B) ;
    
    while (off_norm > tolerance && iteration < max_iterations) {
        //  寻找最大的非对角元素
        int p, q ;
        find_max_off_diagonal(B, &p, &q) ;
        
        double c, s, theta ;
        compute_jacobi_rotation(B, p, q, &c, &s, &theta) ;
        
        // 应用旋转
        apply_jacobi_rotation(B, p, q, c, s) ;
        
       //  更新特征向量
        update_eigenvectors(eigenvectors, p, q, c, s) ;
        
        off_norm = off_diagonal_norm(B) ;
        iteration++ ;
        
        if (verbose && iteration % 10 == 0) {
           
        }
    }
    
    if (verbose) {
       
    }
    
    // 提取特征值（对角线元素）
    for (int i = 0 ; i < MATRIX_ORDER ; i++) {
        eigenvalues[i] = B[i][i] ;
    }
    
   // 特征值排序（降序）
    for (int i = 0 ; i < MATRIX_ORDER-1 ; i++) {
        for (int j = i+1 ; j < MATRIX_ORDER ; j++) {
            if (eigenvalues[i] < eigenvalues[j]) {
                  //交换特征值
                double temp_val = eigenvalues[i] ;
                eigenvalues[i] = eigenvalues[j] ;
                eigenvalues[j] = temp_val ;
                
                 //  交换对应的特征向量
                for (int k = 0 ; k < MATRIX_ORDER ; k++) {
                    double temp_vec = eigenvectors[k][i] ;
                    eigenvectors[k][i] = eigenvectors[k][j] ;
                    eigenvectors[k][j] = temp_vec ;
                }
            }
        }
    }
}
         
static void Get_Feature(float CovMatrix[3][3], float SoftCal[3][3])
{
    // 转换 CovMatrix 从 float 到 double 类型 (Jacobi 函数需要 double)
    double A_double[MATRIX_ORDER][MATRIX_ORDER] ;
    for (int i = 0 ; i < MATRIX_ORDER ; i++) {
        for (int j = 0 ; j < MATRIX_ORDER ; j++) {
            A_double[i][j] = (double)CovMatrix[i][j] ;
        }
    }

    double eigenvalues[MATRIX_ORDER] ;
    double eigenvectors[MATRIX_ORDER][MATRIX_ORDER] ; // V matrix
    
    // 调用 Jacobi 迭代法
    jacobi_eigen(A_double, eigenvalues, eigenvectors, EPS, MAX_ITER, false) ;

     // 计算缩放因子 (Soft Iron Scaling)
     // 软铁校准矩阵 A_soft = V * Lambda^(-1/2) * V^T
    
    // 特征值是方差，其平方根是标准差。椭球轴长与标准差成正比
    // 将椭球缩放到一个球体
    double scale_factors[MATRIX_ORDER] ;
    double avg_lambda = (eigenvalues[0] + eigenvalues[1] + eigenvalues[2]) / 3.0 ;

    for (int i = 0 ; i < MATRIX_ORDER ; i++) {
         // 计算缩放系数： sqrt(平均特征值 / 当前特征值)
         // 目标是让校准后的数据方差都等于 avg_lambda
        if (eigenvalues[i] > EPS) {
            scale_factors[i] = sqrt(avg_lambda / eigenvalues[i]) ;
        } else {
            // 归零保护
            scale_factors[i] = 1.0 ;
        }
    }
    
    // 构建 SoftCal 矩阵 SoftCal = V * Lambda^(-1/2) * V^T
    // SoftCal[3][3] 就是软铁校准矩阵 A_soft
    double temp_matrix[MATRIX_ORDER][MATRIX_ORDER] = {0} ; // V * Lambda^(-1/2)
    
    for (int i = 0 ; i < MATRIX_ORDER ; i++) { //  行 (V)
        for (int j = 0 ; j < MATRIX_ORDER ; j++) { // // 列 (Lambda^(-1/2) 的对角元素)
             // 只有对角线有值: eigenvectors[i][j] * scale_factors[j]
            temp_matrix[i][j] = eigenvectors[i][j] * scale_factors[j] ;
        }
    }

    // SoftCal = temp_matrix * V^T
    for (int i = 0 ; i < MATRIX_ORDER ; i++) {
        for (int j = 0 ; j < MATRIX_ORDER ; j++) {
            SoftCal[i][j] = 0.0f ;
            for (int k = 0 ; k < MATRIX_ORDER ; k++) {
                // V^T 的 (k, j) 元素是 V 的 (j, k) 元素
                SoftCal[i][j] += (float)(temp_matrix[i][k] * eigenvectors[j][k]) ; 
            }
        }
    }
}

bool MagCal_ComputeParams(float data[][3], uint16_t num_samples, MagCal_Param_t *out_param)
{
    float max[3] = {-10000.0f, -10000.0f, -10000.0f};
    float min[3] = {10000.0f, 10000.0f, 10000.0f};

    // 寻找极值 
    for (int i = 0; i < num_samples; i++) {
        if (data[i][0] == 0.0f && data[i][1] == 0.0f) continue; 
        for (int j = 0; j < 3; j++) {
            if (data[i][j] > max[j]) max[j] = data[i][j];
            if (data[i][j] < min[j]) min[j] = data[i][j];
        }
    }

    if ((max[0] - min[0]) < 20.0f || (max[1] - min[1]) < 20.0f) {
        return false; // 校准失败，没有足够的数据跨度
    }

    // 写入硬铁偏置参数
    out_param->hard_iron_offset[0] = (max[0] + min[0]) / 2.0f;
    out_param->hard_iron_offset[1] = (max[1] + min[1]) / 2.0f;
    out_param->hard_iron_offset[2] = (max[2] + min[2]) / 2.0f;

    // 将数据集进行中心化 (减去硬铁偏置)
    for(int i = 0; i < num_samples; i++) {
        data[i][0] -= out_param->hard_iron_offset[0];
        data[i][1] -= out_param->hard_iron_offset[1];
        data[i][2] -= out_param->hard_iron_offset[2];
    }

    // 4. 计算软铁矩阵
    float cov_matrix[3][3];
    Get_CovMatrix(data, num_samples, cov_matrix);
    Get_Feature(cov_matrix, out_param->soft_iron_matrix);

    return true;
}

void MagCal_Apply(const float raw_in[3], const MagCal_Param_t *param, float cal_out[3])
{
    float temp[3];
    
    //  消除硬铁干扰 (平移)
    temp[0] = raw_in[0] - param->hard_iron_offset[0];
    temp[1] = raw_in[1] - param->hard_iron_offset[1];
    temp[2] = raw_in[2] - param->hard_iron_offset[2];
    
    //  消除软铁干扰 (通过矩阵乘法实现球形化缩放和旋转)
    cal_out[0] = temp[0] * param->soft_iron_matrix[0][0] + temp[1] * param->soft_iron_matrix[0][1] + temp[2] * param->soft_iron_matrix[0][2];
    cal_out[1] = temp[0] * param->soft_iron_matrix[1][0] + temp[1] * param->soft_iron_matrix[1][1] + temp[2] * param->soft_iron_matrix[1][2];
    cal_out[2] = temp[0] * param->soft_iron_matrix[2][0] + temp[1] * param->soft_iron_matrix[2][1] + temp[2] * param->soft_iron_matrix[2][2];
}