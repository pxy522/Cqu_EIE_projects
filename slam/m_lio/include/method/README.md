# 高斯牛顿优化算法作ICP点云配准
```
/* ICP点云配准算法流程 */
/* Step1: 粗配准得初始Rt */
/* Step2: 迭代n次后得到R^n t^n */
/* Step3: 源点云经过R^n t^n变换后，与目标点云作最邻居匹配得到点对 */
/* Step4: 根据点对P j 通过优化 argmin(R^n+1 t^n+1){ P^j - R^t@q^j + t^n } 得到R^n+1 t^n+1 */
/* Step5: 判断是否收敛，若收敛则停止，否则继续回到Step2 */

/* 这里我们使用高斯牛顿法进行优化求解 R^n+1 t^n+1 */
/* Gauss-Newton's method solve ICP: J^T*J*delta_x = -J^T*e */
/* J =  [ (de / dt) , (de / dR) ] */
/* e =  P - Rq + t */
/* de / dR = R hat(q) */
/* de / dt = -I */
/* J = [ I  -R hat(q) ] */
/* Hessian = J^T * J */
/* B = -J^T * e */
/* delta_x = Hessian^(-1) * B */
/* t^n+1 = t^n + delta_x[3:6] */
/* R^n+1 = R^n * exp(delta_x[0:3]) */
```