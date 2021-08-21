#include <stdio.h>
#include "matrix.h"

void display_f32(mat_instance_f32 *pSrc)
{
    for(int i=0; i< pSrc->rows; i++)
    {
        for(int j=0; j< pSrc->cols; j++)
        {
            printf("%0.3f,\t",(double)pSrc->pData[i*pSrc->rows+j]);
        }
        printf("\r\n");
    }
}

void display_f64(mat_instance_f64 *pSrc)
{
    for(int i=0; i< pSrc->rows; i++)
    {
        for(int j=0; j< pSrc->cols; j++)
        {
            printf("%0.3f,\t",(double)pSrc->pData[i*pSrc->rows+j]);
        }
        printf("\r\n");
    }
}

int main()
{
    float A[9] = {1.0,3.0,6.0,3.0,9.0,1.0,6.0,5.0,9.0};
    float B[9] = {9.0,8.0,7.0,6.0,5.0,4.0,3.0,2.0,1.0};
    float AB[9];
    double C[9] = {1.0,3.0,6.0,3.0,9.0,1.0,6.0,5.0,9.0};
    double D[9] = {9.0,8.0,7.0,6.0,5.0,4.0,3.0,2.0,1.0};
    double CD[9];

    mat_instance_f32 A_Inst;
    mat_instance_f32 B_Inst;
    mat_instance_f32 AB_Inst;
    mat_instance_f64 C_Inst;
    mat_instance_f64 D_Inst;
    mat_instance_f64 CD_Inst;

    printf("\r\nTest f32 matrix function\r\n");
    mat_init_f32(&A_Inst,3,3,A);
    mat_init_f32(&B_Inst,3,3,B);
    mat_init_f32(&AB_Inst,3,3,AB);

    printf("A:\r\n");
    display_f32(&A_Inst);
    printf("B:\r\n");
    display_f32(&B_Inst);

    printf("A*0.1:\r\n");
    mat_scale_f32(&A_Inst,(float)0.1,&AB_Inst);
    display_f32(&AB_Inst);

    printf("A Transpose:\r\n");
    mat_trans_f32(&A_Inst,&AB_Inst);
    display_f32(&AB_Inst);

    printf("A + B:\r\n");
    mat_add_f32(&A_Inst,&B_Inst,&AB_Inst);
    display_f32(&AB_Inst);

    printf("A - B:\r\n");
    mat_sub_f32(&A_Inst,&B_Inst,&AB_Inst);
    display_f32(&AB_Inst);

    printf("A_INV:\r\n");
    mat_inverse_f32(&A_Inst,&AB_Inst);
    display_f32(&AB_Inst);

    printf("A * A_INV:\r\n");
    mat_mult_f32(&A_Inst,&AB_Inst,&B_Inst);
    display_f32(&B_Inst);

    printf("\r\nTest f64 matrix function\r\n");
    mat_init_f64(&C_Inst,3,3,C);
    mat_init_f64(&D_Inst,3,3,D);
    mat_init_f64(&CD_Inst,3,3,CD);

    printf("C:\r\n");
    display_f64(&C_Inst);
    printf("D:\r\n");
    display_f64(&D_Inst);

    printf("C*0.1:\r\n");
    mat_scale_f64(&C_Inst,(double)0.1,&CD_Inst);
    display_f64(&CD_Inst);

    printf("C Transpose:\r\n");
    mat_trans_f64(&C_Inst,&CD_Inst);
    display_f64(&CD_Inst);

    printf("C + D:\r\n");
    mat_add_f64(&C_Inst,&D_Inst,&CD_Inst);
    display_f64(&CD_Inst);

    printf("C - D:\r\n");
    mat_sub_f64(&C_Inst,&D_Inst,&CD_Inst);
    display_f64(&CD_Inst);

    printf("C_INV:\r\n");
    mat_inverse_f64(&C_Inst,&CD_Inst);
    display_f64(&CD_Inst);

    printf("C * C_INV:\r\n");
    mat_mult_f64(&C_Inst,&CD_Inst,&D_Inst);
    display_f64(&D_Inst);

    return 0;
}
