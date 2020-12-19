


#include "cmsis.h"



extern void mat_add_asm(uint32_t * mat_1,
    uint32_t * mat_2, uint32_t * mat_res);



static uint32_t matrix_1[4] =
{
  0x11121314,
  0x21222324,
  0x31323334,
  0x41424344,
};

static uint32_t matrix_2[4] =
{
  0x51525354,
  0x61626364,
  0x71727374,
  0x81828384,
};

static q15_t matrix_1_dsp[16] =
{
  0x11, 0x12, 0x13, 0x14,
  0x21, 0x22, 0x23, 0x24,
  0x31, 0x32, 0x33, 0x34,
  0x41, 0x42, 0x43, 0x44,
};

static q15_t matrix_2_dsp[16] =
{
  0x51, 0x52, 0x53, 0x54,
  0x61, 0x62, 0x63, 0x64,
  0x71, 0x72, 0x73, 0x74,
  0x81, 0x82, 0x83, 0x84,
};

static uint32_t matrix_add_res[4] = {0};
static uint32_t matrix_add_res_asm[4] = {0};
static q15_t matrix_add_res_dsp[16] = {0};


void mat_add_simd(void)
{
  for (uint32_t idx = 0; idx < 4; idx++)
  {
    matrix_add_res[idx] = __UADD8(matrix_1[idx],
        matrix_2[idx]);
  }
}

void mat_add_dsp_lib(void)
{
  /* Matrix Instance */
  arm_matrix_instance_q15 matrix_1;
  arm_matrix_instance_q15 matrix_2;
  arm_matrix_instance_q15 matrix_res;

  arm_status status;

  /* Initialize Matrix Instance */
  uint16_t srcRows = 4;
  uint16_t srcColumns = 4;

  arm_mat_init_q15(&matrix_1, srcRows, srcColumns, (q15_t *)matrix_1_dsp);
  arm_mat_init_q15(&matrix_2, srcRows, srcColumns, (q15_t *)matrix_2_dsp);
  arm_mat_init_q15(&matrix_res, srcRows, srcColumns, (q15_t *)matrix_add_res_dsp);

  /* Add */
  status = arm_mat_add_q15(&matrix_1, &matrix_2, &matrix_res);
}

void main(void)
{
  mat_add_simd();
  mat_add_dsp_lib();
  mat_add_asm(matrix_1, matrix_2, matrix_add_res_asm);
}


