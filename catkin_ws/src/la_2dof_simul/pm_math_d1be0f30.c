#include "pm_std.h"
#include "pm_std.h"
void pm_math_lin_alg_svd(uint32_T pm_math_VLHhnPUiNQpve5VIL9P3O9,uint32_T n,
const real_T*pm_math_F2l4p_g4sn02huHNflQjMH,boolean_T
pm_math_V6w3JSNtpMlafi_5eay9eA,real_T*pm_math__eqHosfeNhCyXer5wSjLxX,real_T*
pm_math_Vi4Cp0qK964NYTFMGr9Ttn,real_T*pm_math_kJeUz19e49pVaDwcttsZep,real_T*
pm_math_VRZCD_UL_ESThy75dC9J8D);uint32_T pm_math_lin_alg_svdReqdWorkSize(
uint32_T pm_math_VLHhnPUiNQpve5VIL9P3O9,uint32_T n,boolean_T
pm_math_V6w3JSNtpMlafi_5eay9eA);void pm_math_lin_alg_svdSolve(uint32_T
pm_math_VLHhnPUiNQpve5VIL9P3O9,uint32_T n,const real_T*
pm_math_F2l4p_g4sn02huHNflQjMH,const real_T*b,real_T
pm_math_kReQWwdwpSK9bDljzjMa8v,real_T*x,real_T*pm_math_VRZCD_UL_ESThy75dC9J8D)
;uint32_T pm_math_lin_alg_svdSolveReqdWorkSize(uint32_T
pm_math_VLHhnPUiNQpve5VIL9P3O9,uint32_T n);uint32_T
pm_math_lin_alg_svdSolveFromFactorization(uint32_T
pm_math_VLHhnPUiNQpve5VIL9P3O9,uint32_T n,const real_T*
pm_math__eqHosfeNhCyXer5wSjLxX,const real_T*pm_math_Vi4Cp0qK964NYTFMGr9Ttn,
const real_T*pm_math_kJeUz19e49pVaDwcttsZep,const real_T*b,real_T
pm_math_kReQWwdwpSK9bDljzjMa8v,real_T*x,real_T*pm_math_VRZCD_UL_ESThy75dC9J8D)
;uint32_T pm_math_lin_alg_svdSolveFromFactorizationReqdWorkSize(uint32_T
pm_math_VLHhnPUiNQpve5VIL9P3O9,uint32_T n);uint32_T
pm_math_lin_alg_svdSolveAdaptive(uint32_T pm_math_VLHhnPUiNQpve5VIL9P3O9,
uint32_T n,const real_T*pm_math_F2l4p_g4sn02huHNflQjMH,const real_T*b,real_T
pm_math__pOMwIA5LrSig5PHyHM6ut,real_T pm_math__0ren1fNSMteWudwQ_U_yg,real_T
pm_math_FmrN4Uy2o5hwaiBvN_fj_X,real_T*x,real_T*pm_math_VRZCD_UL_ESThy75dC9J8D)
;uint32_T pm_math_lin_alg_svdSolveAdaptiveReqdWorkSize(uint32_T
pm_math_VLHhnPUiNQpve5VIL9P3O9,uint32_T n);uint32_T
pm_math_lin_alg_svdComputeRank(uint32_T pm_math_VLHhnPUiNQpve5VIL9P3O9,
uint32_T n,const real_T*pm_math__eqHosfeNhCyXer5wSjLxX,real_T
pm_math_kReQWwdwpSK9bDljzjMa8v);
#include "stddef.h"
#include "pm_std.h"
extern real_T pm_math_VnOEjVNyg2OZhucGc5E7EV;extern real_T
pm_math_VS_DaUL57WSYYqKdhOZhAh;extern real_T pm_math_FlzEJVEXHExB_HzmP_4DCn;
extern real32_T pm_math_FbzPcgniy7ldiP9jki1kva;extern real32_T
pm_math_VXzHwLau5r0Jh1hUy2Bmz9;extern real32_T pm_math__GrRRdc1Zn8FXHIp4WdZwI;
extern void pm_math__q6vs_E9mICEhq7U4Gz9lI(size_t
pm_math_FpJeiUKpmBKFaHvp42RGuS);extern boolean_T pm_math_FKbWI5oUVjCZZe_cTAJ6Ic
(real_T pm_math_kg4CyRgbu6OdeewZOel7Qx);extern boolean_T
pm_math_VdM_6HcFx_xL_5LdjcZ7os(real32_T pm_math_kg4CyRgbu6OdeewZOel7Qx);extern
boolean_T pm_math_ky6GmdNS7wdyiTXxHKu5b_(real_T pm_math_kg4CyRgbu6OdeewZOel7Qx
);extern boolean_T pm_math_VJGDZZVQuR8__LXaPq08E0(real32_T
pm_math_kg4CyRgbu6OdeewZOel7Qx);typedef struct{struct{uint32_T
pm_math__ARTIDuiem4sfDDt8LmBon;uint32_T pm_math_FhD1Saz7ZedmceSGBn3Lu9;}
pm_math_FRl2WNCcA_CVZ1mCqjGWQg;}pm_math_k9JbamRSA8KJfTkbhKNlzj;typedef struct{
struct{uint32_T pm_math_FhD1Saz7ZedmceSGBn3Lu9;uint32_T
pm_math__ARTIDuiem4sfDDt8LmBon;}pm_math_FRl2WNCcA_CVZ1mCqjGWQg;}
pm_math_VOGaPgsZpCx_Vy8G1ynQHA;typedef struct{union{real32_T
pm_math_VpUvuayfPZCoZaqi9TlQTF;uint32_T pm_math_FaKzb6mka3hHWHjqYy3m5v;}
pm_math_FhD1Saz7ZedmceSGBn3Lu9;}pm_math__87B1TG2jL0Sf5EO4yoSqA;
#include "string.h"
void pm_math_kAcEPCREGSOwXaYastObJk(int32_T pm_math_VLHhnPUiNQpve5VIL9P3O9,
int32_T n,const real_T*pm_math_F2l4p_g4sn02huHNflQjMH,boolean_T
pm_math_V6w3JSNtpMlafi_5eay9eA,real_T*pm_math_Vi4Cp0qK964NYTFMGr9Ttn,real_T*
pm_math__Q_i1Z0_CMGpfPqpuLrnMT,real_T*pm_math_kJeUz19e49pVaDwcttsZep,real_T*
pm_math_VRZCD_UL_ESThy75dC9J8D);void pm_math_lin_alg_svd(uint32_T
pm_math_VLHhnPUiNQpve5VIL9P3O9,uint32_T n,const real_T*
pm_math_F2l4p_g4sn02huHNflQjMH,boolean_T pm_math_V6w3JSNtpMlafi_5eay9eA,real_T
*pm_math__Q_i1Z0_CMGpfPqpuLrnMT,real_T*pm_math_Vi4Cp0qK964NYTFMGr9Ttn,real_T*
pm_math_kJeUz19e49pVaDwcttsZep,real_T*pm_math_VRZCD_UL_ESThy75dC9J8D){
pm_math__q6vs_E9mICEhq7U4Gz9lI(8U);pm_math_kAcEPCREGSOwXaYastObJk(
pm_math_VLHhnPUiNQpve5VIL9P3O9,n,pm_math_F2l4p_g4sn02huHNflQjMH,
pm_math_V6w3JSNtpMlafi_5eay9eA,pm_math_Vi4Cp0qK964NYTFMGr9Ttn,
pm_math__Q_i1Z0_CMGpfPqpuLrnMT,pm_math_kJeUz19e49pVaDwcttsZep,
pm_math_VRZCD_UL_ESThy75dC9J8D);}uint32_T pm_math_lin_alg_svdReqdWorkSize(
uint32_T pm_math_VLHhnPUiNQpve5VIL9P3O9,uint32_T n,boolean_T
pm_math_V6w3JSNtpMlafi_5eay9eA){const int32_T pm_math_F32Ql82vv6pW_PYIdpkFQ0=
pm_math_VLHhnPUiNQpve5VIL9P3O9<=n?pm_math_VLHhnPUiNQpve5VIL9P3O9:n;return
pm_math_VLHhnPUiNQpve5VIL9P3O9*n+(pm_math_F32Ql82vv6pW_PYIdpkFQ0+1)+n+
pm_math_VLHhnPUiNQpve5VIL9P3O9+(pm_math_V6w3JSNtpMlafi_5eay9eA?0:n*n);}void
pm_math_lin_alg_svdSolve(uint32_T pm_math_VLHhnPUiNQpve5VIL9P3O9,uint32_T n,
const real_T*pm_math_F2l4p_g4sn02huHNflQjMH,const real_T*b,real_T
pm_math_kReQWwdwpSK9bDljzjMa8v,real_T*x,real_T*pm_math_VRZCD_UL_ESThy75dC9J8D)
{const uint32_T pm_math_F32Ql82vv6pW_PYIdpkFQ0=pm_math_VLHhnPUiNQpve5VIL9P3O9
<=n?pm_math_VLHhnPUiNQpve5VIL9P3O9:n;uint32_T pm_math_FTaLdMEVj0h_didxvJF_00=
pm_math_lin_alg_svdSolveFromFactorizationReqdWorkSize(
pm_math_VLHhnPUiNQpve5VIL9P3O9,n);real_T*pm_math__Q_i1Z0_CMGpfPqpuLrnMT,*
pm_math_Vi4Cp0qK964NYTFMGr9Ttn,*pm_math_kJeUz19e49pVaDwcttsZep,*
pm_math_VEKz5t_FyApvZLOFgvELNO,*pm_math_k63NgCm5J7_ZgXwg055AL_;
pm_math__Q_i1Z0_CMGpfPqpuLrnMT=pm_math_VRZCD_UL_ESThy75dC9J8D,
pm_math_VRZCD_UL_ESThy75dC9J8D+=pm_math_F32Ql82vv6pW_PYIdpkFQ0;
pm_math_Vi4Cp0qK964NYTFMGr9Ttn=pm_math_VRZCD_UL_ESThy75dC9J8D,
pm_math_VRZCD_UL_ESThy75dC9J8D+=pm_math_VLHhnPUiNQpve5VIL9P3O9*
pm_math_F32Ql82vv6pW_PYIdpkFQ0;pm_math_kJeUz19e49pVaDwcttsZep=
pm_math_VRZCD_UL_ESThy75dC9J8D,pm_math_VRZCD_UL_ESThy75dC9J8D+=n*
pm_math_F32Ql82vv6pW_PYIdpkFQ0;pm_math_k63NgCm5J7_ZgXwg055AL_=
pm_math_VRZCD_UL_ESThy75dC9J8D,pm_math_VRZCD_UL_ESThy75dC9J8D+=
pm_math_FTaLdMEVj0h_didxvJF_00;pm_math_VEKz5t_FyApvZLOFgvELNO=
pm_math_VRZCD_UL_ESThy75dC9J8D;pm_math_lin_alg_svd(
pm_math_VLHhnPUiNQpve5VIL9P3O9,n,pm_math_F2l4p_g4sn02huHNflQjMH,(0U),
pm_math__Q_i1Z0_CMGpfPqpuLrnMT,pm_math_Vi4Cp0qK964NYTFMGr9Ttn,
pm_math_kJeUz19e49pVaDwcttsZep,pm_math_VEKz5t_FyApvZLOFgvELNO);
pm_math_lin_alg_svdSolveFromFactorization(pm_math_VLHhnPUiNQpve5VIL9P3O9,n,
pm_math__Q_i1Z0_CMGpfPqpuLrnMT,pm_math_Vi4Cp0qK964NYTFMGr9Ttn,
pm_math_kJeUz19e49pVaDwcttsZep,b,pm_math_kReQWwdwpSK9bDljzjMa8v,x,
pm_math_k63NgCm5J7_ZgXwg055AL_);}uint32_T pm_math_lin_alg_svdSolveReqdWorkSize
(uint32_T pm_math_VLHhnPUiNQpve5VIL9P3O9,uint32_T n){const int32_T
pm_math_F32Ql82vv6pW_PYIdpkFQ0=pm_math_VLHhnPUiNQpve5VIL9P3O9<=n?
pm_math_VLHhnPUiNQpve5VIL9P3O9:n;return pm_math_F32Ql82vv6pW_PYIdpkFQ0+
pm_math_VLHhnPUiNQpve5VIL9P3O9*pm_math_F32Ql82vv6pW_PYIdpkFQ0+n*
pm_math_F32Ql82vv6pW_PYIdpkFQ0+pm_math_VLHhnPUiNQpve5VIL9P3O9+
pm_math_lin_alg_svdReqdWorkSize(pm_math_VLHhnPUiNQpve5VIL9P3O9,n,(0U))+
pm_math_lin_alg_svdSolveFromFactorizationReqdWorkSize(
pm_math_VLHhnPUiNQpve5VIL9P3O9,n);}uint32_T
pm_math_lin_alg_svdSolveFromFactorization(uint32_T
pm_math_VLHhnPUiNQpve5VIL9P3O9,uint32_T n,const real_T*
pm_math__eqHosfeNhCyXer5wSjLxX,const real_T*pm_math_Vi4Cp0qK964NYTFMGr9Ttn,
const real_T*pm_math_kJeUz19e49pVaDwcttsZep,const real_T*b,real_T
pm_math_kReQWwdwpSK9bDljzjMa8v,real_T*x,real_T*pm_math_VRZCD_UL_ESThy75dC9J8D)
{const uint32_T pm_math_F32Ql82vv6pW_PYIdpkFQ0=pm_math_VLHhnPUiNQpve5VIL9P3O9
<=n?pm_math_VLHhnPUiNQpve5VIL9P3O9:n;uint32_T pm_math__q70gXw8N_xsWy9UoFEkTZ=0
;uint32_T pm_math_kUQBO1dSP8_IVqRAUx4R8G,pm_math_FFZbGh27ya8eem_J_hUtAZ;real_T
*pm_math_VM4unYGEE54zdmx4fBRzOl;pm_math_VM4unYGEE54zdmx4fBRzOl=
pm_math_VRZCD_UL_ESThy75dC9J8D;if(pm_math_F32Ql82vv6pW_PYIdpkFQ0>0&&
pm_math__eqHosfeNhCyXer5wSjLxX[0]>0.0){const real_T
pm_math_kEw8RhbAZIGdfeViKpSqbH=(pm_math_kReQWwdwpSK9bDljzjMa8v*
pm_math__eqHosfeNhCyXer5wSjLxX[0]);for(pm_math__q70gXw8N_xsWy9UoFEkTZ=0;
pm_math__q70gXw8N_xsWy9UoFEkTZ<pm_math_F32Ql82vv6pW_PYIdpkFQ0&&
pm_math__eqHosfeNhCyXer5wSjLxX[pm_math__q70gXw8N_xsWy9UoFEkTZ]>=
pm_math_kEw8RhbAZIGdfeViKpSqbH;++pm_math__q70gXw8N_xsWy9UoFEkTZ){}}for(
pm_math_kUQBO1dSP8_IVqRAUx4R8G=0;pm_math_kUQBO1dSP8_IVqRAUx4R8G<
pm_math__q70gXw8N_xsWy9UoFEkTZ;++pm_math_kUQBO1dSP8_IVqRAUx4R8G){real_T
pm_math_Vzz7fHz6oElvjujXUEM3YM=0.0;for(pm_math_FFZbGh27ya8eem_J_hUtAZ=0;
pm_math_FFZbGh27ya8eem_J_hUtAZ<pm_math_VLHhnPUiNQpve5VIL9P3O9;++
pm_math_FFZbGh27ya8eem_J_hUtAZ)pm_math_Vzz7fHz6oElvjujXUEM3YM+= *
pm_math_Vi4Cp0qK964NYTFMGr9Ttn++*b[pm_math_FFZbGh27ya8eem_J_hUtAZ];
pm_math_VM4unYGEE54zdmx4fBRzOl[pm_math_kUQBO1dSP8_IVqRAUx4R8G]=
pm_math_Vzz7fHz6oElvjujXUEM3YM/pm_math__eqHosfeNhCyXer5wSjLxX[
pm_math_kUQBO1dSP8_IVqRAUx4R8G];}for(pm_math_kUQBO1dSP8_IVqRAUx4R8G=0;
pm_math_kUQBO1dSP8_IVqRAUx4R8G<n;++pm_math_kUQBO1dSP8_IVqRAUx4R8G)x[
pm_math_kUQBO1dSP8_IVqRAUx4R8G]=0.0;for(pm_math_FFZbGh27ya8eem_J_hUtAZ=0;
pm_math_FFZbGh27ya8eem_J_hUtAZ<pm_math__q70gXw8N_xsWy9UoFEkTZ;++
pm_math_FFZbGh27ya8eem_J_hUtAZ){const real_T pm_math_kHfMSiCBJRSoju34wwOKOx= *
pm_math_VM4unYGEE54zdmx4fBRzOl++;for(pm_math_kUQBO1dSP8_IVqRAUx4R8G=0;
pm_math_kUQBO1dSP8_IVqRAUx4R8G<n;++pm_math_kUQBO1dSP8_IVqRAUx4R8G)x[
pm_math_kUQBO1dSP8_IVqRAUx4R8G]+=pm_math_kHfMSiCBJRSoju34wwOKOx**
pm_math_kJeUz19e49pVaDwcttsZep++;}return pm_math__q70gXw8N_xsWy9UoFEkTZ;}
uint32_T pm_math_lin_alg_svdSolveFromFactorizationReqdWorkSize(uint32_T
pm_math_VLHhnPUiNQpve5VIL9P3O9,uint32_T n){return
pm_math_VLHhnPUiNQpve5VIL9P3O9;}uint32_T pm_math_lin_alg_svdSolveAdaptive(
uint32_T pm_math_VLHhnPUiNQpve5VIL9P3O9,uint32_T n,const real_T*
pm_math_F2l4p_g4sn02huHNflQjMH,const real_T*b,real_T
pm_math__pOMwIA5LrSig5PHyHM6ut,real_T pm_math__0ren1fNSMteWudwQ_U_yg,real_T
pm_math_FmrN4Uy2o5hwaiBvN_fj_X,real_T*x,real_T*pm_math_VRZCD_UL_ESThy75dC9J8D)
{const uint32_T pm_math_F32Ql82vv6pW_PYIdpkFQ0=pm_math_VLHhnPUiNQpve5VIL9P3O9
<=n?pm_math_VLHhnPUiNQpve5VIL9P3O9:n;uint32_T pm_math_VPoMeho6rmtkiu6RUm0MmI=0
,pm_math_FQdtq_lrqcpIXyjvI_MHWr=0;uint32_T pm_math_V_BZ_NX0y8KnX5LnoDh895=0;
uint32_T pm_math__q70gXw8N_xsWy9UoFEkTZ,pm_math_kUQBO1dSP8_IVqRAUx4R8G,
pm_math_FFZbGh27ya8eem_J_hUtAZ;real_T*pm_math__Q_i1Z0_CMGpfPqpuLrnMT,*
pm_math_Vi4Cp0qK964NYTFMGr9Ttn,*pm_math_kJeUz19e49pVaDwcttsZep,*
pm_math_VM4unYGEE54zdmx4fBRzOl,*pm_math_kVd_yTmfkRh8WuxM6Hcqy_,*
pm_math_VEKz5t_FyApvZLOFgvELNO;pm_math__Q_i1Z0_CMGpfPqpuLrnMT=
pm_math_VRZCD_UL_ESThy75dC9J8D,pm_math_VRZCD_UL_ESThy75dC9J8D+=
pm_math_F32Ql82vv6pW_PYIdpkFQ0;pm_math_Vi4Cp0qK964NYTFMGr9Ttn=
pm_math_VRZCD_UL_ESThy75dC9J8D,pm_math_VRZCD_UL_ESThy75dC9J8D+=
pm_math_VLHhnPUiNQpve5VIL9P3O9*pm_math_F32Ql82vv6pW_PYIdpkFQ0;
pm_math_kJeUz19e49pVaDwcttsZep=pm_math_VRZCD_UL_ESThy75dC9J8D,
pm_math_VRZCD_UL_ESThy75dC9J8D+=n*pm_math_F32Ql82vv6pW_PYIdpkFQ0;
pm_math_VM4unYGEE54zdmx4fBRzOl=pm_math_VRZCD_UL_ESThy75dC9J8D,
pm_math_VRZCD_UL_ESThy75dC9J8D+=pm_math_VLHhnPUiNQpve5VIL9P3O9;
pm_math_kVd_yTmfkRh8WuxM6Hcqy_=pm_math_VRZCD_UL_ESThy75dC9J8D,
pm_math_VRZCD_UL_ESThy75dC9J8D+=n;pm_math_VEKz5t_FyApvZLOFgvELNO=
pm_math_VRZCD_UL_ESThy75dC9J8D;pm_math_lin_alg_svd(
pm_math_VLHhnPUiNQpve5VIL9P3O9,n,pm_math_F2l4p_g4sn02huHNflQjMH,(0U),
pm_math__Q_i1Z0_CMGpfPqpuLrnMT,pm_math_Vi4Cp0qK964NYTFMGr9Ttn,
pm_math_kJeUz19e49pVaDwcttsZep,pm_math_VEKz5t_FyApvZLOFgvELNO);if(
pm_math_F32Ql82vv6pW_PYIdpkFQ0>0&&pm_math__Q_i1Z0_CMGpfPqpuLrnMT[0]>0.0){
real_T pm_math_kEw8RhbAZIGdfeViKpSqbH;pm_math_kEw8RhbAZIGdfeViKpSqbH=(
pm_math__pOMwIA5LrSig5PHyHM6ut*pm_math__Q_i1Z0_CMGpfPqpuLrnMT[0]);for(
pm_math_VPoMeho6rmtkiu6RUm0MmI=0;pm_math_VPoMeho6rmtkiu6RUm0MmI<
pm_math_F32Ql82vv6pW_PYIdpkFQ0;++pm_math_VPoMeho6rmtkiu6RUm0MmI)if(
pm_math__Q_i1Z0_CMGpfPqpuLrnMT[pm_math_VPoMeho6rmtkiu6RUm0MmI]<
pm_math_kEw8RhbAZIGdfeViKpSqbH)break;pm_math_kEw8RhbAZIGdfeViKpSqbH=(
pm_math__0ren1fNSMteWudwQ_U_yg*pm_math__Q_i1Z0_CMGpfPqpuLrnMT[0]);for(
pm_math_FQdtq_lrqcpIXyjvI_MHWr=pm_math_VPoMeho6rmtkiu6RUm0MmI;
pm_math_FQdtq_lrqcpIXyjvI_MHWr<pm_math_F32Ql82vv6pW_PYIdpkFQ0;++
pm_math_FQdtq_lrqcpIXyjvI_MHWr)if(pm_math__Q_i1Z0_CMGpfPqpuLrnMT[
pm_math_FQdtq_lrqcpIXyjvI_MHWr]<pm_math_kEw8RhbAZIGdfeViKpSqbH)break;}if(
pm_math_FQdtq_lrqcpIXyjvI_MHWr==0)return 0;for(pm_math_kUQBO1dSP8_IVqRAUx4R8G=
0;pm_math_kUQBO1dSP8_IVqRAUx4R8G<pm_math_FQdtq_lrqcpIXyjvI_MHWr;++
pm_math_kUQBO1dSP8_IVqRAUx4R8G){real_T pm_math_Vzz7fHz6oElvjujXUEM3YM=0.0;for(
pm_math_FFZbGh27ya8eem_J_hUtAZ=0;pm_math_FFZbGh27ya8eem_J_hUtAZ<
pm_math_VLHhnPUiNQpve5VIL9P3O9;++pm_math_FFZbGh27ya8eem_J_hUtAZ)
pm_math_Vzz7fHz6oElvjujXUEM3YM+= *pm_math_Vi4Cp0qK964NYTFMGr9Ttn++*b[
pm_math_FFZbGh27ya8eem_J_hUtAZ];pm_math_VM4unYGEE54zdmx4fBRzOl[
pm_math_kUQBO1dSP8_IVqRAUx4R8G]=pm_math_Vzz7fHz6oElvjujXUEM3YM/
pm_math__Q_i1Z0_CMGpfPqpuLrnMT[pm_math_kUQBO1dSP8_IVqRAUx4R8G];}for(
pm_math_kUQBO1dSP8_IVqRAUx4R8G=0;pm_math_kUQBO1dSP8_IVqRAUx4R8G<n;++
pm_math_kUQBO1dSP8_IVqRAUx4R8G)x[pm_math_kUQBO1dSP8_IVqRAUx4R8G]=0.0;for(
pm_math_FFZbGh27ya8eem_J_hUtAZ=0;pm_math_FFZbGh27ya8eem_J_hUtAZ<
pm_math_VPoMeho6rmtkiu6RUm0MmI;++pm_math_FFZbGh27ya8eem_J_hUtAZ){const real_T
pm_math_kHfMSiCBJRSoju34wwOKOx= *pm_math_VM4unYGEE54zdmx4fBRzOl++;for(
pm_math_kUQBO1dSP8_IVqRAUx4R8G=0;pm_math_kUQBO1dSP8_IVqRAUx4R8G<n;++
pm_math_kUQBO1dSP8_IVqRAUx4R8G)x[pm_math_kUQBO1dSP8_IVqRAUx4R8G]+=
pm_math_kHfMSiCBJRSoju34wwOKOx**pm_math_kJeUz19e49pVaDwcttsZep++;}++
pm_math_V_BZ_NX0y8KnX5LnoDh895;if(pm_math_FQdtq_lrqcpIXyjvI_MHWr>
pm_math_VPoMeho6rmtkiu6RUm0MmI){real_T pm_math_kT9EdvcMqS_fi1U1Nm16e4=0.0;
const double*pm_math_kLrNH_YoIJxHbiQ8S5IeJY=pm_math_F2l4p_g4sn02huHNflQjMH;for
(pm_math_kUQBO1dSP8_IVqRAUx4R8G=0;pm_math_kUQBO1dSP8_IVqRAUx4R8G<
pm_math_VLHhnPUiNQpve5VIL9P3O9;++pm_math_kUQBO1dSP8_IVqRAUx4R8G){real_T
pm_math_Vzz7fHz6oElvjujXUEM3YM=0.0;pm_math_kLrNH_YoIJxHbiQ8S5IeJY=
pm_math_F2l4p_g4sn02huHNflQjMH+pm_math_kUQBO1dSP8_IVqRAUx4R8G;for(
pm_math_FFZbGh27ya8eem_J_hUtAZ=0;pm_math_FFZbGh27ya8eem_J_hUtAZ<n;++
pm_math_FFZbGh27ya8eem_J_hUtAZ,pm_math_kLrNH_YoIJxHbiQ8S5IeJY+=
pm_math_VLHhnPUiNQpve5VIL9P3O9)pm_math_Vzz7fHz6oElvjujXUEM3YM+= *
pm_math_kLrNH_YoIJxHbiQ8S5IeJY*x[pm_math_FFZbGh27ya8eem_J_hUtAZ];
pm_math_Vzz7fHz6oElvjujXUEM3YM-=b[pm_math_kUQBO1dSP8_IVqRAUx4R8G];
pm_math_kT9EdvcMqS_fi1U1Nm16e4+=pm_math_Vzz7fHz6oElvjujXUEM3YM*
pm_math_Vzz7fHz6oElvjujXUEM3YM;}memcpy(pm_math_kVd_yTmfkRh8WuxM6Hcqy_,x,n*
sizeof(real_T));x+=n;for(pm_math__q70gXw8N_xsWy9UoFEkTZ=
pm_math_VPoMeho6rmtkiu6RUm0MmI+1;pm_math__q70gXw8N_xsWy9UoFEkTZ<=
pm_math_FQdtq_lrqcpIXyjvI_MHWr;++pm_math__q70gXw8N_xsWy9UoFEkTZ){real_T
pm_math_VqT0gIZXuM8McuQYYsgQvq;const real_T pm_math_kHfMSiCBJRSoju34wwOKOx= *
pm_math_VM4unYGEE54zdmx4fBRzOl++;for(pm_math_kUQBO1dSP8_IVqRAUx4R8G=0;
pm_math_kUQBO1dSP8_IVqRAUx4R8G<n;++pm_math_kUQBO1dSP8_IVqRAUx4R8G)
pm_math_kVd_yTmfkRh8WuxM6Hcqy_[pm_math_kUQBO1dSP8_IVqRAUx4R8G]+=
pm_math_kHfMSiCBJRSoju34wwOKOx**pm_math_kJeUz19e49pVaDwcttsZep++;
pm_math_VqT0gIZXuM8McuQYYsgQvq=0.0;for(pm_math_kUQBO1dSP8_IVqRAUx4R8G=0;
pm_math_kUQBO1dSP8_IVqRAUx4R8G<pm_math_VLHhnPUiNQpve5VIL9P3O9;++
pm_math_kUQBO1dSP8_IVqRAUx4R8G){real_T pm_math_Vzz7fHz6oElvjujXUEM3YM=0.0;
pm_math_kLrNH_YoIJxHbiQ8S5IeJY=pm_math_F2l4p_g4sn02huHNflQjMH+
pm_math_kUQBO1dSP8_IVqRAUx4R8G;for(pm_math_FFZbGh27ya8eem_J_hUtAZ=0;
pm_math_FFZbGh27ya8eem_J_hUtAZ<n;++pm_math_FFZbGh27ya8eem_J_hUtAZ,
pm_math_kLrNH_YoIJxHbiQ8S5IeJY+=pm_math_VLHhnPUiNQpve5VIL9P3O9)
pm_math_Vzz7fHz6oElvjujXUEM3YM+= *pm_math_kLrNH_YoIJxHbiQ8S5IeJY*
pm_math_kVd_yTmfkRh8WuxM6Hcqy_[pm_math_FFZbGh27ya8eem_J_hUtAZ];
pm_math_Vzz7fHz6oElvjujXUEM3YM-=b[pm_math_kUQBO1dSP8_IVqRAUx4R8G];
pm_math_VqT0gIZXuM8McuQYYsgQvq+=pm_math_Vzz7fHz6oElvjujXUEM3YM*
pm_math_Vzz7fHz6oElvjujXUEM3YM;}if(pm_math_VqT0gIZXuM8McuQYYsgQvq<
pm_math_kT9EdvcMqS_fi1U1Nm16e4*pm_math_FmrN4Uy2o5hwaiBvN_fj_X*
pm_math_FmrN4Uy2o5hwaiBvN_fj_X){pm_math_kT9EdvcMqS_fi1U1Nm16e4=
pm_math_VqT0gIZXuM8McuQYYsgQvq;memcpy(x,pm_math_kVd_yTmfkRh8WuxM6Hcqy_,n*
sizeof(real_T));x+=n;++pm_math_V_BZ_NX0y8KnX5LnoDh895;}}}return
pm_math_V_BZ_NX0y8KnX5LnoDh895;}uint32_T
pm_math_lin_alg_svdSolveAdaptiveReqdWorkSize(uint32_T
pm_math_VLHhnPUiNQpve5VIL9P3O9,uint32_T n){const int32_T
pm_math_F32Ql82vv6pW_PYIdpkFQ0=pm_math_VLHhnPUiNQpve5VIL9P3O9<=n?
pm_math_VLHhnPUiNQpve5VIL9P3O9:n;return pm_math_F32Ql82vv6pW_PYIdpkFQ0+
pm_math_VLHhnPUiNQpve5VIL9P3O9*pm_math_F32Ql82vv6pW_PYIdpkFQ0+n*
pm_math_F32Ql82vv6pW_PYIdpkFQ0+pm_math_VLHhnPUiNQpve5VIL9P3O9+n+
pm_math_lin_alg_svdReqdWorkSize(pm_math_VLHhnPUiNQpve5VIL9P3O9,n,(0U));}
uint32_T pm_math_lin_alg_svdComputeRank(uint32_T pm_math_VLHhnPUiNQpve5VIL9P3O9
,uint32_T n,const real_T*pm_math__eqHosfeNhCyXer5wSjLxX,real_T
pm_math_kReQWwdwpSK9bDljzjMa8v){const uint32_T pm_math_F32Ql82vv6pW_PYIdpkFQ0=
pm_math_VLHhnPUiNQpve5VIL9P3O9<=n?pm_math_VLHhnPUiNQpve5VIL9P3O9:n;uint32_T
pm_math__q70gXw8N_xsWy9UoFEkTZ=0;if(pm_math_F32Ql82vv6pW_PYIdpkFQ0>0&&*
pm_math__eqHosfeNhCyXer5wSjLxX>0.0){const real_T pm_math_kEw8RhbAZIGdfeViKpSqbH
=(pm_math_kReQWwdwpSK9bDljzjMa8v*pm_math__eqHosfeNhCyXer5wSjLxX[0]);for(
pm_math__q70gXw8N_xsWy9UoFEkTZ=0;pm_math__q70gXw8N_xsWy9UoFEkTZ<
pm_math_F32Ql82vv6pW_PYIdpkFQ0&&*pm_math__eqHosfeNhCyXer5wSjLxX++>=
pm_math_kEw8RhbAZIGdfeViKpSqbH;++pm_math__q70gXw8N_xsWy9UoFEkTZ){}}return
pm_math__q70gXw8N_xsWy9UoFEkTZ;}
