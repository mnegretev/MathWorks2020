/* Simscape target specific file.
 * This file is generated for the Simscape network associated with the solver block 'left_arm_simul/Manipulator/Solver Configuration'.
 */

#include <math.h>
#include <string.h>
#include "pm_std.h"
#include "sm_std.h"
#include "ne_std.h"
#include "ne_dae.h"
#include "sm_ssci_run_time_errors.h"
#include "sm_RuntimeDerivedValuesBundle.h"

void left_arm_simul_e2442d81_1_computeRuntimeParameters(real_T *in, real_T *out)
{
  (void) in;
  (void) out;
}

void left_arm_simul_e2442d81_1_computeAsmRuntimeDerivedValuesDoubles(const
  double *rtp, double *rtdvd)
{
  (void) rtp;
  (void) rtdvd;
}

void left_arm_simul_e2442d81_1_computeAsmRuntimeDerivedValuesInts(const double
  *rtp, int *rtdvi)
{
  (void) rtp;
  (void) rtdvi;
}

void left_arm_simul_e2442d81_1_computeAsmRuntimeDerivedValues(const double *rtp,
  RuntimeDerivedValuesBundle *rtdv)
{
  left_arm_simul_e2442d81_1_computeAsmRuntimeDerivedValuesDoubles(rtp,
    rtdv->mDoubles.mValues);
  left_arm_simul_e2442d81_1_computeAsmRuntimeDerivedValuesInts(rtp,
    rtdv->mInts.mValues);
}

void left_arm_simul_e2442d81_1_computeSimRuntimeDerivedValuesDoubles(const
  double *rtp, double *rtdvd)
{
  (void) rtp;
  (void) rtdvd;
}

void left_arm_simul_e2442d81_1_computeSimRuntimeDerivedValuesInts(const double
  *rtp, int *rtdvi)
{
  (void) rtp;
  (void) rtdvi;
}

void left_arm_simul_e2442d81_1_computeSimRuntimeDerivedValues(const double *rtp,
  RuntimeDerivedValuesBundle *rtdv)
{
  left_arm_simul_e2442d81_1_computeSimRuntimeDerivedValuesDoubles(rtp,
    rtdv->mDoubles.mValues);
  left_arm_simul_e2442d81_1_computeSimRuntimeDerivedValuesInts(rtp,
    rtdv->mInts.mValues);
}
