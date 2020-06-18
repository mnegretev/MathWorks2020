/* Simscape target specific file.
 * This file is generated for the Simscape network associated with the solver block 'la_2dof_simul/Solver Configuration'.
 */

#include "ne_std.h"
#include "pm_default_allocator.h"
#include "ne_dae.h"
#include "ne_solverparameters.h"
#include "sm_ssci_NeDaePrivateData.h"

NeDae *sm_ssci_constructDae(NeDaePrivateData *smData);
void la_2dof_simul_e38647b6_1_NeDaePrivateData_create(NeDaePrivateData *smData);
void la_2dof_simul_e38647b6_1_dae(
  NeDae **dae,
  const NeModelParameters *modelParams,
  const NeSolverParameters *solverParams)
{
  PmAllocator *alloc = pm_default_allocator();
  NeDaePrivateData *smData =
    (NeDaePrivateData *) alloc->mCallocFcn(alloc, sizeof(NeDaePrivateData), 1);
  (void) modelParams;
  (void) solverParams;
  la_2dof_simul_e38647b6_1_NeDaePrivateData_create(smData);
  *dae = sm_ssci_constructDae(smData);
}
