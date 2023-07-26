#ifndef ROSDYN_UTILITIES__CHAIN_STATUS_N__H
#define ROSDYN_UTILITIES__CHAIN_STATUS_N__H

#include <rdyn_chain_state/chain_state_n.h>

namespace rosdyn
{
template <int N, int MaxN = N>
using ChainStateN = rdyn::ChainStateN<N, MaxN>;

/**
 *
 */
typedef ChainStateN<-1, -1> ChainStateX;
typedef std::shared_ptr<ChainStateX> ChainStateXPtr;
typedef std::shared_ptr<ChainStateX const> ChainStateXConstPtr;

/**
 * Define the ChainStateN# iwth a pre-defined number of axes
 */

#define RDEFINE_ChainStateN_PTR_STATIC_DIMENSION(nAx)                                                                   \
  using ChainState##nAx = ChainStateN<nAx>;                                                                            \
  typedef std::shared_ptr<ChainStateN<nAx> > ChainState##nAx##Ptr;                                                     \
  typedef std::shared_ptr<ChainStateN<nAx> const> ChainState##nAx##ConstPtr;

RDEFINE_ChainStateN_PTR_STATIC_DIMENSION(1);
RDEFINE_ChainStateN_PTR_STATIC_DIMENSION(3);
RDEFINE_ChainStateN_PTR_STATIC_DIMENSION(6);
RDEFINE_ChainStateN_PTR_STATIC_DIMENSION(7);

}  // namespace rosdyn

#endif  // ROSDYN_UTILITIES__CHAIN_STATUS_N__H
