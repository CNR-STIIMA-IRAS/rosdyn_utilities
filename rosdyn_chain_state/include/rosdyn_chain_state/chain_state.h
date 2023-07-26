#ifndef ROSDYN_UTILITIES__CHAIN_STATE__H
#define ROSDYN_UTILITIES__CHAIN_STATE__H

#include <memory>
#include <rdyn_chain_state/chain_state.h>

namespace rosdyn
{


/**
 * @class ChainState
 *
 * The class does not own the pointer to ChainInterface, but only a raw pointer is used.
 * This allow the usage also in the stack as stati object.
 */
using ChainState = rdyn::ChainState;


inline void get_joint_names(ros::NodeHandle& nh, std::vector<std::string>& names)
{
  std::vector<std::string> alternative_keys = { "controlled_resources",
                                                "controlled_resource",
                                                "controlled_joints",
                                                "controlled_joint",
                                                "joint_names",
                                                "joint_name",
                                                "joint_resource/joint_names",
                                                "joint_resource/joint_name"
                                                "joint_resource/controlled_joints",
                                                "joint_resource/controlled_joint"
                                                "joint_resource/controlled_resources",
                                                "joint_resource/controlled_resource" };

  names.clear();
  for (auto const& key : alternative_keys)
  {
    bool ok = false;
    try
    {
      ok = nh.getParam(key, names);
    }
    catch (const std::exception& e)
    {
      std::cerr << "Exception in getting '" << nh.getNamespace() << "/" << key << "': " << e.what() << '\n';
      ok = false;
    }

    if (!ok)
    {
      try
      {
        std::string joint_name;
        ok = nh.getParam(key, joint_name);
        if (ok)
        {
          names.push_back(joint_name);
        }
      }
      catch (const std::exception& e)
      {
        std::cerr << "Exception in getting '" << nh.getNamespace() << "/" << key << "': " << e.what() << '\n';
        ok = false;
      }
    }

    if (ok)
    {
      break;
    }
  }
  return;
}

}


#endif  // ROSDYN_UTILITIES__CHAIN_STATE__H
