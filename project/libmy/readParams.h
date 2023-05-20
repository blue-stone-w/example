#ifndef READ_PARAMS_H
#define READ_PARAMS_H

#include <yaml-cpp/yaml.h>

template <typename Scalar>
static void readParam(YAML::Node config_node, Scalar &value, std::string &field)
{
  if (config_node)
  {
    value = config_node.as<Scalar>( );
  }
  else
  {
    LOG(FATAL) << "  failed to read " << field;
  }
}

#endif