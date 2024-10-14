
#ifndef _ROS_REFLECT_REFLECT_HPP_
#define _ROS_REFLECT_REFLECT_HPP_

#include <deque>
#include <string>
#include <type_traits>
#include <vector>

#include <rclcpp/rclcpp.hpp>

namespace ros_reflect {

/**
 * Deserializes a node's parameters into an object.
 * The object must have a reflect method.
 *
 * Example:
 *
 * struct MyParams {
 *    int x_;
 *    std::vector<std::string> strings_;
 *    // NestedParams must also have a reflect method.
 *    NestedParams nested_;
 *    std::vector<NestedParams> vec_of_params_;
 *
 *    template <typename Reflector>
 *    void reflect(Reflector &r) {
 *      // You can give your properties any name, as long as they are unique.
 *      r.property("x", x_);
 *      r.property("strings", strings_);
 *      r.property("nested", nested_);
 *      r.property("vec_of_params", vec_of_params_);
 *    }
 * };
 *
 */
template <typename NodeParams, typename NodeType>
void fetch_params(NodeType &node, NodeParams &params);

template <typename NodeParams>
void fetch_params(rclcpp::node_interfaces::NodeParametersInterface &interface,
                  NodeParams &params);

////////////////////////////////////////////////////////////////////////////////
// Implementation
////////////////////////////////////////////////////////////////////////////////

template <typename T>
concept IsPrimitiveNodeParam =
    requires(const rclcpp::ParameterValue &v, T &t) {
      { v.get<T>() };
    } || std::is_assignable_v<T, std::vector<std::string>> ||
    std::is_assignable_v<T, std::vector<bool>> ||
    std::is_assignable_v<T, std::vector<int64_t>> ||
    std::is_assignable_v<T, std::vector<double>>;

template <typename T, typename Reflector>
concept HasOwnReflectMethod = requires(T t, Reflector r) {
  { t.reflect(r) };
};

template <typename T, typename Reflector>
concept IsDeserializable =
    IsPrimitiveNodeParam<T> || HasOwnReflectMethod<T, Reflector>;

class ROSParamFetcher {

public:
  using ParametersInterface = rclcpp::node_interfaces::NodeParametersInterface;

  template <typename T>
    requires HasOwnReflectMethod<T, ROSParamFetcher>
  ROSParamFetcher(T &t) {
    t.reflect(*this);
  }

  void fetch_params(const ParametersInterface &interface) {
    // IMPORTANT: Due to the fact that ROS2 cannot parse lists of complex types,
    // properties may be added during calls to fetch_, so we cannot use a
    // range-based loop.
    for (uint i = 0; i < properties_.size(); ++i) {
      properties_[i].fetch_(interface);
    }
  }

  /** Register a primitive property. */
  template <typename T>
    requires IsPrimitiveNodeParam<T>
  void property(const std::string &name, T &ref,
                std::optional<T> &&default_value = std::nullopt) {
    std::string param_name = stack_.get_ros_param_name(name);

    properties_.emplace_back([&ref, path = std::move(param_name),
                              default_value = std::move(default_value)](
                                 const ParametersInterface &interface) {
      // TODO: call Node::declare_parameter somewhere.

      if (interface.has_parameter(path)) {
        get_parameter_wrapper(interface, path, ref);
      } else if (default_value.has_value()) {
        ref = std::move(default_value.value());
      } else {
        throw std::runtime_error("Parameter not found: " + path);
      }
    });
  }

  /** Register a vector of primitive or complex properties. */
  template <typename T>
    requires(IsDeserializable<T, ROSParamFetcher> && !IsPrimitiveNodeParam<T>)
  void property(const std::string &name, std::vector<T> &ref) {
    std::string prefix = stack_.get_ros_param_name(name);

    properties_.emplace_back([&ref, name, prefix = std::move(prefix),
                              // We need to copy the stack for reasons which
                              // will become apparent shortly...
                              stack_at_time_of_registration = stack_,
                              // And a ptr to this...
                              this](const ParametersInterface &node) mutable {
      std::map<std::string, rclcpp::Parameter> params;

      node.get_parameters_by_prefix(prefix, params);

      std::set<std::string> keys;

      for (const auto &[key, param] : params) {

        const size_t dot_index = key.find('.');

        if (dot_index == std::string::npos) {
          // TODO: probably better to return and aggregate errors so we don't
          // have to fix a config, launch a node, fix the config again just to
          // get feedback.
          throw std::runtime_error("Parameter \"" + prefix +
                                   "\" was declared as " +
                                   typeid(std::vector<T>).name() +
                                   " which is reflected as a map with named "
                                   "elements. There are no named elements in " +
                                   prefix + ".");
        }

        const std::string sub_key = key.substr(0, dot_index);

        keys.insert(sub_key);
      }

      // Add properties on the fly... that's why we need a copy of the stack
      // at the time of the initial call to property() ... :)

      // Temprarily swap the stacks so we can add properties on the fly.
      // This isn't pretty. An alternative would be to pass a stack ptr to each
      // version of property(). The problem here is that it's easy to forget to
      // use the stack from the parameters and not the member.
      // Swapping is rather cheap and ensures that the suffering caused by the
      // questionable design of ROS parameter parsing doesn't spread into the
      // rest of the property() implementations.
      // In fact, at this point, stack_ should be empty, so assignment would do.
      // Feels like something between YAGNI and something that could bite me
      // later if I didn't do it.
      std::swap(stack_at_time_of_registration, stack_);
      stack_.push(name);

      // Resize instead of clear and reserve to avoid deleting existing
      // elements.
      ref.resize(keys.size());
      uint i = 0;

      for (auto key = keys.begin(); key != keys.end(); ++key, ++i) {
        property(*key, ref.at(i));
      }

      std::swap(stack_at_time_of_registration, stack_);
    });
  }

  /** Register a complex property. */
  template <typename T>
    requires HasOwnReflectMethod<T, ROSParamFetcher>
  void property(const std::string &name, T &ref) {
    stack_.push(name);

    ref.reflect(*this);

    stack_.pop();
  }

protected:
  /**
   * Wrapper to make ParametersInterface::get_parameter behave like
   * Node::get_parameter...
   */
  template <typename ParameterT>
  static bool get_parameter_wrapper(const ParametersInterface &interface,
                                    const std::string &name,
                                    ParameterT &parameter) {

    rclcpp::Parameter parameter_variant;

    bool result = interface.get_parameter(name, parameter_variant);
    if (result) {
      parameter =
          static_cast<ParameterT>(parameter_variant.get_value<ParameterT>());
    }

    return result;
  }

protected:
  struct Stack {
    void push(const std::string &name) {
      if (stack_internal_.empty()) {
        stack_internal_.push_back(name);
      } else {
        stack_internal_.push_back(stack_internal_.back() + '.' + name);
      }
    }

    void pop() { stack_internal_.pop_back(); }

    [[nodiscard]] std::string
    get_ros_param_name(const std::string &name) const {
      if (stack_internal_.empty()) {
        return name;
      } else {
        return stack_internal_.back() + '.' + name;
      }
    }

  private:
    std::deque<std::string> stack_internal_;
  } stack_;

  class Property {
  public:
    using Fetcher = std::function<void(const ParametersInterface &)>;
    Fetcher fetch_;

    Property(Fetcher fetch) : fetch_(std::move(fetch)) {}
  }; // struct Property

  std::vector<Property> properties_;
};

template <typename NodeParams>
void fetch_params(rclcpp::node_interfaces::NodeParametersInterface &interface,
                  NodeParams &params) {
  ROSParamFetcher(params).fetch_params(interface);
}

template <typename NodeParams, typename NodeType>
void fetch_params(NodeType &node, NodeParams &params) {
  ROSParamFetcher(params).fetch_params(*node.get_node_parameters_interface());
}

} // namespace ros_reflect

#endif // _ROS_REFLECT_REFLECT_HPP_