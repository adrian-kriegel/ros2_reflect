
#include <cstdint>
#include <fstream>
#include <string>

#include <gmock/gmock.h>
#include <gtest/gtest.h>
#include <thread>

#include "ros2_reflect/reflect.hpp"

using namespace rclcpp;
using namespace ros_reflect;
using namespace testing;

/**
 * Helper to create a node with parameters.
 * There seems to be no API exported by ROS 2 to do this.
 * So we need to create a temporary YAML file and pass the path to the node.
 */
inline rclcpp::Node create_node_with_parameters(const std::string &yaml) {

  // Create a temporary YAML file unique to this thread.
  std::ostringstream file_name_oss;
  file_name_oss << "/tmp/params_" << getpid() << "_"
                << std::this_thread::get_id() << ".yaml";
  std::string file_name = file_name_oss.str();

  std::ofstream param_file(file_name);

  param_file << R"(
test_node:
  ros__parameters:
  )";

  std::istringstream yaml_stream(yaml);

  for (std::string line; std::getline(yaml_stream, line);) {
    // Pad with four spaces because we added test_node and ros__parameters
    param_file << "    " << line << std::endl;
  }

  param_file.close();

  rclcpp::NodeOptions options;
  options.arguments({"--ros-args", "--params-file", file_name})
      .allow_undeclared_parameters(true)
      .automatically_declare_parameters_from_overrides(true);

  return rclcpp::Node("test_node", options);
}

namespace ros_reflect_test {

struct NestedNodeParams {

  //
  //  These are "primitive" types which can be directly read from node params.
  //
  int int_;
  bool bool_;
  double double_;
  std::string string_;
  std::vector<std::string> strings_;
  std::vector<int64_t> ints_;
  std::vector<double> doubles_;
  std::vector<bool> bools_;

  //
  //  This is a custom type which requires the reflect mechanism.
  //

  struct MyStruct {

    std::string string_;

    // vector of structs within the struct
    std::vector<MyStruct> my_structs_;

    template <typename Reflector> void reflect(Reflector &r) {
      r.property("string", string_);

      r.property("my_structs", my_structs_);
    }
  } my_struct_;

  template <typename Reflector> void reflect(Reflector &r) {
    r.property("int", int_);
    r.property("bool", bool_);
    r.property("double", double_);
    r.property("string", string_);
    r.property("strings", strings_);
    r.property("ints", ints_);
    r.property("doubles", doubles_);
    r.property("bools", bools_);
    r.property("my_struct", my_struct_);
  }
}; // struct NodeParams

}; // namespace ros_reflect_test

using namespace ros_reflect_test;

/**
 * Not really a test case as it doesn't compile to anything.
 * I just wanted a place for the static_asserts.
 */
TEST(ros_reflect, ReflectPrimitivesStatic) {

  // From ROS2 doc:
  // bool, int64, float64, string, byte[], bool[], int64[], float64[] or
  // string[]

  static_assert(IsPrimitiveNodeParam<bool>);
  static_assert(IsPrimitiveNodeParam<int64_t>);
  static_assert(IsPrimitiveNodeParam<double>);
  static_assert(IsPrimitiveNodeParam<std::string>);
  static_assert(IsPrimitiveNodeParam<std::vector<bool>>);
  static_assert(IsPrimitiveNodeParam<std::vector<int64_t>>);
  static_assert(IsPrimitiveNodeParam<std::vector<double>>);
  static_assert(IsPrimitiveNodeParam<std::vector<std::string>>);

  static_assert(!IsPrimitiveNodeParam<NestedNodeParams>);
  static_assert(!IsPrimitiveNodeParam<NestedNodeParams::MyStruct>);
}

TEST(ros_reflect, ReflectComplex) {

  rclcpp::init(0, nullptr);

  std::string yaml = R"(
int: 42
bool: true
double: 3.14
string: hello
strings: [one, two, three]
ints: [1, 2, 3]
doubles: [1.0, 2.0, 3.0]
bools: [true, false, true]

my_struct:
  string: 'Nested hello'
  my_structs:
    # my_structs is a vector of structs which, while supported my YAML, is not supported by the ROS2 YAML parser
    # this means we have to give each element a name, which can be anything as long as it is unique.
    # TODO: add b_ and a_ to make sure there is no alphabetical sorting going on...
    element0:
      string: 'Hello from the first element.'
      # Ah, yes ROS cannot handle empty lists https://github.com/ros2/rclcpp/issues/1955
      my_structs: None

    element1:
      string: 'Hello from the second element.'
      my_structs: 
        element0:
          string: 'Hello from the first element of the second element.'
          my_structs: None
)";

  auto node = create_node_with_parameters(yaml);

  NestedNodeParams params;

  // Fetch twice to make sure there aren't any weird side-effects like growing
  // vectors.
  for (uint i = 0; i < 2; ++i) {
    fetch_params(node, params);

    // Shallow primitives.
    ASSERT_THAT(params.int_, Eq(42));
    ASSERT_THAT(params.bool_, Eq(true));
    ASSERT_THAT(params.double_, Eq(3.14));
    ASSERT_THAT(params.string_, Eq("hello"));
    ASSERT_THAT(params.strings_, ElementsAre("one", "two", "three"));
    ASSERT_THAT(params.ints_, ElementsAre(1, 2, 3));
    ASSERT_THAT(params.doubles_, ElementsAre(1.0, 2.0, 3.0));
    ASSERT_THAT(params.bools_, ElementsAre(true, false, true));

    // Complex properties.

    ASSERT_THAT(params.my_struct_.string_, Eq("Nested hello"));

    ASSERT_THAT(params.my_struct_.my_structs_.size(), Eq(2));

    ASSERT_THAT(params.my_struct_.my_structs_.at(0).string_,
                Eq("Hello from the first element."));
    ASSERT_THAT(params.my_struct_.my_structs_.at(0).my_structs_.size(), Eq(0));

    ASSERT_THAT(params.my_struct_.my_structs_.at(1).string_,
                Eq("Hello from the second element."));
    ASSERT_THAT(params.my_struct_.my_structs_.at(1).my_structs_.size(), Eq(1));

    ASSERT_THAT(params.my_struct_.my_structs_.at(1).my_structs_.at(0).string_,
                Eq("Hello from the first element of the second element."));
  }
}
