# ros2_reflect 

Very crude attempt at making ROS2 parameters somewhat usable.

## Usage 

The library is header-only, so you don't need to build or link anything.

```C++
#include <ros2_reflect/reflect.hpp>

struct MyParams {
  int x_;
  std::vector<std::string> strings_;
  // NestedParams must also have a reflect method.
  NestedParams nested_;
  std::vector<NestedParams> vec_of_params_;

  template <typename Reflector>
  void reflect(Reflector &r) {
    // You can give your properties any name, as long as they are unique.
    r.property("x", x_);
    r.property("strings", strings_);
    r.property("nested", nested_);
    r.property("vec_of_params", vec_of_params_);
  }
};

MyParams params;

fetch_params(node, params);
```

Make sure you run your node with the following options: 

``` C++
.allow_undeclared_parameters(true)
.automatically_declare_parameters_from_overrides(true)
```
