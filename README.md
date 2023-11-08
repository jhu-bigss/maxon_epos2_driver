# maxon_epos2_driver

Clone the repository to your ROS 2 workspace under the `src` directory:

```bash
git clone git@github.com:jhu-bigss/maxon_epos2_driver.git
```

## Build

You can use the ROS 2 VCS python-based tool (`sudo apt install
python3-vcstool`) to download all the cisst, sawUniversalRobot and ROS
2 specific packages needed for building this package.  Look
for the `.repos` file in this repository and find the link to the raw
content. Then in your ROS 2 workspace, under the `src` directory,
use:
```bash
vcs import --input https://raw.githubusercontent.com/jhu-bigss/maxon_epos2_driver/master/maxon_epos2_driver.repos
```
Then go back to the root of your ROS 2 workspace and build using:
```bash
colcon build
```

## Known Issues
When compiling the GUI, the compiler will complain about `typeinfo` and `vtable`, the error message looks like 
```bash
maxonWidget.cpp.o: in function `cmnClassServicesBase* cmnClassServicesInstantiate<maxonWidget>()':
maxonWidget.cpp:(.text+0x162): undefined reference to `typeinfo for maxonWidget'
maxonWidget.cpp.o: in function `maxonWidget::maxonWidget(maxonInterface*, QWidget*)':
maxonWidget.cpp:(.text+0x2e4): undefined reference to `vtable for maxonWidget'
maxonWidget.cpp:(.text+0x2f9): undefined reference to `vtable for maxonWidget'
maxonWidget.cpp:(.text+0x312): undefined reference to `vtable for maxonWidget'
maxonWidget.cpp.o: in function `maxonWidget::~maxonWidget()':
maxonWidget.cpp:(.text+0xccb): undefined reference to `vtable for maxonWidget'
maxonWidget.cpp:(.text+0xcdd): undefined reference to `vtable for maxonWidget'
```

This happens when the code depends on `QObject`. QT has a lot of predefined macros and a bunch of virtual methods. `QObejct` will include them at the place where you declare it. But all these virtual methods are not implemented, and that will cause the compiler to complain about the `typeinfo` and `vtable`. 

QT provides a tool called `MOC` to parse the header files that use `QObject`, and automatically generate the implementation for these virtual methods. To activate the `MOC`, you just need to insert one line in your `CMakeLists.txt`.
```cmake
set (CMAKE_AUTOMOC ON)
```

This works only after cmake finds QT packages. So it's better to put this command right below the find_packages command.
