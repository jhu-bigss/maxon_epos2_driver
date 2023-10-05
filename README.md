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