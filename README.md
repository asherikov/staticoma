Staticoma -- static configuration manager
=========================================

[Disclaimer: this is a proof of concept, not mature software]

ROS utility package for build-time configuration file generation and
dumping/restoring contents of ROS parameter server to/from ROS bags.

Dependencies:
- `yq` -- https://mikefarah.gitbook.io/yq/
- `cmake`
- `catkin`
- `ROS` (`std_msgs`)
- `Boost`

Demo workspace -> https://github.com/asherikov/staticoma_workspace


Terminology
-----------

The following terms are not common, but useful for definition of the problems
addressed by staticoma:

- Configuration parameters can be divided into three categories:
    1. build-time -- values are known during compilation and never change;
    2. launch-time -- values are known at launch time and never change during
       execution;
    3. run-time -- those that change dynamically during execution.

- Stack -- a robot-specific set of packages providing multiple services and
  configuration parameters for them. Usually there are at least two partially
  overlapping stacks for a specific robot: one for simulation, and one for
  deployments.

- Configuration composition -- construction of configuration parameters from
  multiple, potentially overlapping pieces scattered among multiple (ROS)
  packages in a stack. Roughly speaking default parameters defined by packages
  providing certain functionality are overlayed with robot-sepecific
  parameters.



Standard ROS approach
---------------------

`roslaunch` (http://wiki.ros.org/roslaunch) -- ROS subsystem designed for
ROS-specific service management, provides two options to pass parameters to a
specific node (service):

- individual parameters can be passed via command line, but due to nested
  structure of roslaunch scripts such parameters often must be passed from top
  level to lower level scripts through a chain of intermediate scripts;

- parameters can be uploaded from YAML files to 'parameter server'
  (http://wiki.ros.org/Parameter%20Server) -- ROS service which manages global
  parameters at run-time, sequential uploading of parameters allows
  configuration composition.

`roslaunch` ensures that parameters declared in the stack launch script tree
are uploaded to parameter server in the order of their appearance before any of
the nodes are started.



Scope and goals of `staticoma`
------------------------------

`staticoma` addresses build-time and launch-time parameter management in order
to alleviate some of parameter server issues:

1. ROS allows recording data exchange between nodes using `rosbag` utility. The
   problem is that the content of ROS parameter server is not saved to bag
   files, but often affects data exchange. `staticoma` introduces a workaround
   that allows storing configurations in bag files, making them
   self-sufficient.

2. Launch-time configuration composition performed by `roslaunch` may be
   confusing since configuration pieces are stored in different locations and
   their precedence may not be straightforward. This problem is alleviated by
   performing build-time parameter composition to a single configuration file.

3. Node parameters may be removed or renamed during development, it would be
   nice to have a mechanism to enforce parameter consistency between 'parameter
   consumer' and 'parameter provider' packages. This goal is achieved by
   preventing providers from adding new parameters during build-time
   composition.

4. ROS parameter server has been deprecated in ROS2, and it is a good idea to
   develop services that do not rely on it.

5. `roslaunch` is inferior in many aspects to generic service managers such as
   `systemd`, but replacing it is very difficult due to deep integration with
   parameter server. Limited dependency on parameter server would allow to
   explore alternatives to `roslaunch`.



Key design concepts
-------------------

### Handling build-time parameters

1. All build-time parameters of a stack are written to a single file during
   compilation ('stack-config') using custom `cmake` functions.

2. stack-config is composed step by step by overlaying parameter files provided
   by package dependencies. For example: package B depends on A, A defines
   build-time parameters
   (https://github.com/asherikov/staticoma_workspace/blob/master/src/package_a/CMakeLists.txt#L15):
```
staticoma_export("config/base_config.yaml")
```
and B builds on top of them
(https://github.com/asherikov/staticoma_workspace/blob/master/src/package_b/CMakeLists.txt#L17)
```
staticoma_compose(
    merged_config.yaml
    package_a base_config.yaml
    package_b "config/extended_config.yaml"
)
```
`merged_config.yaml` can in turn be used by other packages
(https://github.com/asherikov/staticoma_workspace/blob/master/src/package_c/CMakeLists.txt#L28),
etc.

3. If it is necessary to prevent package B from addiing new parameters to
   configuration file provided by A, then `STRICT` keyword can be used, e.g.,
```
staticoma_compose(
    merged_config.yaml
    STRICT
    package_a base_config.yaml
    package_b "config/extended_config.yaml"
)
```

4. There exist different merging strategies for `YAML` arrays, `roslaunch`
   supports only 'overwrite' strategy, where the contents of an array is
   completely overriden if the redefined in overlaying parameters. `staticoma`
   uses the same strategy by default, but also supports 'append' strategy,
   which extends arrays with new members, e.g.
   (https://github.com/asherikov/staticoma_workspace/blob/master/src/package_c/CMakeLists.txt#L20):
```
staticoma_compose(
    merged_config.yaml
    ARRAY_MERGE_STRATEGY append
    package_a base_config.yaml
    package_b extended_config.yaml
)
```

5. Global or partial (in some cases) stack-config can be used for partial stack
   launches, which is convenient for development.

6. A dedicated service `staticoma/server` reads stack-config on launch and
   advertises it as a static topic (`stack-config-topic`), which can be
   recorded in a bag file.

7. It is not very safe to rely completely on `staticoma` server, therefore
   nodes should prefer reading stack-config directly from a file.

8. Passing of stack-config filename:
    - environment variable -- should be used by default
      (https://github.com/asherikov/staticoma/blob/master/test/client.test#L4);
    - command line argument -- can be provided by the user if necessary.

9. Initialization of stack-config in a node:
    1. if stack-config filename is given explicitly, open this file, fail if
       does not exist.
    2. if environment variable specifying stack-config filename exists, use
       it to locate the file, fail if does not exist.
    3. if stack-config filename cannot be determined try accessing
       `stack-config-topic`, fail if not available.
    4. Once the content of stack-config has been received, the node can parse
       and use it as necessary.

10. Some parameters, such as robot description, must be set in parameter
    server, so we cannot get rid of parameter server completely. `staticoma`
    server can dump all parameters and publish them in a message to be saved in
    bag files.

11. Restoring of parameter server content from a bag file should be performed
    by an additional `replayer` service that would be executed alongside
    `rosbag play`.

12. `replayer` can be started in two ways:
    - as a normal ROS node --
      https://github.com/asherikov/staticoma/blob/master/test/client_replay1.test#L7),
      in this case parameters are not guaranteed to be uploaded before other
      nodes are started;
    - inside `<param>` roslaunch tag --
      https://github.com/asherikov/staticoma/blob/master/test/client_replay3.test#L7.


### Handling launch-time and run-time parameters

1. Although launch-time parameter values are unknown at build-time, we usually
   know which services provide them, in case of roslaunch scripts this
   information is implicitly hardcoded.

2. A unique topic can be allocated for each dynamic provider known at build
   time. Publishing parameters on topics is also going to make them
   'rosbaggable'. Topic name collisions can be avoided by using naming
   conventions, e.g.,: `/<consumer_id>/<provider_id>/<topic_name>`.

3. Parameter consumer may receive parameters from multiple sources, conflict
   resolution is implementation specific, for example, overlay them in order of
   provider declaration, or instantiate a separate processing thread for each.

4. roslaunch `arg`, `param`, and `rosparam` tags can be replaced with nodes
   starting rostopic with `rostopic pub --latch` nodes, e.g.
   https://github.com/asherikov/staticoma_workspace/blob/master/src/package_b/launch/node.launch#L3.

5. Consuming parameters
   (https://github.com/asherikov/staticoma_workspace/blob/master/src/package_c/src/node.cpp#L101):
    1. obtain stack-config to identify parameter providers.
    2. wait for published parameters
    3. parse parameters


### Parameter representation

1. `staticoma` does not depend on parameter representation, it can be `JSON`,
   `YAML`, `XML`, etc, as long as we have tools to compose them at build-time
   and parse in nodes. Currently, only `YAML` is supported.

2. Configuration files are passed raw (plain YAML, etc): performance and
   throughput are not important since parameters are static in run-time and
   need not be frequently transferred.


### Other considerations

1. If we remove all conditional logic, arguments, and parameters from launch
   files, they are pretty much build-time configuration files, which can be
   composed at build-time as well. This may be useful if we would like to use
   something like `supervisord` for service management, since it uses single
   configuration file instead of multiple 'unit'/'launch' files.

2. It could be interesting to consider migration from YAML to jsonnet. Jsonnet
   is a data templating language (https://jsonnet.org/) which has build-in JSON
   merging functionality and a lot of other functions.



Related software
----------------

* https://github.com/hordurk/rosbag_metadata -- Tool for collecting and writing
  metadata to ROS bagfiles or to accompanying yaml files.
