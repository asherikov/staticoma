1. Install dependencies with `make wsinstall_ubuntu18`
2. Install `yq`, e.g., `snap install yq`, see https://mikefarah.gitbook.io/yq/ for more options.
3. Build staticoma: `make staticoma`
4. Test staticoma: `make test PKG=staticoma`
5. Build demo packages: `make package_c`
6. Launch demo package: `source ./devel/reldebug/setup.bash; roslaunch package_c node.launch`
