### Setup
Follow setup instructions here: [[Setup - Laptop](https://docs.duckietown.org/daffy/opmanual_duckiebot/out/laptop_setup.html)

### Build and Run

dts devel build -f -H uaaduck.local

dts devel run -H uaaduck.local

### Define dependencies

List the dependencies in the files `dependencies-apt.txt` and
`dependencies-py3.txt` (apt packages and pip packages respectively).


### Node changes

Place your code in the directory `/packages/` 



### Setup launchers

The directory `/launchers` can contain as many launchers (launching scripts)
as you want. A default launcher called `default.sh` must always be present.

If you create an executable script (i.e., a file with a valid shebang statement)
a launcher will be created for it. For example, the script file 
`/launchers/my-launcher.sh` will be available inside the Docker image as the binary
`dt-launcher-my-launcher`.

When launching a new container, you can simply provide `dt-launcher-my-launcher` as
command.
