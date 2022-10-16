# Configuration file syntax

By default the file is expected to be named `warp.yaml`

## Configuration


`version` *Optional* The artefacts yaml format specification version.

`project` The name of the associated project. Used for result tracking and authentication

`jobs` A mapping of job names to `Job` definitions, see [Job](#job)


## Job

Each Job has the following properties

`type` Defaults  to `test`

`timeout` *Optional* Time before the job gets marked as `timed out`

`runtime` Contains runtime properties, see [Runtime](#runtime)

`scenarios` One job can contain multiple scenario, usually a test suite linked to a particular environment, see [Scenario definition](#scenarios-definition)

## Scenarios definition

`defaults` contain default scenario settings common to all scenario unless overwritten by a scenario.

`scenarios` contains a list of `Scenario`, see [Scenario](#scenario)

## Runtime

Used to prepare and hook into the test environment

`framework` software framework. Supported values `ros2:0`, `other`

`simulator` simulation engine


## Scenario


`name` Name of the scenario

`run` *Optional* specify command to start tests

`output_path` *Optional* path where the CLI will look for simulation artifacts to upload.

`params` List of parameters to set for the scenario. For each parameter a list of values or a single value can be specified. Scenario variants will automatically be run for each of the parameters combination (grid strategy).

`params_output` *Optional*  File path where to save the parameters. Can have json or yaml extension. For ros2 project it will output a rosparam yaml file. *If not specified, the parameters will be set via environment variables*

`metrics` *Optional*  to specify test metrics. Accepts a json file: the key-values pairs will be used as metric_name/metric_value. ROS projects can alternatively accept a list of topic, the latest values on the topics during a run will be the logged value.

### Framework specific scenario properties

 `ros2_testfile` property is used to specify the launch_test file
